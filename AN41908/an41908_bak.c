#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include <linux/mutex.h>

// 寄存器读写控制位
#define AN41908_WRITE_CMD 0x00  // C0=0（写模式）
#define AN41908_READ_CMD  0x40  // C0=1（读模式，bit6置1）
#define AN41908_C1_UNUSED 0x00  // C1=0（未使用）

#define AN41908_CNT 1              // 设备数量
#define AN41908_NAME "AN41908"        // 设备名称

// ========== 新增：ioctl 相关定义（仅对焦） ==========
// 1. 魔数（唯一标识，选'F'代表Focus）
#define AN41908_IOCTL_MAGIC 'F'

// 2. 对焦参数结构体（传递方向和步长）
struct an41908_focus_param {
    bool forward;  // 方向：false=对焦近，true=对焦远
    int step;      // 步长（应用层传0xff）
};

// 3. 定义ioctl命令（对焦精准控制）
#define AN41908_IOCTL_SET_FOCUS_PRECISE _IOW(AN41908_IOCTL_MAGIC, 0, struct an41908_focus_param)

// 4. 最大命令编号
#define AN41908_IOCTL_MAX_NR 0

// 寄存器数据结构体（传递地址和数据）
struct reg_data {
    uint8_t addr;    // 寄存器地址（0x00~0x2B）
    uint16_t data;   // 16位数据（写：要写入的值；读：读取到的值）
};

struct AN41908_DEV {
	struct spi_device *spi;
	dev_t devid;				/* 设备号 	 */
	struct cdev cdev;			/* cdev 	*/
	struct class *class;		/* 类 		*/
	struct device *device;		/* 设备 	 */
	struct device_node	*nd; 	/* 设备节点 */
	int major;					/* 主设备号 */
	void *private_data;			/* 私有数据 		*/
    struct gpio_desc *res_gpio; /* 复位 GPIO 描述符 */
    struct gpio_desc *vd_fz_gpio; /* 新增：GPIO_VD_FZ 描述符 */
    struct mutex lock;          /* 新增：硬件操作互斥锁 */
};

static struct AN41908_DEV my_an41908_dev;

// /**
//  * @brief  写AN41908寄存器
//  * @param  dev: 设备实例指针
//  * @param  addr: 寄存器地址（0x00~0x2B）
//  * @param  data: 要写入的16位数据
//  * @return 0=成功，负数=失败
//  */
// static int an41908_write_reg(struct AN41908_DEV *dev, uint8_t addr, uint16_t data)
// {
//     int ret;
//     uint8_t tx_buf[3];  // 24位数据拆分为3个字节（bit0~bit23）
    
//     // 步骤1：构造24位数据（按手册位顺序）
//     tx_buf[0] = (0 << 7) | (0 << 6) | (addr & 0x3F); // bit7=C1, bit6=C0, bit5-0=ADDR
//     tx_buf[1] = data & 0xFF;  // bit8~bit15：D0~D7（数据低8位）
//     tx_buf[2] = (data >> 8) & 0xFF;  // bit16~bit23：D8~D15（数据高8位）

//     // 步骤2：SPI传输（自动拉低/拉高CS）
//     struct spi_transfer t = {
//         .tx_buf = tx_buf,
//         .len = 3,  // 传输3字节=24位
//         .speed_hz = dev->spi->max_speed_hz,
//         .bits_per_word = 8,
//     };

//     ret = spi_sync_transfer(dev->spi, &t, 1);
//     if (ret < 0) {
//         dev_err(&dev->spi->dev, "SPI write reg failed! addr=0x%02X, ret=%d\n", addr, ret);
//         return ret;
//     }

//     return 0;
// }

static int an41908_write_reg(struct AN41908_DEV *dev, uint8_t addr, uint16_t data)
{
    int ret;
    uint8_t tx_buf[3];
    
    // 构造写命令（24位）
    // 第一个字节：A0-A5, C0=0(写), C1=0
    tx_buf[0] = (addr & 0x3F);  // C0=0, C1=0
    
    // 第二、三个字节：数据
    tx_buf[1] = data & 0xFF;        // D0-D7
    tx_buf[2] = (data >> 8) & 0xFF; // D8-D15
    
    // SPI传输
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .len = 3,
        .speed_hz = dev->spi->max_speed_hz,
        .bits_per_word = 8,
    };
    
    ret = spi_sync_transfer(dev->spi, &t, 1);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "SPI write reg failed! addr=0x%02X, ret=%d\n", addr, ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief 从AN41908读取寄存器（LSB先行、SPI模式0）
 * @param dev: 设备结构体指针
 * @param addr: 寄存器地址（A0-A5，6位）
 * @param data: 读取的16位数据
 * @return 0成功，负数失败
 */
static int an41908_read_reg(struct AN41908_DEV *dev, uint8_t addr, uint16_t *data)
{
    int ret;
    uint8_t tx_buf[3] = {0};  // 发送缓冲区：命令字节 + 2字节填充
    uint8_t rx_buf[3] = {0};  // 接收缓冲区：2字节填充的回显 + 2字节数据

    /* 1. 构造读命令（地址 + 读控制位） */
    // 地址A0-A5占bit0-bit5，C0=1（读）、C1=0（保留位，可根据手册调整）
    tx_buf[0] = (addr & 0x3F) | (1 << 6);  // bit6=1（C0=1，读操作）

    /* 2. SPI传输：发送命令+填充，同时接收数据 */
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 3,              // 发送3字节（命令+2填充），接收3字节
        .speed_hz = dev->spi->max_speed_hz,
        .bits_per_word = 8,
    };

    ret = spi_sync_transfer(dev->spi, &t, 1);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "读寄存器失败（addr=0x%02X）: %d\n", addr, ret);
        return ret;
    }

    /* 3. 解析接收到的数据（低8位在前，高8位在后） */
    // rx_buf[1]：从机返回的低8位（D0-D7）
    // rx_buf[2]：从机返回的高8位（D8-D15）
    *data = (rx_buf[2] << 8) | rx_buf[1];

    dev_dbg(&dev->spi->dev, "读寄存器成功: addr=0x%02X, data=0x%04X\n", addr, *data);
    return 0;
}




/**
 * @brief  驱动层实现GPIO_VD_FZ的电平翻转（高10ms→低）
 * @param  dev: 设备实例指针
 */
static void an41908_signal_gpio(struct AN41908_DEV *dev)
{
    mutex_lock(&dev->lock);  // 加锁保护
    gpiod_set_value(dev->vd_fz_gpio, 1);  // 高电平
    //mdelay(10);                           // 10ms
    ndelay(250);        //500ns
    gpiod_set_value(dev->vd_fz_gpio, 0);  // 低电平
    mutex_unlock(&dev->lock);
    dev_info(&dev->spi->dev, "VD_FZ上升沿已触发,芯片开始执行对焦");
}

/**
 * @brief  驱动层实现turn_on（发送开启电机的SPI指令）
 * @param  dev: 设备实例指针
 * @return 0=成功，负数=失败
 */
static int an41908_turn_on(struct AN41908_DEV *dev)
{
    int ret;
    mutex_lock(&dev->lock);

    ret = an41908_write_reg(dev, 0x24, 0x040f);     //2. α电机基础控制（配置的0x040f：256细分+使能1+刹车0+PSUMAB=0f+转向0，基础值）
    if (ret < 0) {
        dev_err(&dev->spi->dev, "write reg 0x24 failed! ret=%d\n", ret);
        goto out;
    }

    // 3. α电机步周期（你配置的0x1ab5，固定转速，全程不变）
    //ret = an41908_write_reg(dev, 0x25, 0x1ab5);//在255细分模式下，1ab5x111=758907ns=0.759ms
    ret = an41908_write_reg(dev, 0x25, 0x015F);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "write reg 0x25 failed! ret=%d\n", ret);
        goto out;
    }

out:
    mutex_unlock(&dev->lock);
    return ret;
}

/**
 * @brief  驱动层实现turn_off（发送关闭电机的SPI指令）
 * @param  dev: 设备实例指针
 * @return 0=成功，负数=失败
 */
static int an41908_turn_off(struct AN41908_DEV *dev)
{
    int ret;
    // 停止值：24h基础值0x040f → 清零D0~D7（PSUMAB=0），其他位不变→0x0400
    uint16_t reg24_stop = 0x0400;

    mutex_lock(&dev->lock);

    // 写入停止配置：PSUMAB=0（芯片官方停止指令，电机平稳停转）
    ret = an41908_write_reg(dev, 0x24, reg24_stop);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "关闭电机写入24h失败!ret=%d\n", ret);
        goto out;
    }

    dev_info(&dev->spi->dev, "电机已安全关闭(PSUMAB=0)");

out:
    mutex_unlock(&dev->lock);
    return ret;
}

/**
 * @brief  驱动层实现send_focus_cmd（发送对焦指令）
 * @param  dev: 设备实例指针
 * @param  forward: 方向（false=对焦近，true=对焦远）
 * @param  step: 步长（应用层传0xff）
 * @return 0=成功，负数=失败
 */
static int an41908_send_zoom_cmd(struct AN41908_DEV *dev, bool forward, int step)
{
    int ret; 
    uint16_t reg24_base = 0x040f;   // 基础值：复用turn_on里的24h配置（0x040f），保证高8位核心配置不变
    uint16_t dir_val = forward ? 1 : 0;     // forward映射为转向值：false=0（对焦近），true=1（对焦远）→ 对应24h.D8位
    uint16_t reg24_final = 0;       // 最终要写入的24h值（基础值+动态修改的方向+步长）

    mutex_lock(&dev->lock);

    // 【关键】步长合法性校验：必须1~255（0=电机不转，>255=8位寄存器溢出）
    // 精准对焦建议step=1~0x30（1~48），步长越小精度越高，避免0xff大步长丢步
        if (step < 1 || step > 0xFF) {
        dev_err(&dev->spi->dev, "step参数非法!step=%d,必须1~255\n", step);
        ret = -EINVAL;
        goto out;
    }

    // 核心位操作：只改24h.D0~D8，高8位（D9~D15）保持不变
    reg24_final = reg24_base;          // 先拷贝基础值
    reg24_final &= ~0x01FF;            // 清零D0~D8位（步长位+转向位），不碰其他位
    reg24_final |= (step & 0x00FF);    // 写入新步长→D0~D7位（PSUMAB[7:0]）
    reg24_final |= (dir_val << 8);     // 写入新方向→D8位（CCWCWAB）

    // 写入修改后的24h寄存器：这是本函数唯一的寄存器操作
    ret = an41908_write_reg(dev, 0x24, reg24_final);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "动态写入24h寄存器失败!ret=%d\n", ret);
        goto out;
    }

    // 调试信息：打印最终配置，方便排查问题（可选，建议保留）
    dev_info(&dev->spi->dev, "对焦参数已配置：方向=%s,步长=%d，24h最终值=0x%04x",
             forward ? "远" : "近", step, reg24_final);

out:
    mutex_unlock(&dev->lock);
    return ret;
}

/**
 * @brief  驱动层实现精准对焦（对焦近/远）
 * @param  dev: 设备实例指针
 * @param  forward: 方向（false=对焦近，true=对焦远）
 * @param  step: 步长（应用层传0xff）
 * @return 0=成功，负数=失败
 */
static int an41908_set_zoom_precise(struct AN41908_DEV *dev, bool forward, int step)
{
    int ret;

    // 1. 开启电机
    ret = an41908_turn_on(dev);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "turn_on failed! ret=%d\n", ret);
        return ret;
    }

    // 2. 发送对焦指令
    ret = an41908_send_zoom_cmd(dev, forward, step);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "send_focus_cmd failed! ret=%d\n", ret);
        an41908_turn_off(dev);  // 失败也要关闭电机
        return ret;
    }

    // 3. 触发GPIO_VD_FZ
    an41908_signal_gpio(dev);
      
    // 4. 关闭电机
    // ret = an41908_turn_off(dev);
    // if (ret < 0) {
    //     dev_err(&dev->spi->dev, "turn_off failed! ret=%d\n", ret);
    //     return ret;
    // }

    dev_info(&dev->spi->dev, "focus precise success! forward=%d, step=%d\n", forward, step);
    return 0;
}

static int AN41908_A_FOCUS_motor_init(struct AN41908_DEV *dev)
{
    int ret;
    uint16_t read_date;
    ret = an41908_write_reg(dev, 0x20, 0x1E03);     //设置PWM与DT1,PWM频率为112.5KHz,DT1时间约为3x303.4us~=0.9ms
    if (ret < 0) return ret;

    an41908_read_reg(dev,0x20,&read_date);
    ret = an41908_write_reg(dev, 0x21, 0x0087);     //设置PLS1 PLS2输出信号类型

    ret = an41908_write_reg(dev, 0x22, 0x0002);     //设置电机激励等待时间2x303us=0.6ms,驱动器A与驱动器B的相位差为默认的90°
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x23, 0xd8d8);     //设置电机pwm占空比为90%,写0x7878的时候为50%，占空比越大驱动能力越强
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x24, 0x040f);     //设置电机核心运行总控，0f除以8得到一个VD_FZ周期走两个步距角，正转，关闭刹车状态，1开启关闭电机，256细分模式
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x27, 0x0002);     //设置电机激励等待时间2x303us=0.6ms,驱动器A与驱动器B的相位差为默认的90°
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x28, 0xd8d8);     //设置电机pwm占空比为90%,写0x7878的时候为50%，占空比越大驱动能力越强
    if (ret < 0) return ret;

     ret = an41908_write_reg(dev, 0x29, 0x040f);     //设置电机核心运行总控，0f除以8得到一个VD_FZ周期走两个步距角，正转，关闭刹车状态，1开启关闭电机，256细分模式
    if (ret < 0) return ret;

    dev_info(&dev->spi->dev, "AN41908 focus and zoom motor init success!\n");

    return 0;
}

static void IC_AN41908_init(struct AN41908_DEV *dev)
{    
    mdelay(10);  
    gpiod_set_value(dev->res_gpio, 0);      // 拉低复位引脚（进入硬件复位）
    mdelay(10);
    gpiod_set_value(dev->res_gpio, 1); 
    mdelay(10); 
}

static int AN41908_open(struct inode *inode, struct file *filp)
{
    // 将设备实例指针存入file结构体，后续write/release可直接获取
    filp->private_data = &my_an41908_dev;
    return 0;
}

/**
 * @brief  ioctl处理函数（仅处理对焦命令）
 * @param  filp: 文件指针
 * @param  cmd: ioctl命令
 * @param  arg: 用户层传递的参数
 * @return 0=成功，负数=失败
 */
static long an41908_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct AN41908_DEV *dev = filp->private_data;
    int ret = 0;
    struct an41908_focus_param param;

    // 1. 校验命令合法性
    if (_IOC_TYPE(cmd) != AN41908_IOCTL_MAGIC) {
        dev_err(&dev->spi->dev, "ioctl magic error! cmd=0x%x\n", cmd);
        return -ENOTTY;
    }
    if (_IOC_NR(cmd) > AN41908_IOCTL_MAX_NR) {
        dev_err(&dev->spi->dev, "ioctl nr error! nr=%d\n", _IOC_NR(cmd));
        return -ENOTTY;
    }

    // 2. 处理对焦命令
    switch (cmd) {
        case AN41908_IOCTL_SET_FOCUS_PRECISE:
            // 从用户层拷贝参数（安全操作）
            if (copy_from_user(&param, (void __user *)arg, sizeof(param))) {
                ret = -EFAULT;
                break;
            }
            // 调用驱动层对焦函数
            ret = an41908_set_zoom_precise(dev, param.forward, param.step);
            break;

        default:
            ret = -ENOTTY;
            break;
    }

    return ret;
}

/* 文件操作集合 */
static const struct file_operations an41908_fops = {
    .owner = THIS_MODULE,
    .open = AN41908_open,
    .unlocked_ioctl = an41908_ioctl,  // 新增：绑定ioctl处理函数
};

static int AN41908_probe(struct spi_device *spi)
{
	int ret;
	struct AN41908_DEV *dev = &my_an41908_dev;

	// 1. 注册字符设备号
    //字符设备编号（主设备号 + 次设备号，唯一标识一个字符设备）
    if (dev->major == 0) {
        ret = alloc_chrdev_region(&dev->devid, 0, AN41908_CNT, AN41908_NAME);     //动态分配设备号
    } else {
        dev->devid = MKDEV(dev->major, 0);
        ret = register_chrdev_region(dev->devid, AN41908_CNT, AN41908_NAME);      //手动分配设备号
    }
    if (ret < 0) {
        dev_err(&spi->dev, "Register chrdev failed! ret=%d\n", ret);
        goto err_chrdev;
    }
    dev->major = MAJOR(dev->devid);

	    // 2. 初始化并添加cdev
    //字符设备核心结构体（连接设备号和设备操作函数集spi_oled_fops）
    cdev_init(&dev->cdev, &an41908_fops);
    ret = cdev_add(&dev->cdev, dev->devid, AN41908_CNT);
    if (ret < 0) {
        dev_err(&spi->dev, "Add cdev failed! ret=%d\n", ret);
        goto err_cdev_add;
    }
    
    // 3. 创建设备类
    //用于在/sys/class下创建设备类，自动生成设备节点
    dev->class = class_create(THIS_MODULE, AN41908_NAME);
    if (IS_ERR(dev->class)) {
        ret = PTR_ERR(dev->class);
        dev_err(&spi->dev, "Create class failed! ret=%d\n", ret);
        goto err_class;
    }
    
    // 4. 创建设备实例
    //用于在/dev下创建设备文件，用户态可通过该文件操作设备
    dev->device = device_create(dev->class, NULL, dev->devid, NULL, AN41908_NAME);
    if (IS_ERR(dev->device)) {
        ret = PTR_ERR(dev->device);
        dev_err(&spi->dev, "Create device failed! ret=%d\n", ret);
        goto err_device;
    }

    //5.设备树节点指针（获取设备树中定义的 GPIO、SPI 等配置信息）
    dev->nd = spi->dev.of_node;
    if (!dev->nd) {
        dev_err(&spi->dev, "Failed to get device node!\n");
        ret = -ENOENT;
        goto err_node;
    }

    // 6. 获取GPIO
    dev->res_gpio = devm_gpiod_get_from_of_node(&spi->dev, dev->nd, "reset-gpios", 0, GPIOD_OUT_HIGH, "reset");
    if (IS_ERR(dev->res_gpio)) {
        ret = PTR_ERR(dev->res_gpio);
        dev_err(&spi->dev, "Failed to get reset-gpios: %d\n", ret);
        goto err_node;
    }
    gpiod_set_value(dev->res_gpio, 1);

    // 获取GPIO_VD_FZ
    dev->vd_fz_gpio = devm_gpiod_get_from_of_node(&spi->dev, dev->nd, "vd-fz-gpios", 0, GPIOD_OUT_LOW, "vd-fz");
    if (IS_ERR(dev->vd_fz_gpio)) {
        ret = PTR_ERR(dev->vd_fz_gpio);
        dev_err(&spi->dev, "Failed to get vd-fz-gpios: %d\n", ret);
        goto err_node;
    }
    gpiod_set_value(dev->vd_fz_gpio, 0);  // 初始化为低电平

    mutex_init(&dev->lock);     // 新增：初始化互斥锁

	//7.初始化SPI设备
    spi->mode = SPI_MODE_0 | SPI_LSB_FIRST; //要设置低位先行
    spi->max_speed_hz = 4000000;
    spi_setup(spi);
    dev->private_data = spi;
    dev->spi = spi;

    //8.复位芯片
    IC_AN41908_init(dev);

    ret = AN41908_A_FOCUS_motor_init(dev);
    if (ret != 0) {
        goto err_motor_init;
    }


    dev_info(&spi->dev, "AN41908 probe success! Major=%d, SPI mode=%d, Speed=%dHz\n",
             dev->major, spi->mode, spi->max_speed_hz);
	return 0;

//错误处理
err_motor_init:
err_node:
    device_destroy(dev->class, dev->devid);
err_device:
    class_destroy(dev->class);
err_class:
    cdev_del(&dev->cdev);
err_cdev_add:
    unregister_chrdev_region(dev->devid, AN41908_CNT);
err_chrdev:
    return ret;	
}

static void  AN41908_remove(struct spi_device *spi)
{
	struct AN41908_DEV *dev = &my_an41908_dev;

	// 删除字符设备
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devid, AN41908_CNT);
    device_destroy(dev->class, dev->devid);
    class_destroy(dev->class);
    dev_info(&spi->dev, "AN41908 remove success\n");
}

/* 设备树匹配列表 */
static const struct of_device_id AN41908_of_match[] = {
	{ .compatible = "panasonic,an41908" },
	{ /* Sentinel */ }
};

/* 新增：SPI设备ID表（匹配传统非设备树场景，和设备树compatible对应） */
static const struct spi_device_id AN41908_spi_id[] = {
    { "panasonic,an41908", 0 },  // 名字和设备树compatible一致
    { /* Sentinel */ }
};
MODULE_DEVICE_TABLE(spi, AN41908_spi_id);  // 内核导出ID表

/*AN41908驱动结构体*/
static struct spi_driver AN41908_driver = {
	.probe = AN41908_probe,
	.remove = AN41908_remove,
    .id_table = AN41908_spi_id,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "an41908_spi",
		   	.of_match_table = AN41908_of_match, 
		   },
};

static int __init AN41908_init(void)
{
	return spi_register_driver(&AN41908_driver);
}

static void __exit AN41908_exit(void)
{
	spi_unregister_driver(&AN41908_driver);
}

module_init(AN41908_init);
module_exit(AN41908_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("PANGWEI");