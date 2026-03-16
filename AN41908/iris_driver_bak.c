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

#define AN41908_CNT 1              // 设备数量
#define AN41908_NAME "AN41908"        // 设备名称


//这个驱动的 “魔法数”（一般是一个自定义字符，比如 'F'），用于区分不同驱动的 IOCTL 命令，防止指令串扰
#define AN41908_IOCTL_MAGIC 'F'

// 2. 对焦参数结构体（传递方向和步长）
struct an41908_focus_param {
    unsigned int motor_id;  // 新增：0=电机A, 1=电机B
    bool forward;  // 方向：false=对焦近，true=对焦远
    int step;      // 步长（应用层传0xff）
};

// 3. 定义ioctl命令（对焦精准控制）
#define AN41908_IOCTL_SET_IRIS     _IOW(AN41908_IOCTL_MAGIC, 0, unsigned int)
#define AN41908_IOCTL_VD_PULSE           _IO(AN41908_IOCTL_MAGIC, 1)
#define AN41908_IOCTL_FOCUS_FORWARD           _IO(AN41908_IOCTL_MAGIC, 2)
#define AN41908_IOCTL_FOCUS_REVERSE           _IO(AN41908_IOCTL_MAGIC, 3)
#define AN41908_IOCTL_FOCUS_STOP           _IO(AN41908_IOCTL_MAGIC, 4)
#define AN41908_IOCTL_ZOOM_FORWARD           _IO(AN41908_IOCTL_MAGIC, 5)
#define AN41908_IOCTL_ZOOM_REVERSE           _IO(AN41908_IOCTL_MAGIC, 6)
#define AN41908_IOCTL_ZOOM_STOP           _IO(AN41908_IOCTL_MAGIC, 7)


// 4. 最大命令编号
#define AN41908_IOCTL_MAX_NR 8

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

    struct task_struct *thread;   // 内核线程指针
    bool thread_stop;             // 线程停止标志
};

static struct AN41908_DEV my_an41908_dev;

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
    // 地址A0-A5占bit0-bit5，C0=1（读）、C1=0
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
 * @brief  驱动层实现GPIO_VD_FZ的电平翻转（高10ms→低）
 * @param  dev: 设备实例指针
 */
static void an41908_signal_gpio(struct AN41908_DEV *dev)
{
    mutex_lock(&dev->lock);  // 加锁保护
    gpiod_set_value(dev->vd_fz_gpio, 1);  // 高电平
    udelay(20);        
    gpiod_set_value(dev->vd_fz_gpio, 0);  // 低电平
    mutex_unlock(&dev->lock);
    dev_info(&dev->spi->dev, "VD_FZ上升沿已触发,芯片开始执行对焦");
}

static int AN41908_iris_init(struct AN41908_DEV *dev)
{
    int ret;
    // ret = an41908_write_reg(dev, 0x20, 0x1E03);     //设置PWM与DT1,PWM频率为112.5KHz,DT1时间约为3x303.4us~=0.9ms
    // ret = an41908_write_reg(dev, 0x22, 0x0002);     //设置电机激励等待时间2x303us=0.6ms,驱动器A与驱动器B的相位差为默认的90°
    // ret = an41908_write_reg(dev, 0x23, 0xd8d8);     //设置电机pwm占空比为90%,写0x7878的时候为50%，占空比越大驱动能力越强
    // ret = an41908_write_reg(dev, 0x24, 0x0400);     //设置电机核心运行总控，0f除以8得到一个VD_FZ周期走两个步距角，正转，关闭刹车状态，1开启关闭电机，256细分模式
    // ret = an41908_write_reg(dev, 0x25, 0x015f);     //设置步进周期
    // ret = an41908_write_reg(dev, 0x27, 0x0002);     //设置电机激励等待时间2x303us=0.6ms,驱动器A与驱动器B的相位差为默认的90°
    // ret = an41908_write_reg(dev, 0x28, 0xd8d8);     //设置电机pwm占空比为90%,写0x7878的时候为50%，占空比越大驱动能力越强
    // ret = an41908_write_reg(dev, 0x29, 0x0440);     //设置电机核心运行总控，0f除以8得到一个VD_FZ周期走两个步距角，正转，关闭刹车状态，1开启关闭电机，256细分模式
    // ret = an41908_write_reg(dev, 0x2A, 0x015f);     //设置步进周期

    ret = an41908_write_reg(dev, 0x20, 0x1e03);     //设置PWM与DT1,PWM频率为112.5KHz,DT1时间约为3x303.4us~=0.9ms

    ret = an41908_write_reg(dev, 0x22, 0x0001);     //设置电机激励等待时间2x303us=0.6ms,驱动器A与驱动器B的相位差为默认的90°
    ret = an41908_write_reg(dev, 0x23, 0x7878);     //设置电机pwm占空比为90%,写0x7878的时候为50%，占空比越大驱动能力越强
    ret = an41908_write_reg(dev, 0x24, 0xcfff);     //设置电机核心运行总控，0f除以8得到一个VD_FZ周期走两个步距角，正转，关闭刹车状态，1开启关闭电机，256细分模式
    ret = an41908_write_reg(dev, 0x25, 0x015f);     //设置步进周期

    ret = an41908_write_reg(dev, 0x27, 0x001);     //设置电机激励等待时间2x303us=0.6ms,驱动器A与驱动器B的相位差为默认的90°
    ret = an41908_write_reg(dev, 0x28, 0x7878);     //设置电机pwm占空比为90%,写0x7878的时候为50%，占空比越大驱动能力越强
    ret = an41908_write_reg(dev, 0x29, 0xcfff);     //设置电机核心运行总控，0f除以8得到一个VD_FZ周期走两个步距角，正转，关闭刹车状态，1开启关闭电机，256细分模式
    ret = an41908_write_reg(dev, 0x2A, 0x015f);     //设置步进周期
    
    /*******************************************光圈部分********************************************************************* */
    //组一
    ret = an41908_write_reg(dev, 0x00, 0x0000);     //这个是设置IRS_TGT[9:0],先写0000代表光圈全关闭
    ret = an41908_write_reg(dev, 0x01, 0x7c8a);     //这个低位先不用管都使用为0默认值，高七位是DGAIN[6:0]（PID 控制器增益），也就是PID中的P比例
    ret = an41908_write_reg(dev, 0x02, 0x66f0);     //设置PID零点 = 35Hz，极点 = 950Hz，PID_ZERO[3:0]其实就是PID中的I积分，PID_POLE[3:0]就是PID中的D微分
    ret = an41908_write_reg(dev, 0x03, 0x0e10);     //设置PID的PWM频率 = 31.25KHz
    ret = an41908_write_reg(dev, 0x04, 0x7070);     //设设置霍尔的偏置电流与偏置电压
    ret = an41908_write_reg(dev, 0x05, 0x0004);     //0024PID极性取反，0004则不取反 
    // ret = an41908_write_reg(dev, 0x0A, 0x0780);     // 驱动信号占空比，设置这个会手动控制光圈，bit9控制方向
    ret = an41908_write_reg(dev, 0x0B, 0x0480);     //00480使能光圈，00080不使能光圈 
    ret = an41908_write_reg(dev, 0x0E, 0x0300);     // 

    // //组二
    // ret = an41908_write_reg(dev, 0x00, 0x0000);     //这个是设置IRS_TGT[9:0],先写0000代表光圈全关闭
    // ret = an41908_write_reg(dev, 0x01, 0x688a);     //这个低位先不用管都使用为0默认值，高七位是DGAIN[6:0]（PID 控制器增益），也就是PID中的P比例
    // ret = an41908_write_reg(dev, 0x02, 0x66f0);     //设置PID零点 = 35Hz，极点 = 950Hz，PID_ZERO[3:0]其实就是PID中的I积分，PID_POLE[3:0]就是PID中的D微分
    // ret = an41908_write_reg(dev, 0x03, 0x0e10);     //设置PID的PWM频率 = 31.25KHz
    // ret = an41908_write_reg(dev, 0x04, 0x8028);     //设设置霍尔的偏置电流与偏置电压
    // ret = an41908_write_reg(dev, 0x05, 0x0d24);     //0024PID极性取反，0004则不取反 
    // ret = an41908_write_reg(dev, 0x0B, 0x8480);     //00480使能光圈，00080不使能光圈 
    // ret = an41908_write_reg(dev, 0x0E, 0xffff);     // 

    //组三
    // ret = an41908_write_reg(dev, 0x00, 0x0000);     //这个是设置IRS_TGT[9:0],先写0000代表光圈全关闭
    // ret = an41908_write_reg(dev, 0x01, 0x3E00);     //这个低位先不用管都使用为0默认值，高七位是DGAIN[6:0]（PID 控制器增益），也就是PID中的P比例
    // ret = an41908_write_reg(dev, 0x02, 0x1000);     //设置PID零点 = 35Hz，极点 = 950Hz，PID_ZERO[3:0]其实就是PID中的I积分，PID_POLE[3:0]就是PID中的D微分
    // ret = an41908_write_reg(dev, 0x03, 0x0E10);     //设置PID的PWM频率 = 31.25KHz
    // ret = an41908_write_reg(dev, 0x04, 0xD640);     //设设置霍尔的偏置电流与偏置电压
    // ret = an41908_write_reg(dev, 0x05, 0x0004);     //0024PID极性取反，0004则不取反 
    // ret = an41908_write_reg(dev, 0x0B, 0x0400);     //00480使能光圈，00080不使能光圈 
    // ret = an41908_write_reg(dev, 0x0E, 0x0300);     // 

    an41908_signal_gpio(dev);

    dev_info(&dev->spi->dev, "AN41908 focus and zoom motor init success!\n");

    return ret;
}

static void IC_AN41908_init(struct AN41908_DEV *dev)
{    
    mdelay(10);  
    gpiod_set_value(dev->res_gpio, 0);      
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

static ssize_t an41908_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *off) 
{
    struct AN41908_DEV *dev = filp->private_data;
    struct reg_data reg_info;
    int ret;

    mutex_lock(&dev->lock);

    if (copy_from_user(&reg_info, buf, sizeof(struct reg_data))) {
        dev_err(&dev->spi->dev, "copy_from_user failed!\n");
        ret = -EFAULT;
        goto unlock_mutex;
    }

    ret = an41908_write_reg(dev, reg_info.addr, reg_info.data);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "Write reg 0x%02X failed! ret=%d\n", reg_info.addr, ret);
        goto unlock_mutex;
    }

    dev_info(&dev->spi->dev, "Write reg success: addr=0x%02X, data=0x%04X\n",
             reg_info.addr, reg_info.data);

    ret = sizeof(struct reg_data);

unlock_mutex:
    mutex_unlock(&dev->lock);
    return ret;
}

/**
 * @brief  驱动层read函数（用户空间读取寄存器值）
 * @param  filp: 文件指针（关联设备实例）
 * @param  buf: 用户空间缓冲区（传入要读的地址，接收读取的数据）
 * @param  cnt: 用户请求读取的字节数
 * @param  off: 文件偏移（未使用，字符设备无偏移）
 * @return 成功返回读取的字节数（sizeof(struct reg_data)），负数失败
 */
static ssize_t an41908_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
    struct AN41908_DEV *dev = filp->private_data;
    struct reg_data reg_info = {0};  // 内核层寄存器数据缓存
    int ret;

    // 1. 校验用户传入的缓冲区长度：至少要能容纳struct reg_data（地址+数据）
    if (cnt < sizeof(struct reg_data)) {
        dev_err(&dev->spi->dev, "Read buf too small! need %zu, got %zu\n",
                sizeof(struct reg_data), cnt);
        return -EINVAL;
    }

    // 2. 加互斥锁保护硬件操作（防止多线程同时读写SPI）
    mutex_lock(&dev->lock);

    // 3. 从用户空间拷贝「要读取的寄存器地址」到内核空间
    if (copy_from_user(&reg_info, buf, sizeof(struct reg_data))) {
        dev_err(&dev->spi->dev, "copy_from_user failed for read!\n");
        ret = -EFAULT;
        goto unlock_mutex;
    }

    // 5. 调用底层寄存器读取函数，获取硬件值
    ret = an41908_read_reg(dev, reg_info.addr, &reg_info.data);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "Read reg 0x%02X failed! ret=%d\n", reg_info.addr, ret);
        goto unlock_mutex;
    }

    // 6. 将读取到的数据拷贝回用户空间（包含地址+读取的16位数据）
    if (copy_to_user(buf, &reg_info, sizeof(struct reg_data))) {
        dev_err(&dev->spi->dev, "copy_to_user failed for read!\n");
        ret = -EFAULT;
        goto unlock_mutex;
    }

    dev_dbg(&dev->spi->dev, "Read reg success: addr=0x%02X, data=0x%04X\n",
            reg_info.addr, reg_info.data);

    // 7. 返回成功读取的字节数（固定为struct reg_data的大小）
    ret = sizeof(struct reg_data);

unlock_mutex:
    mutex_unlock(&dev->lock);
    return ret;
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
    unsigned int iris_target;  // 光圈目标值（0~0x3FF）

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
        case AN41908_IOCTL_SET_IRIS:
        { 
            // 从用户空间读取光圈目标值
            if (copy_from_user(&iris_target, (unsigned int __user *)arg, sizeof(iris_target))) {
                dev_err(&dev->spi->dev, "copy_from_user failed for IRIS!\n");
                ret = -EFAULT;
                break;
            }

            // 光圈目标值范围保护（0~0x3FF，对应IRS_TGT[9:0]）
            if (iris_target > 0x3FF) {
                iris_target = 0x3FF;
                dev_warn(&dev->spi->dev, "IRIS target too big, limit to 0x3FF\n");
            } else if (iris_target < 0) {
                iris_target = 0;
                dev_warn(&dev->spi->dev, "IRIS target too small, limit to 0x000\n");
            }
            
            // 写入0x00寄存器（光圈目标值寄存器）
            ret = an41908_write_reg(dev, 0x00, iris_target);

            an41908_signal_gpio(dev);

            if (ret == 0) {
                dev_info(&dev->spi->dev, "Set IRIS target success: 0x%03X\n", iris_target);
            } else {
                dev_err(&dev->spi->dev, "Set IRIS target failed! ret=%d\n", ret);
            }
            break;
        }

        case AN41908_IOCTL_VD_PULSE:
        {
            an41908_signal_gpio(dev);
            break;
        }

        case AN41908_IOCTL_FOCUS_FORWARD://FOCUS正向
        {
            an41908_write_reg(dev, 0x29, 0x0540);                
            an41908_signal_gpio(dev); 
            break;
        }

        case AN41908_IOCTL_FOCUS_REVERSE://FOCUS反向
        {
            an41908_write_reg(dev, 0x29, 0x0440);    
            an41908_signal_gpio(dev);
            break;
        }

        case AN41908_IOCTL_FOCUS_STOP://FOCUS电机停止
        {
            an41908_write_reg(dev, 0x29, 0x0400);
            an41908_signal_gpio(dev);
            break;
        }

        case AN41908_IOCTL_ZOOM_FORWARD://ZOOM正向
        {
            an41908_write_reg(dev, 0x24, 0x0540);
            an41908_signal_gpio(dev);
            break;
        }

        case AN41908_IOCTL_ZOOM_REVERSE://ZOOM反向
        {
            an41908_write_reg(dev, 0x24, 0x0440); 
            an41908_signal_gpio(dev);
            break;
        }

        case AN41908_IOCTL_ZOOM_STOP://ZOOM电机停止
        {
            an41908_write_reg(dev, 0x24, 0x0400);
            an41908_signal_gpio(dev);
            break;
        }

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
    .write = an41908_write,
    .read = an41908_read,
    .unlocked_ioctl = an41908_ioctl,  
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
    gpiod_set_value(dev->vd_fz_gpio, 0);  

    mutex_init(&dev->lock);     

	//7.初始化SPI设备
    spi->mode = SPI_MODE_0 | SPI_LSB_FIRST; //要设置低位先行
    spi->max_speed_hz = 4000000;
    spi_setup(spi);
    dev->private_data = spi;
    dev->spi = spi;

    //8.复位芯片
    IC_AN41908_init(dev);

    ret = AN41908_iris_init(dev);
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

        // 停止线程
    if (dev->thread) {
        dev->thread_stop = true;
        kthread_stop(dev->thread);
        dev->thread = NULL;
    }

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

/* SPI设备ID表（匹配传统非设备树场景，和设备树compatible对应） */
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