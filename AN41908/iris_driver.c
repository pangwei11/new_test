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
#include <linux/moduleparam.h>

/* ===================== 配置宏定义 ===================== */
#define MS41908_CNT              1               // 单设备
#define MS41908_NAME            "MS41908"       // 设备名称
#define MS41908_IOCTL_MAGIC     'F'             // IOCTL魔法数
#define MS41908_SPI_SPEED   4000000         // SPI默认最大速率
#define MS41908_IRIS_MAX_VAL    0x3FF           // 光圈最大有效值
#define MS41908_VD_PULSE_US     20              // VD_FZ脉冲宽度(us)

// 3. 定义ioctl命令
#define MS41908_IOCTL_SET_IRIS     _IOW(MS41908_IOCTL_MAGIC, 0, unsigned int)
#define MS41908_IOCTL_VD_PULSE           _IO(MS41908_IOCTL_MAGIC, 1)
#define MS41908_IOCTL_ZOOM_FORWARD           _IO(MS41908_IOCTL_MAGIC, 2)
#define MS41908_IOCTL_ZOOM_REVERSE           _IO(MS41908_IOCTL_MAGIC, 3)
#define MS41908_IOCTL_ZOOM_STOP           _IO(MS41908_IOCTL_MAGIC, 4)
#define MS41908_IOCTL_FOCUS_FORWARD           _IO(MS41908_IOCTL_MAGIC, 5)
#define MS41908_IOCTL_FOCUS_REVERSE           _IO(MS41908_IOCTL_MAGIC, 6)
#define MS41908_IOCTL_FOCUS_STOP           _IO(MS41908_IOCTL_MAGIC, 7)
#define MS41908_IOCTL_MAX_NR       7               // 最大命令号为7（0-7）

/* ===================== 寄存器地址宏定义 ===================== */
#define MS41908_REG_IRIS_TGT      0x00    // 光圈目标值
#define MS41908_REG_DGAIN         0x01    // PID比例增益
#define MS41908_REG_PID_PARAM     0x02    // PID零点/极点
#define MS41908_REG_PWM_FREQ      0x03    // PWM频率
#define MS41908_REG_HALL_BIAS     0x04    // 霍尔偏置
#define MS41908_REG_PID_POLARITY  0x05    // PID极性
#define MS41908_REG_IRIS_DUTY     0x0A    // 光圈驱动占空比
#define MS41908_REG_IRIS_EN       0x0B    // 光圈使能
#define MS41908_REG_IRIS_CFG      0x0E    // 光圈配置
#define MS41908_REG_PWM_DT1       0x20    // PWM/DT1配置
#define MS41908_REG_MOTOR_A_WAIT  0x22    // 电机A激励等待时间
#define MS41908_REG_MOTOR_A_DUTY  0x23    // 电机A占空比
#define MS41908_REG_MOTOR_A_CTRL  0x24    // 电机A（对焦）控制
#define MS41908_REG_MOTOR_A_PERIOD 0x25   // 电机A步进周期
#define MS41908_REG_MOTOR_B_WAIT  0x27    // 电机B激励等待时间
#define MS41908_REG_MOTOR_B_DUTY  0x28    // 电机B占空比
#define MS41908_REG_MOTOR_B_CTRL  0x29    // 电机B（变焦）控制
#define MS41908_REG_MOTOR_B_PERIOD 0x2A   // 电机B步进周期

/* ===================== 电机控制参数宏定义 ===================== */
#define MS41908_MOTOR_DUTY_50PCT  0x7878  // 50%占空比
#define MS41908_MOTOR_WAIT_1STEP  0x0001  // 1*303us等待时间
#define MS41908_MOTOR_PERIOD_DEF  0x015F  // 默认步进周期
#define MS41908_MOTOR_FOCUS_FWD   0x0540  // 对焦正转
#define MS41908_MOTOR_FOCUS_REV   0x0440  // 对焦反转
#define MS41908_MOTOR_FOCUS_STOP  0x0400  // 对焦停止
#define MS41908_MOTOR_ZOOM_FWD    0x0540  // 变焦正转
#define MS41908_MOTOR_ZOOM_REV    0x0440  // 变焦反转
#define MS41908_MOTOR_ZOOM_STOP   0x0400  // 变焦停止

/* ===================== 寄存器初始化配置表（提高可维护性） ===================== */
static const struct reg_init {
    uint8_t addr;
    uint16_t data;
} ms41908_reg_init_list[] = {
    {MS41908_REG_PWM_DT1,        0x1e03},
    {MS41908_REG_MOTOR_A_WAIT,   0x0001},
    {MS41908_REG_MOTOR_A_DUTY,   0x7878},
    {MS41908_REG_MOTOR_A_CTRL,   0xcfff},
    {MS41908_REG_MOTOR_A_PERIOD, 0x015F},
    {MS41908_REG_MOTOR_B_WAIT,   0x0001},
    {MS41908_REG_MOTOR_B_DUTY,   0x7878},
    {MS41908_REG_MOTOR_B_CTRL,   0xcfff},
    {MS41908_REG_MOTOR_B_PERIOD, 0x015F},
    {MS41908_REG_IRIS_TGT,       0x0000},
    {MS41908_REG_DGAIN,          0x7c8a},
    {MS41908_REG_PID_PARAM,      0x66f0},
    {MS41908_REG_PWM_FREQ,       0x0e10},
    {MS41908_REG_HALL_BIAS,      0x7070},
    {MS41908_REG_PID_POLARITY,   0x0004},
    {MS41908_REG_IRIS_EN,        0x0480},
    {MS41908_REG_IRIS_CFG,       0x0300},
};

// 寄存器数据结构体（传递地址和数据）
struct reg_data {
    uint8_t addr;    // 寄存器地址
    uint16_t data;   // 16位数据
};

/* ===================== 设备结构体 ===================== */
struct MS41908_DEV {
	struct spi_device *spi;
	dev_t devid;				/* 设备号 	 */
	struct cdev cdev;			/* cdev 	*/
	struct class *class;		/* 类 		*/
	struct device *device;		/* 设备 	 */
	struct device_node	*nd; 	/* 设备节点 */
	int major;					/* 主设备号 */
	void *private_data;			/* 私有数据 		*/
    struct gpio_desc *res_gpio; /* 复位 GPIO 描述符 */
    struct gpio_desc *vd_fz_gpio; /* GPIO_VD 描述符 */
    struct mutex lock;          /* 新增：硬件操作互斥锁 */
};

static struct MS41908_DEV my_ms41908_dev;

/**
 * @brief 从ms41908读取寄存器（LSB先行、SPI模式0）
 * @param dev: 设备结构体指针
 * @param addr: 寄存器地址（A0-A5，6位）
 * @param data: 读取的16位数据
 * @return 0成功，负数失败
 */
static int ms41908_read_reg(struct MS41908_DEV *dev, uint8_t addr, uint16_t *data)
{
    int ret;
    uint8_t tx_buf[3] = {0};  // 发送缓冲区：命令字节 + 2字节填充
    uint8_t rx_buf[3] = {0};  // 接收缓冲区：2字节填充的回显 + 2字节数据
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 3,              // 发送3字节（命令+2填充），接收3字节
        .speed_hz = dev->spi->max_speed_hz,
        .bits_per_word = 8,
    };

    /* 1. 构造读命令（地址 + 读控制位） */
    tx_buf[0] = (addr & 0x3F) | (1 << 6); 

    ret = spi_sync_transfer(dev->spi, &t, 1);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "读寄存器失败（addr=0x%02X）: %d\n", addr, ret);
        return ret;
    }

    /* 3. 解析接收到的数据（低8位在前，高8位在后） */
    *data = (rx_buf[2] << 8) | rx_buf[1];

    dev_dbg(&dev->spi->dev, "读寄存器成功: addr=0x%02X, data=0x%04X\n", addr, *data);
    return 0;
}

/**
 * @brief 向ms41908写入寄存器
 * @param dev: 设备结构体指针
 * @param addr: 寄存器地址
 * @param data: 要写入的16位数据
 * @return 0成功，负数失败
 */
static int ms41908_write_reg(struct MS41908_DEV *dev, uint8_t addr, uint16_t data)
{
    int ret;
    uint8_t tx_buf[3];
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .len = 3,
        .speed_hz = dev->spi->max_speed_hz,
        .bits_per_word = 8,
    };
    
    // 构造写命令（24位）
    tx_buf[0] = (addr & 0x3F);  
    // 第二、三个字节：数据
    tx_buf[1] = data & 0xFF;        // D0-D7
    tx_buf[2] = (data >> 8) & 0xFF; // D8-D15
    
    ret = spi_sync_transfer(dev->spi, &t, 1);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "SPI write reg failed! addr=0x%02X, ret=%d\n", addr, ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief  驱动层实现GPIO_VD_FZ的电平翻转
 * @param  dev: 设备实例指针
 */
static void ms41908_vd_signal(struct MS41908_DEV *dev)
{
    if (!dev || !dev->vd_fz_gpio) {
        dev_err(dev ? &dev->spi->dev : NULL, "Invalid VD_FZ GPIO\n");
        return;
    }

    gpiod_set_value(dev->vd_fz_gpio, 1);  // 高电平
    udelay(MS41908_VD_PULSE_US);        
    gpiod_set_value(dev->vd_fz_gpio, 0);  // 低电平

}

/**
 * @brief 初始化MS41908寄存器配置
 * @param dev: 设备实例指针
 * @return 0成功，负数失败
 */
static int ms41908_init_reg(struct MS41908_DEV *dev)
{
    int ret, i;
    int init_count = ARRAY_SIZE(ms41908_reg_init_list);

    for (i = 0; i < init_count; i++) {
        ret = ms41908_write_reg(dev,
                               ms41908_reg_init_list[i].addr,
                               ms41908_reg_init_list[i].data);
        if (ret < 0) {
            dev_err(&dev->spi->dev, "Init reg 0x%02X failed at index %d\n",
                    ms41908_reg_init_list[i].addr, i);
            return ret;
        }
    }

    ms41908_vd_signal(dev);
    dev_info(&dev->spi->dev, "MS41908 register init success (total %d regs)\n", init_count);

    return 0;
}

/**
 * @brief 复位MS41908芯片
 * @param dev: 设备实例指针
 */
static void ms41908_reset(struct MS41908_DEV *dev)
{    
    if (!dev || !dev->res_gpio) {
        dev_err(&dev->spi->dev, "Invalid reset GPIO\n");
        return;
    }

    mdelay(10);  
    gpiod_set_value(dev->res_gpio, 0);      
    mdelay(10);
    gpiod_set_value(dev->res_gpio, 1); 
    mdelay(10); 

    dev_dbg(&dev->spi->dev, "MS41908 reset done\n");
}

/**
 * @brief 设备文件打开函数
 */
static int ms41908_open(struct inode *inode, struct file *filp)
{
    // 将设备实例指针存入file结构体，后续write/release可直接获取
    filp->private_data = &my_ms41908_dev;
    return 0;
}

/**
 * @brief 设备文件写函数（写入寄存器）
 */
static ssize_t ms41908_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *off) 
{
    struct MS41908_DEV *dev = filp->private_data;
    struct reg_data reg_info;
    int ret;

    mutex_lock(&dev->lock);

    if (copy_from_user(&reg_info, buf, sizeof(struct reg_data))) {
        dev_err(&dev->spi->dev, "copy_from_user failed!\n");
        ret = -EFAULT;
        goto unlock_mutex;
    }

    ret = ms41908_write_reg(dev, reg_info.addr, reg_info.data);
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
static ssize_t ms41908_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
    struct MS41908_DEV *dev = filp->private_data;
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
    ret = ms41908_read_reg(dev, reg_info.addr, &reg_info.data);
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
static long ms41908_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct MS41908_DEV *dev = filp->private_data;
    int ret = 0;
    unsigned int iris_target;  // 光圈目标值（0~0x3FF）

    // 1. 校验命令合法性
    if (_IOC_TYPE(cmd) != MS41908_IOCTL_MAGIC) {
        dev_err(&dev->spi->dev, "ioctl magic error! cmd=0x%x\n", cmd);
        return -ENOTTY;
    }
    if (_IOC_NR(cmd) > MS41908_IOCTL_MAX_NR) {
        dev_err(&dev->spi->dev, "ioctl nr error! nr=%d\n", _IOC_NR(cmd));
        return -ENOTTY;
    }

    mutex_lock(&dev->lock);
    switch (cmd) {
        case MS41908_IOCTL_SET_IRIS:
        { 
            if (copy_from_user(&iris_target, (unsigned int __user *)arg, sizeof(iris_target))) {
                dev_err(&dev->spi->dev, "copy_from_user failed for IRIS!\n");
                ret = -EFAULT;
                break;
            }

            /* 范围限制 */
            iris_target = clamp_val(iris_target, 0, MS41908_IRIS_MAX_VAL);
            
            // 写入0x00寄存器（光圈目标值寄存器）
            ret = ms41908_write_reg(dev, MS41908_REG_IRIS_TGT, iris_target);
            if (ret == 0) {
                ms41908_vd_signal(dev);
                dev_info(&dev->spi->dev, "Set IRIS target to 0x%03X\n", iris_target);
            }
            break;
        }

        case MS41908_IOCTL_VD_PULSE:
        {
            ms41908_vd_signal(dev);
            dev_dbg(&dev->spi->dev, "VD_FZ pulse generated\n");
            break;
        }

        case MS41908_IOCTL_ZOOM_FORWARD:
        {
            ret = ms41908_write_reg(dev, MS41908_REG_MOTOR_B_CTRL, MS41908_MOTOR_ZOOM_FWD);
            if (ret == 0) ms41908_vd_signal(dev);
            break;
        }

        case MS41908_IOCTL_ZOOM_REVERSE:
        {
            ret = ms41908_write_reg(dev, MS41908_REG_MOTOR_B_CTRL, MS41908_MOTOR_ZOOM_REV);
            if (ret == 0) ms41908_vd_signal(dev);
            break;
        }

        case MS41908_IOCTL_ZOOM_STOP:
        {
            ret = ms41908_write_reg(dev, MS41908_REG_MOTOR_B_CTRL, MS41908_MOTOR_ZOOM_STOP);
            if (ret == 0) ms41908_vd_signal(dev);
            break;
        }

        case MS41908_IOCTL_FOCUS_FORWARD:
        {
            ret = ms41908_write_reg(dev, MS41908_REG_MOTOR_A_CTRL, MS41908_MOTOR_FOCUS_FWD);
            if (ret == 0) ms41908_vd_signal(dev);
            break;
        }

        case MS41908_IOCTL_FOCUS_REVERSE:
        {
            ret = ms41908_write_reg(dev, MS41908_REG_MOTOR_A_CTRL, MS41908_MOTOR_FOCUS_REV);
            if (ret == 0) ms41908_vd_signal(dev);
            break;
        }

        case MS41908_IOCTL_FOCUS_STOP:
        {
            ret = ms41908_write_reg(dev, MS41908_REG_MOTOR_A_CTRL, MS41908_MOTOR_FOCUS_STOP);
            if (ret == 0) ms41908_vd_signal(dev);
            break;
        }

        default:
            dev_warn(&dev->spi->dev, "Unsupported IOCTL cmd: 0x%08X\n", cmd);
            ret = -ENOTTY;
            break;
    }
    mutex_unlock(&dev->lock);

    return ret;
}

/* 文件操作集合 */
static const struct file_operations ms41908_fops = {
    .owner = THIS_MODULE,
    .open = ms41908_open,
    .write = ms41908_write,
    .read = ms41908_read,
    .unlocked_ioctl = ms41908_ioctl,  
};

/**
 * @brief SPI设备探测函数
 */
static int ms41908_probe(struct spi_device *spi)
{
	int ret;
	struct MS41908_DEV *dev = &my_ms41908_dev;

	// 1. 注册字符设备号
    //字符设备编号（主设备号 + 次设备号，唯一标识一个字符设备）
    if (dev->major == 0) {
        ret = alloc_chrdev_region(&dev->devid, 0, MS41908_CNT, MS41908_NAME);     //动态分配设备号
    } else {
        dev->devid = MKDEV(dev->major, 0);
        ret = register_chrdev_region(dev->devid, MS41908_CNT, MS41908_NAME);      //手动分配设备号
    }
    if (ret < 0) {
        dev_err(&spi->dev, "Register chrdev failed! ret=%d\n", ret);
        goto err_chrdev;
    }
    dev->major = MAJOR(dev->devid);

	// 2. 初始化并添加cdev
    cdev_init(&dev->cdev, &ms41908_fops);
    ret = cdev_add(&dev->cdev, dev->devid, MS41908_CNT);
    if (ret < 0) {
        dev_err(&spi->dev, "Add cdev failed! ret=%d\n", ret);
        goto err_cdev_add;
    }
    
    // 3. 创建设备类
    //用于在/sys/class下创建设备类，自动生成设备节点
    dev->class = class_create(THIS_MODULE, MS41908_NAME);
    if (IS_ERR(dev->class)) {
        ret = PTR_ERR(dev->class);
        dev_err(&spi->dev, "Create class failed! ret=%d\n", ret);
        goto err_class;
    }
    
    // 4. 创建设备实例
    //用于在/dev下创建设备文件，用户态可通过该文件操作设备
    dev->device = device_create(dev->class, NULL, dev->devid, NULL, MS41908_NAME);
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
    spi->max_speed_hz = MS41908_SPI_SPEED;
    spi_setup(spi);
    dev->private_data = spi;
    dev->spi = spi;

    //8.复位芯片
    ms41908_reset(dev);

    ret = ms41908_init_reg(dev);
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
    unregister_chrdev_region(dev->devid, MS41908_CNT);
err_chrdev:
    return ret;	
}

static void  ms41908_remove(struct spi_device *spi)
{
	struct MS41908_DEV *dev = &my_ms41908_dev;

	// 删除字符设备
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devid, MS41908_CNT);
    device_destroy(dev->class, dev->devid);
    class_destroy(dev->class);
    dev_info(&spi->dev, "AN41908 remove success\n");
}

/* 设备树匹配列表 */
static const struct of_device_id ms41908_of_match[] = {
	{ .compatible = "panasonic,ms41908" },
	{ /* Sentinel */ }
};

/* SPI设备ID表（匹配传统非设备树场景） */
static const struct spi_device_id ms41908_spi_id[] = {
    { "panasonic,ms41908", 0 },  // 名字和设备树compatible一致
    { /* Sentinel */ }
};
MODULE_DEVICE_TABLE(spi, ms41908_spi_id);  // 内核导出ID表

/*ms41908驱动结构体*/
static struct spi_driver ms41908_driver = {
	.probe = ms41908_probe,
	.remove = ms41908_remove,
    .id_table = ms41908_spi_id,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "ms41908_spi",
		   	.of_match_table = ms41908_of_match, 
		   },
};

static int __init ms41908_init(void)
{
	return spi_register_driver(&ms41908_driver);
}

static void __exit ms41908_exit(void)
{
	spi_unregister_driver(&ms41908_driver);
}

module_init(ms41908_init);
module_exit(ms41908_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("PANGWEI");