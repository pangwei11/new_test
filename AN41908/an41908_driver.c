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

//修改测试

//测试

// 寄存器读写控制位
#define AN41908_WRITE_CMD 0x00  // C0=0（写模式）
#define AN41908_READ_CMD  0x40  // C0=1（读模式，bit6置1）
#define AN41908_C1_UNUSED 0x00  // C1=0（未使用）

#define AN41908_CNT 1              // 设备数量
#define AN41908_NAME "AN41908"        // 设备名称

// ========== 新增：ioctl 相关定义（仅对焦） ==========
// 1. 是这个驱动的 “魔法数”（一般是一个自定义字符，比如 'F'），用于区分不同驱动的 IOCTL 命令，防止指令串扰
#define AN41908_IOCTL_MAGIC 'F'

// 2. 对焦参数结构体（传递方向和步长）
struct an41908_focus_param {
    unsigned int motor_id;  // 新增：0=电机A, 1=电机B
    bool forward;  // 方向：false=对焦近，true=对焦远
    int step;      // 步长（应用层传0xff）
};

// 3. 定义ioctl命令（对焦精准控制）
#define AN41908_IOCTL_SET_FOCUS_PRECISE _IOW(AN41908_IOCTL_MAGIC, 0, struct an41908_focus_param)
#define AN41908_IOCTL_MOTOR_OFF _IOW(AN41908_IOCTL_MAGIC, 1, unsigned int)
#define AN41908_IOCTL_SET_IRIS     _IOW(AN41908_IOCTL_MAGIC, 2, unsigned int)
#define AN41908_IOCTL_VD_PULSE           _IO(AN41908_IOCTL_MAGIC, 3)
#define AN41908_IOCTL_IRIS_DIR           _IOW(AN41908_IOCTL_MAGIC, 4, unsigned int)

// 4. 最大命令编号
#define AN41908_IOCTL_MAX_NR 4

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

// /**
//  * @brief 从AN41908读取寄存器（LSB先行、SPI模式0）
//  * @param dev: 设备结构体指针
//  * @param addr: 寄存器地址（A0-A5，6位）
//  * @param data: 读取的16位数据
//  * @return 0成功，负数失败
//  */
// static int an41908_read_reg(struct AN41908_DEV *dev, uint8_t addr, uint16_t *data)
// {
//     int ret;
//     uint8_t tx_buf[3] = {0};  // 发送缓冲区：命令字节 + 2字节填充
//     uint8_t rx_buf[3] = {0};  // 接收缓冲区：2字节填充的回显 + 2字节数据

//     /* 1. 构造读命令（地址 + 读控制位） */
//     // 地址A0-A5占bit0-bit5，C0=1（读）、C1=0（保留位，可根据手册调整）
//     tx_buf[0] = (addr & 0x3F) | (1 << 6);  // bit6=1（C0=1，读操作）

//     /* 2. SPI传输：发送命令+填充，同时接收数据 */
//     struct spi_transfer t = {
//         .tx_buf = tx_buf,
//         .rx_buf = rx_buf,
//         .len = 3,              // 发送3字节（命令+2填充），接收3字节
//         .speed_hz = dev->spi->max_speed_hz,
//         .bits_per_word = 8,
//     };

//     ret = spi_sync_transfer(dev->spi, &t, 1);
//     if (ret < 0) {
//         dev_err(&dev->spi->dev, "读寄存器失败（addr=0x%02X）: %d\n", addr, ret);
//         return ret;
//     }

//     /* 3. 解析接收到的数据（低8位在前，高8位在后） */
//     // rx_buf[1]：从机返回的低8位（D0-D7）
//     // rx_buf[2]：从机返回的高8位（D8-D15）
//     *data = (rx_buf[2] << 8) | rx_buf[1];

//     dev_dbg(&dev->spi->dev, "读寄存器成功: addr=0x%02X, data=0x%04X\n", addr, *data);
//     return 0;
// }

/**
 * @brief  驱动层实现GPIO_VD_FZ的电平翻转（高10ms→低）
 * @param  dev: 设备实例指针
 */
static void an41908_signal_gpio(struct AN41908_DEV *dev)
{
    mutex_lock(&dev->lock);  // 加锁保护
    gpiod_set_value(dev->vd_fz_gpio, 1);  // 高电平
    ndelay(500);        //500ns
    gpiod_set_value(dev->vd_fz_gpio, 0);  // 低电平
    mutex_unlock(&dev->lock);
    dev_info(&dev->spi->dev, "VD_FZ上升沿已触发,芯片开始执行对焦");
}

/**
 * @brief  驱动层实现turn_on（发送开启电机的SPI指令）
 * @param  dev: 设备实例指针
 * @return 0=成功，负数=失败
 */
static int an41908_turn_on(struct AN41908_DEV *dev,int motor_id)
{
    int ret;
    uint16_t ctrl_reg, step_reg;
    uint16_t ctrl_val = 0x040f;   // 基础控制值（与原来一致）
    uint16_t step_val = 0x015F;   // 步周期值（可根据需要调整）

    if (motor_id == 0) {
        ctrl_reg = 0x24;
        step_reg = 0x25;
    } else if (motor_id == 1) {
        ctrl_reg = 0x29;
        step_reg = 0x2A;           // 电机B的步周期寄存器地址
    } else {
        return -EINVAL;
    }

    dev_info(&dev->spi->dev, "write ctrl reg 0x%02X for motor %d ! ret=%d\n", ctrl_reg, motor_id, ret);
    mutex_lock(&dev->lock);
    ret = an41908_write_reg(dev, ctrl_reg, ctrl_val);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "write ctrl reg 0x%02X for motor %d failed! ret=%d\n", ctrl_reg, motor_id, ret);
        goto out;
    }
    ret = an41908_write_reg(dev, step_reg, step_val);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "write step reg 0x%02X for motor %d failed! ret=%d\n", step_reg, motor_id, ret);
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
static int an41908_turn_off(struct AN41908_DEV *dev,int motor_id)
{
    int ret;
    uint16_t ctrl_reg;
    uint16_t reg_stop = 0x0400;   // 停止值：PSUMAB=0

    if (motor_id == 0) {
        ctrl_reg = 0x24;
    } else if (motor_id == 1) {
        ctrl_reg = 0x29;
    } else {
        return -EINVAL;
    }

    mutex_lock(&dev->lock);
    ret = an41908_write_reg(dev, ctrl_reg, reg_stop);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "关闭电机%d写入控制寄存器0x%02X失败! ret=%d\n", motor_id, ctrl_reg, ret);
        goto out;
    }
    dev_info(&dev->spi->dev, "电机%d已安全关闭(PSUMAB=0)", motor_id);
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
static int an41908_send_zoom_cmd(struct AN41908_DEV *dev, int motor_id, bool forward, int step)
{
    int ret; 
    uint16_t ctrl_reg;
    uint16_t ctrl_base = 0x040f;   // 基础值（与turn_on一致）
    uint16_t dir_val = forward ? 1 : 0;
    uint16_t ctrl_final;

    if (step < 1 || step > 0xFF) {
        dev_err(&dev->spi->dev, "step参数非法! step=%d, 必须1~255\n", step);
        return -EINVAL;
    }

    if (motor_id == 0) {
        ctrl_reg = 0x24;
    } else if (motor_id == 1) {
        ctrl_reg = 0x29;
    } else {
        return -EINVAL;
    }

    mutex_lock(&dev->lock);
    ctrl_final = ctrl_base;           // 先拷贝基础值
    ctrl_final &= ~0x01FF;            // 清零D0~D8
    ctrl_final |= (step & 0x00FF);    // 写入新步长→D0~D7位（PSUMAB[7:0]）
    ctrl_final |= (dir_val << 8);     // 写入新方向→D8位

    ret = an41908_write_reg(dev, ctrl_reg, ctrl_final);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "动态写入控制寄存器0x%02X失败! ret=%d\n", ctrl_reg, ret);
        goto out;
    }

    dev_info(&dev->spi->dev, "对焦参数已配置：motor=%d,方向=%s,步长=%d,控制值=0x%04x",
             motor_id, forward ? "远" : "近", step, ctrl_final);

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
static int an41908_set_zoom_precise(struct AN41908_DEV *dev, int motor_id, bool forward, int step)
{
    int ret;

    an41908_write_reg(dev, 0x0B, 0x0080);//测试

    // 1. 开启电机
    ret = an41908_turn_on(dev,motor_id);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "turn_on motor %d failed! ret=%d\n", motor_id, ret);
        return ret;
    }

    // 2. 发送对焦指令
    ret = an41908_send_zoom_cmd(dev, motor_id, forward, step);
    if (ret < 0) {
        dev_err(&dev->spi->dev, "send_focus_cmd motor %d failed! ret=%d\n", motor_id, ret);
        an41908_turn_off(dev, motor_id);  // 失败时关闭该电机
        return ret;
    }

    // 3. 触发GPIO_VD_FZ
    an41908_signal_gpio(dev);
      
    // //4. 根据需要决定是否立即关闭电机（当前注释掉，保持电机使能）
    // ret = an41908_turn_off(dev, motor_id);
    // if (ret < 0) {
    //     dev_err(&dev->spi->dev, "turn_off motor %d failed! ret=%d\n", motor_id, ret);
    //     return ret;
    // }

    dev_info(&dev->spi->dev, "motor %d focus precise success! forward=%d, step=%d\n", motor_id, forward, step);
    return 0;
}

// static int iris_cycle_thread(void *data)
// {
//     struct AN41908_DEV *dev = data;
//     int value = 0;          // 当前光圈值（十进制）
//     int step = 100;         // 步长
//     int direction = 1;      // 方向：1增加，-1减少
//     char *dir_str;

//     while (!kthread_should_stop() && !dev->thread_stop) {
//         dir_str = (direction == 1) ? "增大" : "减小";
//         // 写入寄存器0x00（IRS_TGT），只取低10位
//         uint16_t reg_val = value & 0x3FF;
//         mutex_lock(&dev->lock);
//         an41908_write_reg(dev, 0x00, reg_val);
//         mutex_unlock(&dev->lock);

//         // 打印当前方向和寄存器值
//         dev_info(&dev->spi->dev, "光圈循环: %s, 写入值=0x%03X (十进制%d)\n", dir_str, reg_val, value);

//         // 触发VD脉冲（使新目标生效）
//         an41908_signal_gpio(dev);

//         // 延时5ms
//         msleep(20);

//         // 更新value
//         value += direction * step;
//         if (value >= 1000) {
//             value = 1000;
//             direction = -1;   // 改为减少
//         } else if (value <= 0) {
//             value = 0;
//             direction = 1;    // 改为增加
//         }
//     }
//     return 0;
// }

// static int test_iris(struct AN41908_DEV *dev)
// {
//     // 确保线程未运行
//     if (dev->thread)
//         return 0;

//     dev->thread_stop = false;
//     dev->thread = kthread_run(iris_cycle_thread, dev, "iris_cycle");
//     if (IS_ERR(dev->thread)) {
//         int ret = PTR_ERR(dev->thread);
//         dev_err(&dev->spi->dev, "Failed to create iris cycle thread: %d\n", ret);
//         dev->thread = NULL;
//         return ret;
//     }

//     dev_info(&dev->spi->dev, "Iris cycle thread started\n");
//     return 0;
// }

static int AN41908_A_FOCUS_motor_init(struct AN41908_DEV *dev)
{
    int ret;
    ret = an41908_write_reg(dev, 0x20, 0x1E03);     //设置PWM与DT1,PWM频率为112.5KHz,DT1时间约为3x303.4us~=0.9ms
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x21, 0x0087);     //设置PLS1 PLS2输出信号类型

    ret = an41908_write_reg(dev, 0x22, 0x0002);     //设置电机激励等待时间2x303us=0.6ms,驱动器A与驱动器B的相位差为默认的90°
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x23, 0xd8d8);     //设置电机pwm占空比为90%,写0x7878的时候为50%，占空比越大驱动能力越强
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x24, 0x040f);     //设置电机核心运行总控，0f除以8得到一个VD_FZ周期走两个步距角，正转，关闭刹车状态，1开启关闭电机，256细分模式
    //ret = an41908_write_reg(dev, 0x24, 0x0400);//先关闭
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x27, 0x0002);     //设置电机激励等待时间2x303us=0.6ms,驱动器A与驱动器B的相位差为默认的90°
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x28, 0xd8d8);     //设置电机pwm占空比为90%,写0x7878的时候为50%，占空比越大驱动能力越强
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x29, 0x040f);     //设置电机核心运行总控，0f除以8得到一个VD_FZ周期走两个步距角，正转，关闭刹车状态，1开启关闭电机，256细分模式
    //ret = an41908_write_reg(dev, 0x29, 0x0400);//先关闭
    if (ret < 0) return ret;

    /*******************************************光圈部分********************************************************************* */

    ret = an41908_write_reg(dev, 0x00, 0x03E0);     //这个是设置IRS_TGT[9:0],先写0000代表光圈全关闭
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x01, 0x7c8a);     //这个低位先不用管都使用为0默认值，高七位是DGAIN[6:0]（PID 控制器增益），也就是PID中的P比例
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x02, 0x66f0);     //设置PID零点 = 35Hz，极点 = 950Hz，PID_ZERO[3:0]其实就是PID中的I积分，PID_POLE[3:0]就是PID中的D微分
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x03, 0x0e10);     //设置PID的PWM频率 = 31.25KHz
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x04, 0x7070);     //设设置霍尔的偏置电流与偏置电压
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x05, 0x0F04);     //0024PID极性取反，0004则不取反,F设置霍尔传感器信号增益最大
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x0A, 0x075F);     // 驱动信号占空比，设置这个会手动控制光圈，bit9控制方向
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x0B, 0x0480);     //00480使能光圈，00080不使能光圈 
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x0E, 0x0300);     // 
    if (ret < 0) return ret;

    dev_info(&dev->spi->dev, "AN41908 focus and zoom motor init success!\n");

    return 0;
}

#if 0
static int AN41908_iris_init(struct AN41908_DEV *dev)
{
    int ret;
    ret = an41908_write_reg(dev, 0x00, 0x0000);     //这个是设置IRS_TGT[9:0],先写0000代表光圈全关闭
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x01, 0x7c8a);     //这个低位先不用管都使用为0默认值，高七位是DGAIN[6:0]（PID 控制器增益），也就是PID中的P比例
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x02, 0x66f0);     //设置PID零点 = 35Hz，极点 = 950Hz，PID_ZERO[3:0]其实就是PID中的I积分，PID_POLE[3:0]就是PID中的D微分
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x03, 0x0e10);     //设置PID的PWM频率 = 31.25KHz
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x04, 0x80ff);     //设设置霍尔的偏置电流与偏置电压
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x05, 0x0024);     //设设置霍尔的偏置电流与偏置电压
    if (ret < 0) return ret;

    // ret = an41908_write_reg(dev, 0x0A, 0x075F);     // 驱动信号占空比与 PWM_IRIS[2:0] 的设置值有关
    // if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x0B, 0x0480);     // ??????
    if (ret < 0) return ret;

    ret = an41908_write_reg(dev, 0x0E, 0x0300);     // 
    if (ret < 0) return ret;

    dev_info(&dev->spi->dev, "AN41908 iris motor init success!\n");
    return 0;
}
#endif

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

static ssize_t spi_oled_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *off) 
{
    
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
            // 校验电机ID
            if (param.motor_id != 0 && param.motor_id != 1) {
                dev_err(&dev->spi->dev, "Invalid motor_id %u\n", param.motor_id);
                ret = -EINVAL;
                break;
            }
            dev_info(&dev->spi->dev, "motor_id %u\n", param.motor_id);
            // 调用驱动层对焦函数
            ret = an41908_set_zoom_precise(dev, param.motor_id, param.forward, param.step);
            break;

        case AN41908_IOCTL_MOTOR_OFF:
        {
            unsigned int motor_id;
            // 从用户层拷贝电机ID（参数为 unsigned int）
            if (copy_from_user(&motor_id, (void __user *)arg, sizeof(motor_id))) {
                ret = -EFAULT;
                break;
            }
            if (motor_id != 0 && motor_id != 1) {
                dev_err(&dev->spi->dev, "Invalid motor_id %u for turn_off\n", motor_id);
                ret = -EINVAL;
                break;
            }
            ret = an41908_turn_off(dev, motor_id);
            break;
        }

        case AN41908_IOCTL_SET_IRIS:
        {
            unsigned int target;
            if (copy_from_user(&target, (void __user *)arg, sizeof(target))) {
                ret = -EFAULT;
                break;
            }
            if (target > 1023) {
                dev_err(&dev->spi->dev, "Invalid iris target %u (max 1023)\n", target);
                ret = -EINVAL;
                break;
            }

            //先关闭两个电机，避免触发时干扰
            // an41908_turn_off(dev, 0);
            // an41908_turn_off(dev, 1);

            ret = an41908_write_reg(dev, 0x0B, 0x0480);//测试

            an41908_signal_gpio(dev);

            an41908_write_reg(dev, 0x00, target & 0x3FF);
            // 触发 VD_IS/VD_FZ 脉冲，使新目标生效
            an41908_signal_gpio(dev);
            ndelay(500);
            an41908_signal_gpio(dev);
            ndelay(500);
            an41908_signal_gpio(dev);

            dev_info(&dev->spi->dev, "iris set to %u\n", target);
            break;
        }

        case AN41908_IOCTL_VD_PULSE:
        {
            an41908_signal_gpio(dev);
            break;
        }

        case AN41908_IOCTL_IRIS_DIR:
        {
            unsigned int dir;
            u16 reg_val=0x075F;

                // 从用户空间拷贝方向参数
            if (copy_from_user(&dir, (void __user *)arg, sizeof(dir))) {
                ret = -EFAULT;
                break;
            }
                // 校验方向值（必须为0或1）
            if (dir != 0 && dir != 1) {
                dev_err(&dev->spi->dev, "Invalid iris direction %u (must be 0 or 1)\n", dir);
                ret = -EINVAL;
                break;
            }
                // 2. 修改bit9
            if (dir)
                reg_val |= (1 << 9);   // 置1
            else
                reg_val &= ~(1 << 9);  // 清0
                // 3. 写回寄存器
            ret = an41908_write_reg(dev, 0x0A, reg_val);
            if (ret == 0) {
                dev_info(&dev->spi->dev, "Iris direction set to %u (reg 0x0A=0x%04x)\n", dir, reg_val);
            } else {
                dev_err(&dev->spi->dev, "Failed to write reg 0x0A, err=%d\n", ret);
            }

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

    // test_iris(dev);

    // ret = AN41908_iris_init(dev);
    // if (ret != 0) {
    //     goto err_motor_init;
    // }

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