#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <termios.h>
  
// 复制驱动层的ioctl定义（保持一致）
//这个驱动的 “魔法数”，用于区分不同驱动的 IOCTL 命令，防止指令串扰
#define MS41908_IOCTL_MAGIC 'F'

// 3. 定义ioctl命令
#define MS41908_IOCTL_SET_IRIS     _IOW(MS41908_IOCTL_MAGIC, 0, unsigned int)   //该命令是用于设置光圈目标值，范围是0x00~0x3FF,由于存在霍尔偏置，0x00~0x3FF是理想的量程，实际上靠近0x00和0x3ff部分的值是无效的
#define MS41908_IOCTL_VD_PULSE           _IO(MS41908_IOCTL_MAGIC, 1)            //该命令是给一个驱动信号，变焦、对焦、设置光圈大小、改变寄存器值都依赖这个信号，如果想让变焦对焦连续变动，可以连续给这个信号，但是这个信号的周期不能小于60Hz，也就是16.66ms，否则会丢步。合适的周期可以让电机连续转动没有卡顿感，周期太长则会有明显的卡顿感。
#define MS41908_IOCTL_ZOOM_FORWARD           _IO(MS41908_IOCTL_MAGIC, 2)        //该命令是设置电机变焦的正向运动
#define MS41908_IOCTL_ZOOM_REVERSE           _IO(MS41908_IOCTL_MAGIC, 3)        //该命令是设置电机变焦的反向运动
#define MS41908_IOCTL_ZOOM_STOP           _IO(MS41908_IOCTL_MAGIC, 4)           //该命令是设置电机变焦停止运动
#define MS41908_IOCTL_FOCUS_FORWARD           _IO(MS41908_IOCTL_MAGIC, 5)       //该命令是设置电机对焦的正向运动
#define MS41908_IOCTL_FOCUS_REVERSE           _IO(MS41908_IOCTL_MAGIC, 6)       //该命令是设置电机对焦的反向运动
#define MS41908_IOCTL_FOCUS_STOP           _IO(MS41908_IOCTL_MAGIC, 7)          //该命令是设置电机对焦停止
//注意：例如MS41908_IOCTL_ZOOM_FORWARD让镜头变焦后，如果没有给MS41908_IOCTL_ZOOM_STOP命令，就运行MS41908_IOCTL_FOCUS_FORWARD对焦命令，会导致镜头也会变焦一次，因为对焦和变焦都会以VD为驱动信号

// 光圈调节步进
#define IRIS_STEP 0x10
// 光圈值范围
#define IRIS_MIN 0x000
#define IRIS_MAX 0x3FF
#define IRIS_LOW_THRESHOLD 0x0010  // 减小逻辑的阈值（≤此值则置0）
#define IRIS_HIGH_THRESHOLD 0x03F0 // 增大逻辑的阈值（≥此值则置0x3FF）

// 和驱动层一致的结构体定义
struct reg_data {
    uint8_t addr;   //寄存器地址
    uint16_t data;  //数据
};

// 寄存器写入函数
static int write_an41908_reg(int fd, uint8_t reg_addr, uint16_t reg_data) {
    struct reg_data reg = {
        .addr = reg_addr,
        .data = reg_data
    };

    ssize_t ret = write(fd, &reg, sizeof(struct reg_data));
    if (ret != sizeof(struct reg_data)) {
        perror("write reg failed");
        return -1;
    }
    printf("Write reg 0x%02X: 0x%04X success\n", reg_addr, reg_data);
    return 0;
}

// 寄存器读取函数
static int read_an41908_reg(int fd, uint8_t reg_addr, uint16_t *reg_data) {
    struct reg_data reg = {
        .addr = reg_addr,
        .data = 0
    };

    ssize_t ret = read(fd, &reg, sizeof(struct reg_data));
    if (ret != sizeof(struct reg_data)) {
        perror("read reg failed");
        return -1;
    }
    *reg_data = reg.data;
    return 0;
}

// 配置终端为无缓冲模式
static void set_terminal_mode(void) {
    struct termios ctrl;
    tcgetattr(STDIN_FILENO, &ctrl);
    ctrl.c_lflag &= ~ICANON;  // 关闭行缓冲
    ctrl.c_lflag &= ~ECHO;    // 关闭回显
    tcsetattr(STDIN_FILENO, TCSANOW, &ctrl);
}

int main(void)
{
    int ret =0;
    struct reg_data reg_0x00 = {0};     //定义光圈目标值寄存器
    uint16_t current_iris_val = 0;      // 缓存0x00寄存器当前值

    // 0x00寄存器最大值限制
    #define REG00_MAX 0x3FF

    int fd = open("/dev/MS41908", O_RDWR);
    if (fd < 0) {
        perror("open /dev/MS41908 failed");
        return -1;
    }

    // 配置终端，支持实时按键监听
    set_terminal_mode();

    printf("Press 'Ctrl+C' to exit\n\n");;

    while (1) {
        char ch;
        // 读取1个按键（无缓冲模式）
        int ret = read(STDIN_FILENO, &ch, 1);
        if (ret <= 0) continue;

        switch (ch) {
            //光圈全开
            case 'w':  
            {
                unsigned int iris_full_open = 0x0000; // 光圈全开值
                // 直接调用IOCTL命令设置光圈
                int ret = ioctl(fd, MS41908_IOCTL_SET_IRIS, &iris_full_open);
                if (ret < 0) {
                    perror("ioctl set iris full open failed");
                } else {
                    printf("光圈已全开\n");
                }
                break;
            }
            //光圈全闭
            case 'q':  
            {
                unsigned int iris_full_close = 0x03FF; // 全闭
                // 直接调用IOCTL命令设置光圈
                int ret = ioctl(fd, MS41908_IOCTL_SET_IRIS, &iris_full_close);
                if (ret < 0) {
                    perror("ioctl set iris full close failed");
                } else {
                    printf("光圈已全闭\n");
                }
                break;
            }

            case 'v':  // 产生20个VD脉冲，频率是60HZ
            {
                //可以通过控制给多少个VD脉冲，从而控制对焦和变焦的大小
                for (int i = 0; i < 20; i++) {
                    ioctl(fd, MS41908_IOCTL_VD_PULSE, 0);  // 触发单次脉冲
                    usleep(16666);                         // 等待约16.6ms
                }
                break;
            }

            case 'z':  //对焦反向
            {
                //此时可以不给VD信号，因为在驱动里面，MS41908_IOCTL_FOCUS_REVERSE已经给过一次一次VD信号，如果想让他连续动，可以按多次Z键，或者按下V键连续给20个脉冲，他都会连续动20次，所以想控制他动多少可以通过控制给多少个VD信号即可，下面的同理
                ioctl(fd,MS41908_IOCTL_FOCUS_REVERSE,0);
                
                break;
            }

            case 'x':   //对焦正向
            {
                ioctl(fd,MS41908_IOCTL_FOCUS_FORWARD,0);
                
                break;
            }

            case '+':  //变焦正向
            {
                ioctl(fd,MS41908_IOCTL_ZOOM_FORWARD,0);
                
                break;
            }

            case '-':  //变焦反向
            {
                ioctl(fd,MS41908_IOCTL_ZOOM_REVERSE,0);
                
                break;
            }

            case 't':  //对焦关闭
            {
                ioctl(fd,MS41908_IOCTL_FOCUS_STOP,0);
                
                break;
            }

            case 'y':  //变焦关闭
            {
                ioctl(fd,MS41908_IOCTL_ZOOM_STOP,0);
                
                break;
            }

            case 'r':   //读取0x00寄存器的值，也就是设置光圈大小的值
            {
                // 1. 配置要读取的寄存器地址（0x00）
                reg_0x00.addr = 0x00;
                reg_0x00.data = 0;  

                // 2. 调用驱动read接口读取寄存器值
                ssize_t read_ret = read(fd, &reg_0x00, sizeof(struct reg_data));
                if (read_ret != sizeof(struct reg_data)) {
                    perror("read reg 0x00 failed");
                    break;
                }

                // 3. 打印读取结果
                printf("=== Read reg 0x00 success ===\n");
                printf("  Reg addr: 0x%02X\n", reg_0x00.addr);
                printf("  Reg value: 0x%04X (decimal: %d)\n", reg_0x00.data, reg_0x00.data);
                // 额外提示：0x00寄存器是光圈目标值，范围0~0x3FF
                if (reg_0x00.data > REG00_MAX) {
                    printf("  Warning: Reg 0x00 value exceeds max (0x3FF)\n");
                }
                break;
            }

            // ========== l键 减小光圈 ==========
            case 'l':
            {
                // 1. 读取0x00寄存器当前值
                ret = read_an41908_reg(fd, 0x00, &current_iris_val);
                if (ret < 0) {
                    printf("Failed to read reg 0x00 before decrease\n");
                    break;
                }
                printf("Current reg 0x00 value: 0x%04X\n", current_iris_val);

                // 2. 逻辑：≤0x0010则置0，否则减0x10
                if (current_iris_val <= IRIS_LOW_THRESHOLD) {
                    current_iris_val = IRIS_MIN;
                    printf("Reg 0x00 ≤ 0x%04X, set to min (0x%04X)\n", IRIS_LOW_THRESHOLD, IRIS_MIN);
                } else {
                    current_iris_val -= IRIS_STEP;
                    printf("Reg 0x00 decreased by 0x%04X → 0x%04X\n", IRIS_STEP, current_iris_val);
                }

                // 3. 写入新值到0x00寄存器
                ret = write_an41908_reg(fd, 0x00, current_iris_val);
                if (ret < 0) {
                    break;
                }

                // 4. 触发脉冲生效
                ioctl(fd, MS41908_IOCTL_VD_PULSE, 0);
                break;
            }

            // ========== k键 增大光圈 ==========
            case 'k':
            {
                // 1. 读取0x00寄存器当前值
                ret = read_an41908_reg(fd, 0x00, &current_iris_val);
                if (ret < 0) {
                    printf("Failed to read reg 0x00 before increase\n");
                    break;
                }
                printf("Current reg 0x00 value: 0x%04X\n", current_iris_val);

                // 2. 逻辑：≥0x03F0则置0x3FF，否则加0x10
                if (current_iris_val >= IRIS_HIGH_THRESHOLD) {
                    current_iris_val = IRIS_MAX;
                    printf("Reg 0x00 ≥ 0x%04X, set to max (0x%04X)\n", IRIS_HIGH_THRESHOLD, IRIS_MAX);
                } else {
                    current_iris_val += IRIS_STEP;
                    printf("Reg 0x00 increased by 0x%04X → 0x%04X\n", IRIS_STEP, current_iris_val);
                }

                // 3. 写入新值到0x00寄存器
                ret = write_an41908_reg(fd, 0x00, current_iris_val);
                if (ret < 0) {
                    break;
                }

                // 4. 触发脉冲生效
                ioctl(fd, MS41908_IOCTL_VD_PULSE, 0);
                break;
            }

            case 3:  // Ctrl+C退出
                printf("\nExit...\n");
                close(fd);
                return 0;

            default:
                printf("Unknown key: %c\n", ch);
                break;
        }
    }

    close(fd);
    return 0;
}