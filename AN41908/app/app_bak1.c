#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <termios.h>
  

// 复制驱动层的ioctl定义（保持一致）
#define AN41908_IOCTL_MAGIC 'F'

#define AN41908_IOCTL_SET_FOCUS_PRECISE _IOW(AN41908_IOCTL_MAGIC, 0, struct an41908_focus_param)
#define AN41908_IOCTL_MOTOR_OFF _IOW(AN41908_IOCTL_MAGIC, 1, unsigned int)
#define AN41908_IOCTL_SET_IRIS     _IOW(AN41908_IOCTL_MAGIC, 2, unsigned int)
#define AN41908_IOCTL_VD_PULSE           _IO(AN41908_IOCTL_MAGIC, 3)
#define AN41908_IOCTL_IRIS_DIR _IOW(AN41908_IOCTL_MAGIC, 4, unsigned int)

// 光圈目标值（初始值和驱动初始化的0x200一致）
static unsigned int g_iris_target = 0x200;
// 光圈调节步进（和原始代码一致，每次±0x10）
#define IRIS_STEP 0x10
// 光圈值范围
#define IRIS_MIN 0x000
#define IRIS_MAX 0x3FF
#define IRIS_LOW_THRESHOLD 0x0010  // 减小逻辑的阈值（≤此值则置0）
#define IRIS_HIGH_THRESHOLD 0x03F0 // 增大逻辑的阈值（≥此值则置0x3FF）

// 和驱动层一致的结构体定义
struct reg_data {
    uint8_t addr;
    uint16_t data;
};

// 通用寄存器写入函数（封装重复逻辑，便于复用）
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

// 配置终端为无缓冲模式（按按键立即响应，无需回车）
static void set_terminal_mode(void) {
    struct termios ctrl;
    tcgetattr(STDIN_FILENO, &ctrl);
    ctrl.c_lflag &= ~ICANON;  // 关闭行缓冲
    ctrl.c_lflag &= ~ECHO;    // 关闭回显
    tcsetattr(STDIN_FILENO, TCSANOW, &ctrl);
}

// 设置光圈目标值
static int set_iris_target(int fd, unsigned int target) {
    int ret = ioctl(fd, AN41908_IOCTL_SET_IRIS, &target);
    if (ret < 0) {
        perror("ioctl AN41908_IOCTL_SET_IRIS failed");
        return -1;
    }
    printf("Set IRIS target: 0x%03X (current: 0x%03X)\n", target, g_iris_target);
    g_iris_target = target;
    return 0;
}

int main(void)
{
    int ret =0;
    struct reg_data reg = {0};
    uint16_t current_iris_val = 0; // 缓存0x00寄存器当前值

    // 跟踪0x00寄存器的值，初始为0x00
    static unsigned int reg00_value = 0x00;
    // 0x00寄存器最大值限制
    #define REG00_MAX 0x3FF
    // 0x00寄存器每次增加的步长
    #define REG00_STEP 0x10

    int fd = open("/dev/AN41908", O_RDWR);
    if (fd < 0) {
        perror("open /dev/AN41908 failed");
        return -1;
    }

    // 配置终端，支持实时按键监听
    set_terminal_mode();

    printf("Press 'Ctrl+C' to exit\n\n");;

    printf("\n=== AN41908寄存器初始化完成 ===\n\n");

    while (1) {
        char ch;
        // 读取1个按键（无缓冲模式）
        int ret = read(STDIN_FILENO, &ch, 1);
        if (ret <= 0) continue;

        switch (ch) {
            case 'w':  // 减小光圈（目标值-0x10）
            {
                reg.addr= 0x00;
                reg.data=0x0000;
                    // 3. 写入寄存器
                if (write(fd, &reg, sizeof(struct reg_data)) != sizeof(struct reg_data)) {
                    perror("write reg failed");
                    close(fd);
                    return -1;
                }

                ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);  // 触发单次脉冲
                break;
            }

            case 'q':  // 增大光圈（目标值+0x10）
            {
                reg.addr= 0x00;
                reg.data=0x03f0;
                    // 3. 写入寄存器
                if (write(fd, &reg, sizeof(struct reg_data)) != sizeof(struct reg_data)) {
                    perror("write reg failed");
                    close(fd);
                    return -1;
                }

                ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);  // 触发单次脉冲
                break;
            }

            case 'e':  // 减小光圈（目标值-0x10）
            {
                // 1. 计算新值（先加步长）
                reg00_value += REG00_STEP;
                // 2. 限制最大值不超过0x3FF
                if (reg00_value > REG00_MAX) {
                    reg00_value = REG00_MAX;
                    printf("Reg 0x00 has reached max value: 0x%03X\n", REG00_MAX);
                }
                // 3. 配置0x00寄存器参数
                reg.addr = 0x00;
                reg.data = reg00_value;
                // 4. 写入寄存器
                if (write(fd, &reg, sizeof(struct reg_data)) != sizeof(struct reg_data)) {
                    perror("write reg 0x00 failed");
                    close(fd);
                    return -1;
                }
                printf("Reg 0x00 updated to: 0x%03X (step: 0x10, max: 0x3FF)\n", reg00_value);
                // 5. 触发单次脉冲（和其他case保持一致）
                ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);
                break;
            }

            case 'v':  // 产生60个脉冲（约1秒）
            {
                for (int i = 0; i < 20; i++) {
                    ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);  // 触发单次脉冲
                    usleep(16666);                         // 等待约16.6ms
                }
                break;
            }

            case 'm':  
            {
                reg.addr= 0x0A;
                reg.data=0x075F;
                    // 3. 写入寄存器
                if (write(fd, &reg, sizeof(struct reg_data)) != sizeof(struct reg_data)) {
                    perror("write reg failed");
                    close(fd);
                    return -1;
                }
                ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);  // 触发单次脉冲
                break;
            }

            case 'g':  
            {
                reg.addr= 0x0A;
                reg.data=0x055F;
                    // 3. 写入寄存器
                if (write(fd, &reg, sizeof(struct reg_data)) != sizeof(struct reg_data)) {
                    perror("write reg failed");
                    close(fd);
                    return -1;
                }
                ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);  // 触发单次脉冲
                break;
            }

            case 'z':  
            {
                reg.addr= 0x029;
                reg.data=0x0440;
                    // 3. 写入寄存器
                if (write(fd, &reg, sizeof(struct reg_data)) != sizeof(struct reg_data)) {
                    perror("write reg failed");
                    close(fd);
                    return -1;
                }
                ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);  // 触发单次脉冲
                break;
            }

            case 'x':  
            {
                reg.addr= 0x029;
                reg.data=0x0540;
                    // 3. 写入寄存器
                if (write(fd, &reg, sizeof(struct reg_data)) != sizeof(struct reg_data)) {
                    perror("write reg failed");
                    close(fd);
                    return -1;
                }
                ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);  // 触发单次脉冲
                break;
            }

            case 'r':
            {
                // 1. 配置要读取的寄存器地址（0x00）
                reg.addr = 0x00;
                reg.data = 0;  // 读取时该字段无意义，初始化为0即可

                // 2. 调用驱动read接口读取寄存器值
                ssize_t read_ret = read(fd, &reg, sizeof(struct reg_data));
                if (read_ret != sizeof(struct reg_data)) {
                    perror("read reg 0x00 failed");
                    break;
                }

                // 3. 打印读取结果（格式化输出，便于查看）
                printf("=== Read reg 0x00 success ===\n");
                printf("  Reg addr: 0x%02X\n", reg.addr);
                printf("  Reg value: 0x%04X (decimal: %d)\n", reg.data, reg.data);
                // 额外提示：0x00寄存器是光圈目标值，范围0~0x3FF
                if (reg.data > REG00_MAX) {
                    printf("  Warning: Reg 0x00 value exceeds max (0x3FF)\n");
                }
                break;
            }

                        // ========== 新增：l键 减小光圈（和单片机逻辑一致） ==========
            case 'l':
            {
                // 1. 读取0x00寄存器当前值
                ret = read_an41908_reg(fd, 0x00, &current_iris_val);
                if (ret < 0) {
                    printf("Failed to read reg 0x00 before decrease\n");
                    break;
                }
                printf("Current reg 0x00 value: 0x%04X\n", current_iris_val);

                // 2. 单片机逻辑：≤0x0010则置0，否则减0x10
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
                    //printf("Failed to write reg 0x00 after decrease\n");
                    break;
                }

                // 4. 触发脉冲生效
                ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);
                //printf("IRIS decreased successfully! New value: 0x%04X\n", current_iris_val);
                break;
            }

            // ========== 新增：k键 增大光圈（和单片机逻辑一致） ==========
            case 'k':
            {
                // 1. 读取0x00寄存器当前值
                ret = read_an41908_reg(fd, 0x00, &current_iris_val);
                if (ret < 0) {
                    printf("Failed to read reg 0x00 before increase\n");
                    break;
                }
                printf("Current reg 0x00 value: 0x%04X\n", current_iris_val);

                // 2. 单片机逻辑：≥0x03F0则置0x3FF，否则加0x10
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
                    //printf("Failed to write reg 0x00 after increase\n");
                    break;
                }

                // 4. 触发脉冲生效
                ioctl(fd, AN41908_IOCTL_VD_PULSE, 0);
                //printf("IRIS increased successfully! New value: 0x%04X\n", current_iris_val);
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