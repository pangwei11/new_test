#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <termios.h>  // 终端配置头文件

// 复制驱动层的ioctl定义（保持一致）
#define AN41908_IOCTL_MAGIC 'F'
struct an41908_focus_param {
    bool forward;
    int step;
};
#define AN41908_IOCTL_SET_FOCUS_PRECISE _IOW(AN41908_IOCTL_MAGIC, 0, struct an41908_focus_param)

// 配置终端为无缓冲模式（按按键立即响应，无需回车）
static void set_terminal_mode(void) {
    struct termios ctrl;
    tcgetattr(STDIN_FILENO, &ctrl);
    ctrl.c_lflag &= ~ICANON;  // 关闭行缓冲
    ctrl.c_lflag &= ~ECHO;    // 关闭回显
    tcsetattr(STDIN_FILENO, TCSANOW, &ctrl);
}

int main(void)
{
    int fd = open("/dev/AN41908", O_RDWR);
    if (fd < 0) {
        perror("open /dev/AN41908 failed");
        return -1;
    }

    // 配置终端，支持实时按键监听
    set_terminal_mode();
    printf("AN41908 Focus Test\n");
    printf("Press 'z' for Focus Near, 'x' for Focus Far, 'Ctrl+C' to exit\n");

    struct an41908_focus_param param = {
        .step = 0x40  
    };

    while (1) {
        char ch;
        // 读取1个按键（无缓冲模式）
        int ret = read(STDIN_FILENO, &ch, 1);
        if (ret <= 0) continue;

        switch (ch) {
            case 'z':  // 对焦近
                printf("Focus near...\n");
                param.forward = 0;
                ret = ioctl(fd, AN41908_IOCTL_SET_FOCUS_PRECISE, &param);
                if (ret < 0) perror("ioctl focus near failed");
                break;

            case 'x':  // 对焦远
                printf("Focus far...\n");
                param.forward = 1;
                ret = ioctl(fd, AN41908_IOCTL_SET_FOCUS_PRECISE, &param);
                if (ret < 0) perror("ioctl focus far failed");
                break;

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