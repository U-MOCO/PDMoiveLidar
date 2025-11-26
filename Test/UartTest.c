#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>   // open()
#include <termios.h> // tcgetattr(), tcsetattr()
#include <unistd.h>  // read(), write(), close()
#include <errno.h>   // errno

#define SERIAL_PORT "/dev/ttyS4"
#define BAUDRATE B115200
#define BUFFER_SIZE 128

unsigned char modbus_hello[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB};
unsigned char modbus_curposition[] = {0x01, 0x04, 0x00, 0x16, 0x00, 0x02, 0x90, 0x0F};
unsigned char modbus_lidar[] = {0x01, 0x04, 0x00, 0x1E, 0x00, 0x02, 0x11, 0xCD};

// 打开串口
int open_serial(const char *device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("Error opening serial port");
        return -1;
    }
    return fd;
}

// 配置串口参数
int configure_serial(int fd, speed_t baudrate)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0)
    {
        perror("Error getting tty attributes");
        return -1;
    }

    cfsetospeed(&tty, baudrate); // 设置输出波特率
    cfsetispeed(&tty, baudrate); // 设置输入波特率

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8位数据位
    tty.c_cflag |= (CLOCAL | CREAD);            // 启用接收器，忽略调制解调器线状态
    tty.c_cflag &= ~(PARENB | PARODD);          // 无奇偶校验
    tty.c_cflag &= ~CSTOPB;                     // 1个停止位

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
    tty.c_iflag &= ~(ICRNL | INLCR);        // 关闭CR-LF转换

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 关闭规范模式，关闭回显
    tty.c_oflag &= ~OPOST;                          // 关闭输出处理

    tty.c_cc[VMIN] = 0;   // 非阻塞模式
    tty.c_cc[VTIME] = 10; // 1秒超时

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("Error setting tty attributes");
        return -1;
    }

    return 0;
}

// 发送数据
int send_data(int fd, const char *data)
{
    int len = strlen(data);
    int bytes_written = write(fd, data, len);

    if (bytes_written < 0)
    {
        perror("Error writing to serial port");
        return -1;
    }
    return bytes_written;
}

// 发送二进制数据
int send_binary_data(int fd, const unsigned char *data, int len)
{
    int bytes_written = write(fd, data, len);

    if (bytes_written < 0)
    {
        perror("Error writing to serial port");
        return -1;
    }
    return bytes_written;
}

// 打印十六进制数据
void print_hex(const char *prefix, const unsigned char *data, int len)
{
    printf("%s", prefix);
    for (int i = 0; i < len; i++)
    {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

// 接收数据
int receive_data(int fd, char *buffer, int buffer_size)
{
    int bytes_read = read(fd, buffer, buffer_size);

    if (bytes_read < 0)
    {
        perror("Error reading from serial port");
        return -1;
    }
    return bytes_read;
}

// 关闭串口
void close_serial(int fd)
{
    if (close(fd) != 0)
    {
        perror("Error closing serial port");
    }
}

int main()
{
    int serial_fd;
    unsigned char recv_buffer[BUFFER_SIZE];

    // 打开串口
    serial_fd = open_serial(SERIAL_PORT);
    if (serial_fd < 0)
        return 1;

    // 配置串口
    if (configure_serial(serial_fd, BAUDRATE) != 0)
    {
        close_serial(serial_fd);
        return 1;
    }

    printf("Serial port %s opened and configured successfully.\n", SERIAL_PORT);

    // Modbus指令: 01 04 00 00 00 02 71 CB
    // 功能码04: 读取输入寄存器
    while (1)
    {
        int cmd_len = sizeof(modbus_curposition);

        // 发送Modbus指令
        // printf("Sending Modbus command to %s:\n", SERIAL_PORT);
        // print_hex("TX: ", modbus_curposition, cmd_len);

        if (send_binary_data(serial_fd, modbus_curposition, cmd_len) < 0)
        {
            close_serial(serial_fd);
            return 1;
        }

        // 等待设备响应
        usleep(100000); // 等待100ms

        // 接收数据
        int bytes_received = read(serial_fd, recv_buffer, BUFFER_SIZE);
        if (bytes_received > 0)
        {
            print_hex("RX: ", recv_buffer, bytes_received);
            // printf("Received %d bytes\n", bytes_received);
        }
        else if (bytes_received == 0)
        {
            printf("No data received (timeout).\n");
        }
        else
        {
            perror("Error reading from serial port");
        }
    }

    // 关闭串口
    close_serial(serial_fd);

    return 0;
}