#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>   // open()
#include <termios.h> // tcgetattr(), tcsetattr()
#include <unistd.h>  // read(), write(), close()
#include <errno.h>   // errno

/* #region 调试宏定义 */

// 调试级别定义: 0=关闭, 1=ERROR, 2=WARNING, 3=INFO, 4=DEBUG
#define DEBUG_LEVEL 3
struct timeval LOG_tv;

#define LOG_ERROR(fmt, ...)                                                                 \
    do                                                                                      \
    {                                                                                       \
        if (DEBUG_LEVEL >= 1)                                                               \
            fprintf(stderr, "[ERROR] %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
    } while (0)

#define LOG_WARN(fmt, ...)                                                    \
    do                                                                        \
    {                                                                         \
        if (DEBUG_LEVEL >= 2)                                                 \
            fprintf(stdout, "[WARN] %s: " fmt "\n", __func__, ##__VA_ARGS__); \
    } while (0)

#define LOG_INFO(fmt, ...)                                                    \
    do                                                                        \
    {                                                                         \
        if (DEBUG_LEVEL >= 3)                                                 \
            fprintf(stdout, "[INFO] %s: " fmt "\n", __func__, ##__VA_ARGS__); \
    } while (0)

#define LOG_DEBUG(fmt, ...)                                                                 \
    do                                                                                      \
    {                                                                                       \
        if (DEBUG_LEVEL >= 4)                                                               \
            fprintf(stdout, "[DEBUG] %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
    } while (0)

/* #endregion */

#define VERSION "1.0.0"
#define INFO_STRING "PD Movie Lidar Controller"

#define NODE_ID 0x01
#define READ 0x04
#define WRITE_S 0x06
#define WRITE_L 0x10
#define SERIAL_PORT "/dev/ttyS4"
#define BAUDRATE B115200
#define BUFFER_SIZE 128

//读
unsigned char modbus_product_code[] = {0x00, 0x00, 0x00, 0x02};
unsigned char modbus_current_operation_mode[] = {0x00, 0x08, 0x00, 0x01};//当前用户动作
unsigned char modbus_position_actual_value[] = {0x00, 0x16, 0x00, 0x02};//读取当前位置
unsigned char modbus_mixed_radar_data[] = {0x00, 0x1E, 0x00, 0x02};//读取混合雷达数据

//写
int auto_focus_mode = 0; // 0=close, 1=Hybrid mode, 2=Energy mode
int remote_control = 0; // 0=off, 1=on
int target_position = 0; // 目标位置
int operation_mode = 8; // 0=待机 8=位置控制模式 9=速度控制模式

unsigned char modbus_remote_control_on[] = {0x00, 0x00, 0x00, 0x02, 0x04, 0x01, 0x00, 0x01, 0x00};//远程控制启动
unsigned char modbus_auto_focus_mode_close[] = {0x00, 0x00, 0x00, 0x02, 0x04, 0x0A, 0x00, 0x0, 0x00};//自动对焦模式关闭
unsigned char modbus_range_learning_control[] = {0x00, 0x00, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0x01, 0x00};//行程学习
unsigned char modbus_operation_mode_csp[] = {0x00, 0x02, 0x00, 0x02, 0x04, 0x08, 0x00, 0x0F, 0x00};//位置控制模式
unsigned char modbus_target_position[] = {0x00, 0x04, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00,};//设置目标位置

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

unsigned short modbus_crc16(const unsigned char *data, int len)
{
    unsigned short crc = 0xFFFF;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
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

// 发送modbus数据
int send_modbus_data(int fd, char node_id, char cmd, const unsigned char *info, int info_len)
{
    unsigned char buf[256];

    buf[0] = node_id & 0xFF;
    buf[1] = cmd & 0xFF;
    memcpy(&buf[2], info, info_len);
    unsigned short crc = modbus_crc16(buf, 2 + info_len);
    buf[2 + info_len] = crc & 0xFF;
    buf[3 + info_len] = (crc >> 8) & 0xFF;

    int bytes_written = write(fd, buf, 4 + info_len);

    if (bytes_written < 0)
    {
        perror("Error writing to serial port");
        return -1;
    }
    else
    {
        print_hex("TX: ", buf, 4 + info_len);
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

void set_target_position(int fd, int position)
{
    unsigned char cmd[9];
    memcpy(cmd, modbus_target_position, sizeof(modbus_target_position));
    // 设置目标位置（假设目标位置为2字节，高字节在cmd[5]，低字节在cmd[6]）
    cmd[5] = (position >> 8) & 0xFF;
    cmd[6] = position & 0xFF;
    send_modbus_data(fd, NODE_ID, WRITE_L, cmd, sizeof(cmd));
}

int test_setters(int serial_fd)
{
    unsigned char recv_buffer[BUFFER_SIZE];
    int i = 0;
    send_modbus_data(serial_fd, NODE_ID, WRITE_L, modbus_remote_control_on, sizeof(modbus_remote_control_on));
    usleep(500000); // 等待100ms

    send_modbus_data(serial_fd, NODE_ID, WRITE_L, modbus_range_learning_control, sizeof(modbus_range_learning_control));
    for(int j=0; j<10; j++){
        sleep(1); // 等待1秒
        printf("Learning... %d/10\n", j+1);
    }
    
    send_modbus_data(serial_fd, NODE_ID, WRITE_L, modbus_operation_mode_csp, sizeof(modbus_operation_mode_csp));
    usleep(500000); // 等待500ms

    while(1){
        target_position = (i % 2) ? 0xFFFF : 0; // 交替设置目标位置
        set_target_position(serial_fd, target_position);
        usleep(100000); // 等待100ms
        i++;
        for(int j=0; j<9; j++){
        if (send_modbus_data(serial_fd, NODE_ID, WRITE_L, modbus_position_actual_value, sizeof(modbus_position_actual_value)) < 0)
        {
            close_serial(serial_fd);
            return 1;
        }
        usleep(100000); // 等待100ms
        
        // 接收数据
        int bytes_received = read(serial_fd, recv_buffer, BUFFER_SIZE);
        if (bytes_received > 0)
        {
            print_hex("RX: ", recv_buffer, bytes_received);
            // printf("Received %d bytes\n", bytes_received);
        }
        }

    }

}

int main()
{
    int serial_fd;

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

    printf("Version: %s, Info: %s\n", VERSION, INFO_STRING);

    test_setters(serial_fd);

    // 关闭串口
    close_serial(serial_fd);

    return 0;
}
