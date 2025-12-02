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
unsigned char modbus_auto_focus_mode_close[] = {0x00, 0x00, 0x00, 0x02, 0x04, 0x0C, 0x00, 0x00, 0x00};//纯手动无线对焦模式
unsigned char modbus_auto_focus_mode_hyprid[] = {0x00, 0x00, 0x00, 0x02, 0x04, 0x0C, 0x00, 0x01, 0x00};//双Lidar自动对焦模式（Hybrid mode）
unsigned char modbus_auto_focus_mode_energy[] = {0x00, 0x00, 0x00, 0x02, 0x04, 0x0C, 0x00, 0x02, 0x00};//单Lidar自动对焦模式（Energy mode）

unsigned char modbus_range_learning_control[] = {0x00, 0x00, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0x01, 0x00};//行程学习c
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

    // while(1){
    //     target_position = (i % 2) ? 0xFFFF : 0; // 交替设置目标位置
    //     set_target_position(serial_fd, target_position);
    //     usleep(100000); // 等待100ms
    //     i++;
    //     for(int j=0; j<9; j++){
    //     if (send_modbus_data(serial_fd, NODE_ID, WRITE_L, modbus_position_actual_value, sizeof(modbus_position_actual_value)) < 0)
    //     {
    //         close_serial(serial_fd);
    //         return 1;
    //     }
    //     usleep(100000); // 等待100ms
        
    //     // 接收数据
    //     int bytes_received = read(serial_fd, recv_buffer, BUFFER_SIZE);
    //     if (bytes_received > 0)
    //     {
    //         print_hex("RX: ", recv_buffer, bytes_received);
    //         // printf("Received %d bytes\n", bytes_received);
    //     }
    //     }

    // }

}

// 十六进制字符串转字节数组函数
int hex_string_to_bytes(const char* hex_str, unsigned char* bytes, int max_len)
{
    int len = 0;
    char temp[3] = {0};
    
    while (*hex_str && len < max_len) {
        // 跳过空格
        if (*hex_str == ' ') {
            hex_str++;
            continue;
        }
        
        // 提取两个十六进制字符
        if (hex_str[0] && hex_str[1]) {
            temp[0] = hex_str[0];
            temp[1] = hex_str[1];
            bytes[len++] = (unsigned char)strtol(temp, NULL, 16);
            hex_str += 2;
        } else {
            break;
        }
    }
    
    return len;
}

void interactive_modbus(int serial_fd)
{
    char input[256];
    unsigned char send_data[128];
    unsigned char recv_buffer[BUFFER_SIZE];
    int len;
    
    printf("Enter Modbus hex codes (e.g. 01 06 00 00 00 0A):\n");
    
    while(1) {
        printf("Modbus> ");
        fflush(stdout);
        
        if (fgets(input, sizeof(input), stdin) == NULL) {
            break;
        }
        
        // 去除换行符
        input[strcspn(input, "\n")] = 0;
        
        if (strcmp(input, "quit") == 0) {
            break;
        }
        
        // 将输入的十六进制字符串转换为字节数据
        len = hex_string_to_bytes(input, send_data, sizeof(send_data));
        if (len > 0) {
            // 发送Modbus数据
            write(serial_fd, send_data, len);
            
            // 接收并显示响应
            usleep(100000);
            int bytes_received = read(serial_fd, recv_buffer, BUFFER_SIZE);
            if (bytes_received > 0) {
                print_hex("Response: ", recv_buffer, bytes_received);
            }
        } else {
            printf("Invalid hex format\n");
        }
    }
}

int main()
{
    int serial_fd;
    char input_line[256];

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

    printf("Modbus CRC计算器 - 输入十六进制数据，自动计算CRC并发送\n");
    printf("格式示例: 01 04 00 00 00 02\n");
    printf("输入 'quit' 退出\n");
    
    // 设置终端为行缓冲模式，立即响应输入
    struct termios old_term, new_term;
    tcgetattr(STDIN_FILENO, &old_term);
    new_term = old_term;
    new_term.c_lflag &= ~ICANON;  // 禁用规范模式
    new_term.c_lflag |= ECHO;     // 启用回显
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

    test_setters(serial_fd);
    
    while (1)
    {
        printf("\n请输入Modbus数据: ");
        fflush(stdout);
        
        // 使用fgets获取整行输入
        if (fgets(input_line, sizeof(input_line), stdin) == NULL)
        {
            break;
        }
        
        // 去除换行符
        input_line[strcspn(input_line, "\n")] = 0;
        
        // 检查是否退出
        if (strcmp(input_line, "quit") == 0 || strcmp(input_line, "exit") == 0)
        {
            break;
        }
        
        // 如果输入为空，跳过
        if (strlen(input_line) == 0)
        {
            continue;
        }
        
        // 解析十六进制数据
        unsigned char modbus_data[256];
        int data_count = 0;
        char *token = strtok(input_line, " ");
        
        while (token != NULL && data_count < 256)
        {
            // 支持0x前缀和纯十六进制
            modbus_data[data_count] = (unsigned char)strtol(token, NULL, 16);
            data_count++;
            token = strtok(NULL, " ");
        }
        
        if (data_count == 0)
        {
            printf("错误: 没有有效的十六进制数据\n");
            continue;
        }
        
        // 计算CRC
        unsigned short crc = modbus_crc16(modbus_data, data_count);
        
        // 打印结果
        printf("CRC计算完成: 0x%04X\n", crc);
        printf("完整帧: ");
        for (int i = 0; i < data_count; i++)
        {
            printf("%02X ", modbus_data[i]);
        }
        printf("%02X %02X\n", crc & 0xFF, (crc >> 8) & 0xFF);
        
        // 发送到串口（如果需要）
        if (serial_fd > 0)
        {
            unsigned char send_buf[258];
            memcpy(send_buf, modbus_data, data_count);
            send_buf[data_count] = crc & 0xFF;      // CRC低字节
            send_buf[data_count + 1] = (crc >> 8) & 0xFF; // CRC高字节
            
            // 发送
            int bytes_written = write(serial_fd, send_buf, data_count + 2);
            if (bytes_written > 0)
            {

            }
            else
            {
                perror("发送失败");
            }
        }
        
        // 立即接收并显示响应（非阻塞方式）
        if (serial_fd > 0)
        {
            unsigned char recv_buf[BUFFER_SIZE];
            int bytes_read;
            
            // 尝试读取串口数据，最多等待100ms
            fd_set readfds;
            struct timeval tv;
            
            FD_ZERO(&readfds);
            FD_SET(serial_fd, &readfds);
            tv.tv_sec = 0;
            tv.tv_usec = 100000;  // 100ms
            
            if (select(serial_fd + 1, &readfds, NULL, NULL, &tv) > 0)
            {
                bytes_read = read(serial_fd, recv_buf, BUFFER_SIZE);
                if (bytes_read > 0)
                {
                    printf("接收到响应 (%d 字节): ", bytes_read);
                    for (int i = 0; i < bytes_read; i++)
                    {
                        printf("%02X ", recv_buf[i]);
                    }
                    printf("\n");
                }
            }
        }
    }
    
    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    
    // 关闭串口
    if (serial_fd > 0)
    {
        close(serial_fd);
    }
    
    return 0;
}