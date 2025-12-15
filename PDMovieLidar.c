#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>   // open()
#include <termios.h> // tcgetattr(), tcsetattr()
#include <unistd.h>  // read(), write(), close()
#include <errno.h>   // errno
#include <sys/socket.h>  // socket
#include <netinet/in.h>  // sockaddr_in
#include <arpa/inet.h>   // inet_aton, inet_ntoa
#include <signal.h>      // signal
#include "cJSON.h"   // JSON解析

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

/* 预定义的JSON命令 */
typedef struct {
    const char *command_name;
    unsigned char cmd[32];
    int cmd_len;
    const char *description;
} PredefinedCommand;

/* 支持的完整命令列表 */
PredefinedCommand predefined_commands[] = {
    {
        "query_radar_data",
        {0x01, 0x04, 0x00, 0x1E, 0x00, 0x02, 0x11, 0xCD},
        8,
        "查询当前雷达混合距离数据"
    },
    {
        "remote_control_on",
        {0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x01, 0x00, 0x01, 0x00, 0xF3, 0xC3},
        13,
        "远程控制开启"
    },
    {
        "force_radar_on",
        {0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x0A, 0x00, 0x01, 0x00, 0xF1, 0xE7},
        13,
        "强制雷达开启"
    },
    {
        "range_learning_csp_run",
        {0x01, 0x10, 0x00, 0x00, 0x00, 0x04, 0x08, 0xFF, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x0F, 0x00, 0xF0, 0x30},
        17,
        "行程学习+进入CSP模式+运行"
    },
    {NULL, {}, 0, NULL}
};

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

/* JSON命令处理函数 */
int process_json_command(int serial_fd, const char *json_str)
{
    cJSON *json = NULL;
    cJSON *cmd_item = NULL;
    const char *command = NULL;
    int result = -1;
    
    /* 解析JSON */
    json = cJSON_Parse(json_str);
    if (!json) {
        fprintf(stderr, "[ERROR] JSON解析失败\n");
        return -1;
    }
    
    /* 获取command字段 */
    cmd_item = cJSON_GetObjectItem(json, "command");
    if (!cmd_item || cmd_item->type != cJSON_String) {
        fprintf(stderr, "[ERROR] JSON必须包含'command'字段\n");
        cJSON_Delete(json);
        return -1;
    }
    
    command = cmd_item->valuestring;
    
    /* 查找匹配的预定义命令 */
    for (int i = 0; predefined_commands[i].command_name != NULL; i++) {
        if (strcmp(predefined_commands[i].command_name, command) == 0) {
            const unsigned char *cmd_data = predefined_commands[i].cmd;
            int cmd_len = predefined_commands[i].cmd_len;
            
            printf("\n[INFO] 执行命令: %s (%s)\n", 
                   predefined_commands[i].command_name,
                   predefined_commands[i].description);
            
            /* 打印要发送的命令 */
            printf("[TX] ");
            for (int j = 0; j < cmd_len; j++) {
                printf("%02X ", cmd_data[j]);
            }
            printf("\n");
            
            /* 发送命令 */
            int bytes_written = write(serial_fd, cmd_data, cmd_len);
            if (bytes_written < 0) {
                fprintf(stderr, "[ERROR] 串口发送失败\n");
                cJSON_Delete(json);
                return -1;
            }
            
            /* 接收响应 */
            unsigned char recv_buffer[BUFFER_SIZE];
            usleep(200000);  /* 等待200ms */
            int bytes_received = read(serial_fd, recv_buffer, BUFFER_SIZE);
            
            if (bytes_received > 0) {
                printf("[RX] ");
                for (int j = 0; j < bytes_received; j++) {
                    printf("%02X ", recv_buffer[j]);
                }
                printf("\n");
                
                /* 验证CRC */
                if (bytes_received >= 2) {
                    unsigned short received_crc = (recv_buffer[bytes_received - 1] << 8) | recv_buffer[bytes_received - 2];
                    unsigned short calculated_crc = modbus_crc16(recv_buffer, bytes_received - 2);
                    
                    if (received_crc == calculated_crc) {
                        printf("[INFO] CRC校验成功\n");
                        result = 0;
                    } else {
                        printf("[WARN] CRC校验失败: 接收=0x%04X, 计算=0x%04X\n", 
                               received_crc, calculated_crc);
                        result = 0;  /* 仍然视为成功，因为接收到了数据 */
                    }
                } else {
                    result = 0;
                }
            } else {
                printf("[WARN] 没有接收到响应\n");
                result = 0;
            }
            
            cJSON_Delete(json);
            return result;
        }
    }
    
    /* 命令未找到 */
    fprintf(stderr, "[ERROR] 未知命令: %s\n", command);
    fprintf(stderr, "[INFO] 支持的命令有:\n");
    for (int i = 0; predefined_commands[i].command_name != NULL; i++) {
        fprintf(stderr, "  - %s: %s\n", 
                predefined_commands[i].command_name,
                predefined_commands[i].description);
    }
    
    cJSON_Delete(json);
    return -1;
}

/* UDP 服务器模式 */
void udp_server_mode(int serial_fd)
{
    int udp_socket;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    unsigned char buffer[1024];
    unsigned char response_buffer[1024];
    int bytes_received;
    const char *response_msg;
    const char *allowed_ip = "192.168.200.20";  /* 只允许的IP地址 */

    /* 创建 UDP socket */
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        perror("Error creating UDP socket");
        return;
    }

    /* 设置地址重用 */
    int reuse = 1;
    if (setsockopt(udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        perror("Error setting socket options");
        close(udp_socket);
        return;
    }

    /* 绑定端口 */
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(8080);

    if (bind(udp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Error binding UDP socket");
        close(udp_socket);
        return;
    }

    printf("\n========================================\n");
    printf("UDP 服务器启动\n");
    printf("监听端口: 8080\n");
    printf("只接收来自 %s 的 JSON 命令\n", allowed_ip);
    printf("========================================\n\n");

    while (1) {
        /* 接收 UDP 数据 */
        bytes_received = recvfrom(udp_socket, buffer, sizeof(buffer) - 1, 0,
                                  (struct sockaddr *)&client_addr, &client_addr_len);

        if (bytes_received < 0) {
            perror("Error receiving UDP data");
            continue;
        }

        buffer[bytes_received] = '\0';

        /* 获取客户端 IP 地址 */
        char *client_ip = inet_ntoa(client_addr.sin_addr);
        
        printf("[UDP] 接收到来自 %s:%d 的数据\n",
               client_ip,
               ntohs(client_addr.sin_port));

        /* 检查 IP 地址是否被允许 */
        if (strcmp(client_ip, allowed_ip) != 0) {
            printf("[WARN] 拒绝来自 %s 的连接（不在允许列表中）\n", client_ip);
            
            /* 可选：发送拒绝响应 */
            response_msg = "{\"status\":\"error\",\"message\":\"IP not authorized\"}";
            int response_len = strlen(response_msg);
            sendto(udp_socket, response_msg, response_len, 0,
                   (struct sockaddr *)&client_addr, client_addr_len);
            printf("[UDP] 已发送拒绝响应\n\n");
            continue;
        }

        printf("[UDP] IP 地址验证通过\n");
        printf("[UDP] 数据: %s\n", (char *)buffer);

        /* 处理 JSON 命令 */
        if (process_json_command(serial_fd, (const char *)buffer) == 0) {
            response_msg = "{\"status\":\"success\",\"message\":\"Command executed\"}";
        } else {
            response_msg = "{\"status\":\"error\",\"message\":\"Command failed\"}";
        }

        /* 发送响应 */
        int response_len = strlen(response_msg);
        if (sendto(udp_socket, response_msg, response_len, 0,
                   (struct sockaddr *)&client_addr, client_addr_len) < 0) {
            perror("Error sending UDP response");
        } else {
            printf("[UDP] 发送响应: %s\n\n", response_msg);
        }
    }

    close(udp_socket);
}

/* JSON交互模式 */
void interactive_json_mode(int serial_fd)
{
    char input[512];
    
    printf("\n========================================\n");
    printf("JSON交互模式 - 输入JSON命令或输入'help'查看帮助\n");
    printf("========================================\n\n");
    
    printf("支持的命令:\n");
    for (int i = 0; predefined_commands[i].command_name != NULL; i++) {
        printf("  - %s: %s\n", 
               predefined_commands[i].command_name,
               predefined_commands[i].description);
    }
    printf("\n");
    
    while (1) {
        printf("json> ");
        fflush(stdout);
        
        if (fgets(input, sizeof(input), stdin) == NULL) {
            break;
        }
        
        /* 去除换行符 */
        input[strcspn(input, "\n")] = 0;
        
        /* 检查退出命令 */
        if (strcmp(input, "quit") == 0 || strcmp(input, "exit") == 0) {
            printf("[INFO] 退出JSON模式\n");
            break;
        }
        
        /* 检查帮助命令 */
        if (strcmp(input, "help") == 0) {
            printf("\n支持的命令:\n");
            for (int i = 0; predefined_commands[i].command_name != NULL; i++) {
                printf("  - %s: %s\n", 
                       predefined_commands[i].command_name,
                       predefined_commands[i].description);
            }
            printf("\nJSON格式示例:\n");
            printf("  {\"command\": \"query_radar_data\"}\n");
            printf("  {\"command\": \"remote_control_on\"}\n");
            printf("  {\"command\": \"force_radar_on\"}\n");
            printf("  {\"command\": \"range_learning_csp_run\"}\n\n");
            continue;
        }
        
        /* 跳过空输入 */
        if (strlen(input) == 0) {
            continue;
        }
        
        /* 处理JSON命令 */
        if (process_json_command(serial_fd, input) == 0) {
            printf("[SUCCESS] 命令执行成功\n\n");
        } else {
            printf("[FAILED] 命令执行失败\n\n");
        }
    }
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

    printf("========================================\n");
    printf("%s v%s\n", INFO_STRING, VERSION);
    printf("========================================\n");
    printf("选择模式:\n");
    printf("1. JSON命令模式 (推荐)\n");
    printf("2. 十六进制Modbus模式\n");
    printf("3. UDP网络模式 (监听端口 8080)\n");
    printf("4. 自动测试\n");
    printf("请选择 (1-4): ");
    
    fgets(input_line, sizeof(input_line), stdin);
    input_line[strcspn(input_line, "\n")] = 0;
    
    int mode = atoi(input_line);
    
    switch(mode) {
        case 1:
            /* JSON模式 */
            interactive_json_mode(serial_fd);
            break;
        case 2:
            /* Modbus十六进制模式 */
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
            break;
        case 3:
            /* UDP 网络模式 */
            udp_server_mode(serial_fd);
            break;
        case 4:
            /* 自动测试模式 */
            printf("\n[INFO] 开始自动测试...\n");
            test_setters(serial_fd);
            printf("[INFO] 自动测试完成\n");
            break;
        default:
            printf("无效选择\n");
    }
    
    // 关闭串口
    if (serial_fd > 0)
    {
        close(serial_fd);
    }
    
    return 0;
}