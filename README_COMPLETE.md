# PDMovieLidar 使用说明

## 快速开始

```bash
make
./PDMovieLidar
```

按提示选择模式（1-4）。

---

## 支持的命令

所有模式都支持这4个命令：

```
query_radar_data          # 查询雷达数据
remote_control_on         # 开启远程控制
force_radar_on            # 强制雷达开启
range_learning_csp_run    # 行程学习+CSP+运行
```

---

## 4种模式说明

### 模式1：JSON本地模式

本地交互式，输入 JSON 命令：

```json
{"command": "query_radar_data"}
```

内置命令：`help`（帮助）、`quit`（退出）

---

### 模式2：Modbus十六进制模式

输入十六进制 Modbus 命令，自动计算 CRC：

```
01 04 00 1E 00 02
```

---

### 模式3：UDP网络模式

远程通过网络控制。

**配置**：
- 协议：UDP
- 端口：8080
- 允许的客户端 IP：192.168.200.20

**使用 NetAssist**：
1. IP 地址：192.168.200.7（设备IP）
2. 端口：8080
3. 发送 JSON 命令

**响应**：
- 成功：`{"status":"success","message":"Command executed"}`
- 失败：`{"status":"error","message":"Command failed"}`
- IP拒绝：`{"status":"error","message":"IP not authorized"}`

**Python 发送**：
```python
import socket, json

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd = json.dumps({"command": "query_radar_data"})
sock.sendto(cmd.encode(), ("192.168.200.7", 8080))
response, _ = sock.recvfrom(1024)
print(response.decode())
sock.close()
```

---

### 模式4：自动测试

运行预定义的测试序列。

---

## 常见问题

| 问题 | 解决方案 |
|------|--------|
| 无法打开串口 | 检查 `/dev/ttyS4` 权限：`sudo chmod 666 /dev/ttyS4` |
| 无响应 | 检查设备连接、设备是否开启、波特率（115200） |
| UDP 无法连接 | 检查设备 IP、UDP 端口 8080、防火墙 |
| IP 被拒绝 | 确保发送方 IP 是 192.168.200.20（只有此 IP 被允许） |
| JSON 格式错误 | 确保使用双引号：`{"command": "..."}` |

---

## 技术参数

- 串口：`/dev/ttyS4`，波特率 115200
- CRC16：多项式 0xA001，初值 0xFFFF
- 响应超时：200ms
