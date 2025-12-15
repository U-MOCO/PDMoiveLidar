# 原美道雷达跟焦

## 常用命令

### 使用 Makefile 编译（推荐）
```bash
make clean
make
```

### 直接使用交叉编译器编译
```bash
arm-none-linux-gnueabihf-gcc -g PDMovieLidar.c cJSON.c -o PDMovieLidar -pthread -I . -lm
```

### 原始测试程序编译
```bash
arm-none-linux-gnueabihf-gcc -g UartTest.c -o test -pthread -I . -lm
```

## 引脚

`ttyS4 tx 7 rx 6`

## 协议

## API

### 请求位置
```json
{
    "CMD":"Location"
}
```

``

## 功能

1. 直接运行雷达和跟焦
    1. 使用内部自带的雷达跟焦
    2. 雷达数据到软件，软件驱动马达
2. 雷达拿数据，另外的驱动马达
3. 激光单独拿距离
4. 马达单独用
5. 手动控雷达，但是激光仍然拿数据
6. 请求马达位置
7. 雷达开关
8. 行程学习（跟焦校准）
