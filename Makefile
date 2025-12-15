CC = arm-none-linux-gnueabihf-gcc
CFLAGS = -Wall -Wextra -g
LDFLAGS = -pthread -lm

# 目标文件
TARGET = PDMovieLidar

# 源文件
SOURCES = PDMovieLidar.c cJSON.c
OBJECTS = $(SOURCES:.c=.o)

# 默认目标
all: $(TARGET)

# 构建目标
$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

# 编译源文件
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 清理
clean:
	rm -f $(OBJECTS) $(TARGET)
	@echo "Clean complete"

# 安装（需要root权限）
install: $(TARGET)
	cp $(TARGET) /usr/local/bin/

# 运行程序
run: $(TARGET)
	./$(TARGET)

# PHONY目标
.PHONY: all clean install run
