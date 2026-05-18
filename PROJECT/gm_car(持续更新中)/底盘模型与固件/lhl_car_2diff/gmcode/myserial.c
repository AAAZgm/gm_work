#include "myserial.h"



// 数据缓冲区（用于存储接收的串口数据）
uint8_t readbuffer[BUFFER_SIZE];
// 读索引（指向缓冲区中待读取数据的位置）
uint8_t readIndex = 0;
// 写索引（指向缓冲区中待写入数据的位置）
uint8_t writeIndex = 0;

uint8_t now_command[16]={0};//当前在做的命令，注意清它
/**
* @brief 更新读索引（读取数据后移动读指针）
* @param length 已读取的数据长度
*/
void Command_AddReadIndex(uint8_t length) {
    readIndex += length;
    readIndex %= BUFFER_SIZE;  // 取模运算实现缓冲区循环
}

/**
* @brief 读取缓冲区中指定位置的数据（考虑缓冲区循环特性）
* @param i 相对于读索引的偏移位置
* @return 读取到的字节数据
*/
uint8_t Command_Read(uint8_t i) {
    uint8_t index = i % BUFFER_SIZE;  // 计算实际索引，防止越界
    return readbuffer[index];
}

/**
* @brief 获取缓冲区中未读取的数据长度
* @return 未读取的数据长度
* @retval 0 无未读取数据
* @retval 1~BUFFER_SIZE-1 存在未读取数据
* @retval BUFFER_SIZE 缓冲区已满
*/
uint8_t Command_GetLength() {
    return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;  // 通过公式计算有效数据长度
}

/**
* @brief 获取缓冲区剩余可用空间大小
* @return 剩余空间大小
* @retval 0 缓冲区已满
* @retval 1~BUFFER_SIZE-1 剩余可用空间
* @retval BUFFER_SIZE 缓冲区为空
*/
uint8_t Command_GetRemain() {
    return BUFFER_SIZE - Command_GetLength();  // 总大小减去已用大小得到剩余空间
}

/**
* @brief 向缓冲区写入数据
* @param data 待写入的数据指针
* @param length 待写入的数据长度
* @return 实际写入的数据长度（0表示空间不足）
*/
uint8_t Command_Write(uint8_t *data, uint8_t length) {
    // 若剩余空间不足，返回0（写入失败）
    if (Command_GetRemain() < length) {
        return 0;
    }
    // 分两种情况拷贝数据（避免缓冲区边界问题）
    if (writeIndex + length < BUFFER_SIZE) {
        // 数据未跨缓冲区边界，直接拷贝
        memcpy(readbuffer + writeIndex, data, length);
        writeIndex += length;
    } else {
        // 数据跨缓冲区边界，分两段拷贝
        uint8_t firstLength = BUFFER_SIZE - writeIndex;  // 第一段长度（到缓冲区末尾）
        memcpy(readbuffer + writeIndex, data, firstLength);
        memcpy(readbuffer, data + firstLength, length - firstLength);  // 第二段从缓冲区开头开始
        writeIndex = length - firstLength;  // 更新写索引
    }
    return length;  // 返回实际写入长度
}

/**
* @brief 从缓冲区中解析完整命令（循环等待直到解析到有效命令或退出）
* @param command 用于存储解析出的命令的缓冲区
* @return 解析出的命令长度（0表示未解析到有效命令）
* @retval 0 未解析到有效命令
*/
uint8_t Command_GetCommand(uint8_t *command) {
    // 循环解析命令
    while (1) {
        // 若未读取数据长度小于命令最小长度，返回0
        if (Command_GetLength() < COMMAND_MIN_LENGTH) {
        return 0;
        }
        // 检查命令帧头（0xAA），未找到则移动读索引跳过当前字节
        if (Command_Read(readIndex) != 0xAA) {
        Command_AddReadIndex(1);
        continue;
        }
        // 获取命令长度（帧头后第一个字节）
        uint8_t length = Command_Read(readIndex + 1);
        // 若当前数据长度不足命令长度，返回0
        if (Command_GetLength() < length) {
        return 0;
        }
        // 计算校验和（除校验位外所有字节的和）
        uint8_t sum = 0;
        for (uint8_t i = 0; i < length - 1; i++) {
        sum += Command_Read(readIndex + i);
        }
        // 校验和不匹配，移动读索引跳过当前帧头
        if (sum != Command_Read(readIndex + length - 1)) {
        Command_AddReadIndex(1);
        continue;
        }
        // 校验通过，拷贝命令数据到输出缓冲区
        for (uint8_t i = 0; i < length; i++) {
        command[i] = Command_Read(readIndex + i);
        }
        // 移动读索引，跳过已解析的命令
        Command_AddReadIndex(length);
        return length;  // 返回命令长度
    }
}

/**
* @brief 从缓冲区中解析完整命令（单次尝试解析，不循环等待）
* @param command 用于存储解析出的命令的缓冲区
* @return 解析出的命令长度（0表示未解析到有效命令）
* @retval 0 未解析到有效命令
*/
uint8_t while_Command_GetCommand(uint8_t *command) {
    // 1. 若未读取数据长度小于命令最小长度，返回0
//serial_printf("Command_GetLength()%d, COMMAND_MIN_LENGTH%d\r\n",Command_GetLength(), COMMAND_MIN_LENGTH);
    if (Command_GetLength() < COMMAND_MIN_LENGTH) {
        return 0;
    }
	
			
    
//serial_printf("Command_Read(readIndex)%d,%d\r\n",Command_Read(readIndex),readIndex);
		
    // 2. 检查命令帧头（0xAA），未找到则移动读索引并返回0
    if (Command_Read(readIndex) != 0xAA) {			
        Command_AddReadIndex(1);
        return 0;
    }

    // 3. 获取命令长度，若当前数据不足则返回0,我这里是定长
    uint8_t length =COMMAND_MIN_LENGTH;
   

    // 4. 校验和验证，不匹配则移动读索引并返回0
  
		uint8_t sum = 0;
		for(uint8_t i=0; i<COMMAND_MIN_LENGTH-2; i++){
			 sum += Command_Read(i+readIndex);
			//serial_printf("%X,%d,%d\t",Command_Read(i),i,readIndex);//X！=x
		}
		
//    for (uint8_t i = 0; i < length - 2; i++) {serial_printf("%x,%d,\t",Command_Read(i),readIndex);
//        xor_check ^= Command_Read(readIndex + i);
//    }
	//	serial_printf("xor_check%d,Command_Read(readIndex + length - 2)%d\r\n",sum,Command_Read(readIndex + length - 2));
		
    if (sum != Command_Read(readIndex + length - 2)) {
        Command_AddReadIndex(1);
			//		serial_printf("no");
        return 0;
    }		
		
		
    // 5. 拷贝有效命令到输出缓冲区，并移动读索引
    for (uint8_t i = 0; i < length; i++) {
       command[i] = Command_Read(readIndex + i);
    }
    Command_AddReadIndex(length);
		//serial_printf("ok");
    return length;  // 返回命令长度
}

/**
* @brief 串口格式化输出函数（类似printf）
* @param format 格式化字符串
* @param ... 可变参数列表
如果用static要注意确保是单线程的，而vsnprintf 会自动在末尾加 '\0'，所以strlen不会读乱缓冲区
*/
void serial_printf(char *format, ...)
{
	char String[sp_msg_send];								// 定义发送缓冲区（注意：msg_send需提前定义）
	va_list arg;					  				// 定义可变参数列表指针
	va_start(arg, format);					// 初始化可变参数列表
	vsprintf(String, format, arg);	// 将格式化字符串和可变参数写入缓冲区
	va_end(arg);										// 结束可变参数处理
	HAL_UART_Transmit(&huart1,(uint8_t*)String,strlen(String),0xffff);		// 通过HAL库发送串口数据（超时时间0xffff）
}
//如果用static要注意确保是单线程的，而vsnprintf 会自动在末尾加 '\0'，所以strlen不会读乱缓冲区
void serial_printf2(char *format, ...)
{
	char String[sp_msg_send];								// 定义发送缓冲区（注意：msg_send需提前定义）
	va_list arg;					  				// 定义可变参数列表指针
	va_start(arg, format);					// 初始化可变参数列表
	vsprintf(String, format, arg);	// 将格式化字符串和可变参数写入缓冲区
	va_end(arg);										// 结束可变参数处理
	HAL_UART_Transmit(&huart2,(uint8_t*)String,strlen(String),0xffff);		// 通过HAL库发送串口数据（超时时间0xffff）
}

/*
1. 重定义fputc函数以支持printf输出到串口
int fputc(int ch,FILE*f)
{
Serial_SendByte(ch);  // 调用字节发送函数
return ch;
}

2. 自定义串口格式化输出函数
void Serial_Printf(char *format, ...)
{
	char String[100];								// 定义发送缓冲区
	va_list arg;					  				// 定义可变参数列表指针
	va_start(arg, format);					// 初始化可变参数列表
	vsprintf(String, format, arg);	// 格式化数据到缓冲区
	va_end(arg);										// 结束可变参数处理
	Serial_SendString(String);			// 调用字符串发送函数
}

3. 使用sprintf格式化字符串示例
char buf[50];
sprintf(buf, "电压值%dV 电流值%dmA", 5, 100);  // 将格式化数据写入buf数组
*/
