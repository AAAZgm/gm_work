#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

void Serial3_Init(void);
void Serial3_SendByte(uint8_t Byte);
void Serial3_SendArray(uint8_t *Array, uint16_t Length);
void Serial3_SendString(char *String);
void Seria3l_SendNumber(uint32_t Number, uint8_t Length);
void Serial3_Printf(char *format, ...);


uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);


uint8_t Serial3_GetRxFlag(void);
uint8_t Serial3_GetRxData(void);

#endif
