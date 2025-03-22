#include "stm32f10x.h"                 
#include "stdio.h"

#define ADDRESS 0x23 //Dia chi BH1750
#define POWER_ON 0x01 //Lenh bat nguon cho BH1750
#define CHR_MODE 0x10 //Lenh do lien tuc voi do phan giai cao
#define I2C_TIMEOUT 100000

void I2C_Config(void);
void USART_Config(void);
void USART_SendString(char *str);
void BH1750_Config(void);
void Delay_ms(uint32_t time);
void I2C_Write(uint8_t HW_Addr, uint8_t Sub, uint8_t Data);
uint16_t I2C_Read(uint8_t HW_Addr, uint8_t Sub);
void I2C_Read_Buf(uint8_t HW_Addr, uint8_t *p_buf, unsigned int buf_size);

uint32_t ticks;

void BH1750_Config(void){
	I2C_Config();
	I2C_Write(ADDRESS, 0xFF, POWER_ON);
	I2C_Write(ADDRESS, 0xFF, CHR_MODE);
}

void I2C_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    
	//PB6 - SCL, PB7 - SDA
  GPIO_InitTypeDef gpio;		
  gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  gpio.GPIO_Mode = GPIO_Mode_AF_OD;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio);
    
  I2C_InitTypeDef i2c;
  i2c.I2C_Mode = I2C_Mode_I2C;
  i2c.I2C_DutyCycle = I2C_DutyCycle_2;
  i2c.I2C_OwnAddress1 = 0x00;
  i2c.I2C_Ack = I2C_Ack_Disable;
  i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  i2c.I2C_ClockSpeed = 100000;
  I2C_Init(I2C1, &i2c);
  I2C_Cmd(I2C1, ENABLE);
}

void USART_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef gpio;
    
  //PA9 - Tx
  gpio.GPIO_Pin = GPIO_Pin_9;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpio);

  //PA10 - Rx
  gpio.GPIO_Pin = GPIO_Pin_10;
  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio);

  USART_InitTypeDef uart;
	
  uart.USART_BaudRate = 9600;
  uart.USART_WordLength = USART_WordLength_8b;
  uart.USART_StopBits = USART_StopBits_1;
  uart.USART_Parity = USART_Parity_No;
  uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &uart);
  USART_Cmd(USART1, ENABLE);
}

void USART_SendString(char *str){
	while(*str){
		USART_SendData(USART1, *str);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		str++;
	}
}

void I2C_Write(uint8_t HW_Addr, uint8_t Sub, uint8_t Data){
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	ticks = I2C_TIMEOUT;
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && ticks){
		ticks--;
	}
	if(ticks == 0) return;
	
	ticks = I2C_TIMEOUT;
	I2C_Send7bitAddress(I2C1,	HW_Addr << 1, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && ticks){
		ticks--;
	}
	if(ticks == 0) return;
	
	if(Sub != 0xFF){
		ticks = I2C_TIMEOUT;
		I2C_SendData(I2C1, Sub);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && ticks){
			ticks--;
		}
		if(ticks == 0) return;
	}
	
	ticks = I2C_TIMEOUT;
	I2C_SendData(I2C1, Data);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && ticks){
		ticks--;
	}
	if(ticks == 0) return;
	
	ticks = I2C_TIMEOUT;
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) && ticks){
		ticks--;
	}
	if(ticks == 0) return;
}

uint16_t I2C_Read(uint8_t HW_Addr, uint8_t Sub){
	uint16_t data = 0;
	
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
			
	I2C_Send7bitAddress(I2C1,	HW_Addr << 1, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	if(Sub != 0xFF){
		I2C_SendData(I2C1, Sub);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}
	
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C1, HW_Addr << 1, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data = I2C_ReceiveData(I2C1) << 8;
	
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data |= I2C_ReceiveData(I2C1);

	I2C_GenerateSTOP(I2C1, ENABLE);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	return data;
}

void I2C_Read_Buf(uint8_t HW_Addr, uint8_t *p_buf,	unsigned int buf_size){
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1,	HW_Addr << 1, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	//I2C_Send7bitAddress(I2C1, HW_Addr << 1, I2C_Direction_Receiver);
	uint8_t i;
	for(i = 0; i < buf_size; i++){
		if(i == buf_size - 1){
			I2C_AcknowledgeConfig(I2C1, DISABLE);
		}
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		p_buf[i] = I2C_ReceiveData(I2C1);
	}
	
	//I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
	
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
}

void Delay_ms(uint32_t time){
	while(time){
		TIM_SetCounter(TIM2, 0);
		while(TIM_GetCounter(TIM2) < 1000);
		time--;
	}
}


int main(void){
	I2C_Config();
	USART_Config();
	BH1750_Config();
	char buff[32];
	float light;
	uint16_t light_tmp;
	uint8_t MSB, LSB;
	
	while(1){
		light_tmp = I2C_Read(ADDRESS, 0xFF);
		MSB = (light_tmp >> 8) & 0xFF;
		LSB = light_tmp & 0xFF;
		light = ((MSB << 8) | LSB) / 1.2;
		sprintf(buff, "Do sang: %.2f Lux\n", light);	
    USART_SendString(buff);    
    Delay_ms(1000);
	}
}
