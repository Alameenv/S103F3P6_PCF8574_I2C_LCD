/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */
#include "stm8s.h"
#include "i2c_lcd.h"

void clock_Setup(void);

uint8_t loveEmpty[] = {
	
	0b01010,
	0b10101,
	0b10001,
	0b01010,
	0b00100,
	0b00000,
	0b00000,
	0b00000
};
uint8_t LoveFilled []=
{
	0b01010,
	0b11111,
	0b11111,
	0b01110,
	0b00100,
	0b00000,
	0b00000,
	0b00000,
};
void main(void)
{


	clock_Setup();
	i2c_lcd_Init();
	
	// 16x02 LCD 
	createChar(0,loveEmpty);
	getChar( 14 , 1, 0 );
	createChar(1,LoveFilled);
	getChar( 14 , 2, 1 );
	
	gotoXY(1,1);
	i2c_lcd_Print("Hello World!");
	
	gotoXY(1,2);
	i2c_lcd_Print("int:");
	i2c_lcd_Print_Int( 6, 2, 123, 4  );

	while (1);
}

// clock
void clock_Setup(void){
	CLK_DeInit();
	CLK_HSECmd(DISABLE);
	CLK_LSICmd(DISABLE);
	CLK_HSICmd(ENABLE);
	while(CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == FALSE);
	
	CLK_ClockSwitchCmd(ENABLE);
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV8);
	CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
	
	CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO,CLK_SOURCE_HSI,DISABLE,CLK_CURRENTCLOCKSTATE_ENABLE);
	
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C,ENABLE);
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI,DISABLE);
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC,DISABLE);
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_AWU,DISABLE);
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1,DISABLE);
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1,DISABLE);
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2,DISABLE);
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4,DISABLE);
	
}
