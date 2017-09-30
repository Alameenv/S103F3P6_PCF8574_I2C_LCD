#ifndef _I2C_LCD_H
#define _I2C_LCD_H

#include "stm8s.h"

#define lcdType20x4
//#define lcdType16x2

#define RS		0b00000001	//PCF8574T I2C IC pin -> P0		// RS -> 0 cmd , 1 data
#define RW		0b00000010//PCF8574T I2C IC pin -> P1// RW -> 0 write , 1 read  ==> Not used and set to 0 -> write
#define EN		0b00000100	// PCF8574T I2C IC pin -> P2

#define PCF8574T_addr	0x4E		//write address - PCF8574T

#define _backlight  0x08

#define CMD		0
#define DATA	1

#define DISPLAY_OFF		0b00001000
#define DISPLAY_ON		0b00001100
#define CURSOR_SHOW		0b00000010
#define CURSOR_BLINK	0b00000001
#define CURSOR_NONE		0b00000000



void i2c_gpio_Setup(void);
void i2c_lcd_Setup(void);
void i2c_Write(unsigned char cmd);
//uint8_t i2c_Read( void );
void delay_loop( uint16_t nCount );

void expanderWrite(uint8_t _data);
void pulseWrite( uint8_t portPulse );
void sendByte( uint8_t byte , uint8_t mode );
void i2c_lcd_Init(void);
void i2c_lcd_Print(char *str);
void gotoXY( uint8_t X,  uint8_t Y );
void createChar(  uint8_t location, uint8_t charMap[] );
void getChar( uint8_t X, uint8_t Y, uint8_t location );
void i2c_lcd_Print_Int( uint8_t X, uint8_t Y, int val,int8_t field_length  );

void i2c_gpio_Setup(){
	GPIO_DeInit(GPIOB);
	GPIO_Init(GPIOB,GPIO_PIN_4,GPIO_MODE_OUT_OD_HIZ_FAST);  // SCL
	GPIO_Init(GPIOB,GPIO_PIN_5,GPIO_MODE_OUT_OD_HIZ_FAST);	// SDA
}

// i2c
void i2c_lcd_Setup(void){
	I2C_DeInit();
	I2C_Init(100000,PCF8574T_addr,I2C_DUTYCYCLE_2,
						I2C_ACK_CURR,I2C_ADDMODE_7BIT,
						(CLK_GetClockFreq()/1000000));
						
	I2C_Cmd(ENABLE);
}

void i2c_Write(unsigned char cmd){	
	

	I2C_GenerateSTART(ENABLE);
	while(! I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(PCF8574T_addr,I2C_DIRECTION_TX);
	while(! I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	I2C_SendData(cmd);
	while(! I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_GenerateSTOP(ENABLE);
}

/*
uint8_t i2c_Read( void ){
	uint8_t readByte =0;
	while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
	
	I2C_GenerateSTART(ENABLE);
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(PCF8574T_addr,I2C_DIRECTION_RX);	//0x4F  PCF8574T_addr
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	readByte = I2C_ReceiveData();
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
	
	
	
	I2C_AcknowledgeConfig(I2C_ACK_NONE);
	while (I2C_GetFlagStatus( I2C_FLAG_TRANSFERFINISHED) == RESET);
	
	I2C_GenerateSTOP(ENABLE);
	
	return readByte;
}*/

void delay_loop( uint16_t nCount  ){

	while (nCount != 0)
  {
    nCount--;
  }
}

void expanderWrite(uint8_t _data){
	i2c_Write( _data | _backlight  );
}

void pulseWrite( uint8_t portPulse ){
	
	portPulse |=  EN ;
	expanderWrite( portPulse ) ;
	
	delay_loop( 0x0010 );
	
	portPulse &= ~EN;
	expanderWrite( portPulse );
	
}

void sendByte( uint8_t byte , uint8_t mode ){
	
	uint8_t mask_lower_nibble , swap_mask_Lower_nibble , read_port;
	
	mask_lower_nibble = byte;
	
	// Shift 4 place to the left and Mask lower byte -> SWAP and MASK
	swap_mask_Lower_nibble =   ( byte << 4 ) & 0xF0 ;		
	
	mask_lower_nibble		&=	0xF0;			// Mask lower nibble
	
	read_port		=	0b00000101;
	read_port		|=	mask_lower_nibble;
	expanderWrite( read_port );

	if( mode == CMD ){
		read_port &= ~( RS ) ;		// RS = 0 -> CMD
		expanderWrite( read_port );
	}
	if( mode == DATA ){
		read_port |= (  RS ) ;		// RS = 1 -> DATA
		expanderWrite( read_port );
	}
	pulseWrite( read_port );
	
	// SECOND HALF
	read_port		=	0b00000101;
	read_port		|=	swap_mask_Lower_nibble;
	expanderWrite( read_port );
	
	if( mode == CMD ){
		read_port &= ~(  RS ) ;		// RS = 0 -> CMD
		expanderWrite( read_port );
	}
	if( mode == DATA ){
		read_port |= (  RS ) ;		// RS = 1 -> DATA
		expanderWrite( read_port );
	}
	pulseWrite( read_port );
	
	delay_loop( 0x0100 );
}

void i2c_lcd_Init(void){
	i2c_gpio_Setup();
	i2c_lcd_Setup();
	#ifdef lcdType20x4
	delay_loop( 0x00FF );  //HSI 2MHz ( prescaler 8 ) and SysClk 1MHz ( prescaler 2 ) 
	sendByte( 0x28 , CMD );
	sendByte( 0x06 , CMD );
	sendByte( 0x02 , CMD );
	sendByte( 0x01 , CMD );
	sendByte( DISPLAY_OFF | CURSOR_SHOW , CMD );
	sendByte( DISPLAY_ON | CURSOR_SHOW , CMD );
	#endif
	
	#ifdef lcdType16x2
	//HSI 2MHz ( prescaler 8 ) and SysClk 1MHz ( prescaler 2 ) 
	sendByte( 0x28 , CMD );
	sendByte( 0x02 , CMD );
	sendByte( 0x01 , CMD ); // 0x01
	sendByte( DISPLAY_OFF | CURSOR_SHOW , CMD );
	sendByte( DISPLAY_ON | CURSOR_SHOW , CMD );
	#endif
	
}

void i2c_lcd_Print(char *str){
	while(*str ){
		sendByte( *str++ , DATA );
	}
}

void gotoXY( uint8_t X,  uint8_t Y ){
	
	char LcdTypeOne[ 4 ] = { 0 , 64 , 20 , 84 }; // 20 x 4 LCD
	char LcdTypeTwo[ 2 ] = { 0 , 64 }; // 16 x 2 LCD
	
	#ifdef	lcdType20x4
		if( X >= 21) return;
		sendByte( 0x80 + LcdTypeOne[ Y - 1 ] + ( X - 1 ) , CMD );
	#endif

	#ifdef  lcdType16x2
		if( X >= 17 ) return;
		sendByte(0x80 + LcdTypeTwo[ Y - 1 ] + ( X - 1 ) , CMD );
	#endif
}

void createChar(  uint8_t location, uint8_t charMap[] ) {
	uint8_t i = 0;
	location &= 0x07; // we only have 8 locations 0-7
	sendByte( 0x40 | ( location << 3 ), CMD );
	
	delay_loop( 0x0100 );
	
	for (  i ; i < 8 ; i++ ) {
		sendByte( charMap[i] , DATA );
	}
}
void getChar( uint8_t X, uint8_t Y, uint8_t location ){
	gotoXY( X, Y );
	sendByte( location , DATA);
}
void i2c_lcd_Print_Int( uint8_t X, uint8_t Y, int val,int8_t field_length  ){
	
	char str[5] = {0,0,0,0,0};
	int i = 4 ,j = 0;
	
	gotoXY( X , Y );
	//Handle negative integers
	if( val < 0 )
	{
		sendByte( '-' , DATA );   //Write Negative sign
		val *=  ( - 1 );     //convert to positive
	}
	else
	{
		sendByte( ' ', DATA );
	}

	while( val )
	{
		str[i] = val % 10;
		val = val / 10;
		i--;
	}

	if( field_length == -1 )
	while( str[j] == 0 ) j++;
	else
	j = 5 - field_length;

	for( i = j; i < 5; i++ )
	{
		sendByte( '0' + str[i] , DATA );
	}
}

#endif