#include "lcd.h"


void lcd_begin(){
	uint8_t data;
	HAL_Delay(50);
	data=0b00111100;
	send_lcd_command(5,data);//Wait 4ms

	data=0b00111100;
	send_lcd_command(1,data);//Wait 100us

	data=0b00111100;
	send_lcd_command(1,data);//Wait 100us

	data=0b00101100;
	send_lcd_command(1,data);//Wait 100us

	data=0b00101100;
	send_lcd_command(1,data);//Wait 100us

	data=0b10001100;
	send_lcd_command(1,data);//NF

	data=0x0C;
	send_lcd_command(1,data);
	//-----------------------------------------------------------
	data=(0xC0)|(1<<2);
	send_lcd_command(5,data);//6b

	data=(0x00)|(1<<2);
	send_lcd_command(1,data);//7a

	data=(0x10)|(1<<2);
	send_lcd_command(1,data);//7b

	data=(0x00)|(1<<2);
	send_lcd_command(1,data);//8a

	data=(0x60)|(1<<2);
	send_lcd_command(5,data);//8b
}

void send_lcd_command(uint8_t delay, uint8_t info){
	HAL_I2C_Master_Transmit(&hi2c1, 0x27<<1, &info, 1, 1000);
	HAL_Delay(1);
	info=info&~(1<<2);
	HAL_I2C_Master_Transmit(&hi2c1, 0x27<<1, &info, 1, 1000);
	HAL_Delay(delay);//Wait 100us
}

void send_msg(char* text) {
    uint8_t data;

    while (*text) {
        data = (*text & 0xF0) | 0x0D | 0x08;
        send_lcd_command(1, data);
        data = ((*text << 4) & 0xF0) | 0x0D | 0x08;
        send_lcd_command(1, data);
        text++;
    }
}

void lcd_clean(){
	uint8_t data=(0x08)|(1<<2);
	HAL_I2C_Master_Transmit(&hi2c1,  0x27<<1 , &data, 1, 1000);
	send_lcd_command(5, data);
	data=(0x18)|(1<<2);
	send_lcd_command(5, data);
}
