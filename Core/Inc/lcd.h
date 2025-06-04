/*
 * lcd.h
 *
 *  Created on: May 30, 2025
 *      Author: Diego
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"



extern I2C_HandleTypeDef hi2c1;
extern uint8_t data;

void lcd_begin();
void send_lcd_command(uint8_t delay, uint8_t info);
void send_msg(char* text);
void lcd_clean();




#endif /* INC_LCD_H_ */
