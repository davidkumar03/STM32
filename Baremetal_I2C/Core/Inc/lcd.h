/*
 * lcd.h
 *
 *  Created on: Jun 4, 2025
 *      Author: davidkumar
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

void lcd_init (void);
void lcd_send_string (char *str);
void setCursor(int a, int b);
void lcd_clear (void);

#endif /* INC_LCD_H_ */
