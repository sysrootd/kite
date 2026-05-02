// lcd.h
#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include "gpio.h"

#define LCD_RS_PORT    GPIOB
#define LCD_RS_PIN     4
#define LCD_RW_PORT    GPIOB
#define LCD_RW_PIN     5
#define LCD_EN_PORT    GPIOB
#define LCD_EN_PIN     8
#define LCD_D4_PORT    GPIOB
#define LCD_D4_PIN     0
#define LCD_D5_PORT    GPIOB
#define LCD_D5_PIN     1
#define LCD_D6_PORT    GPIOB
#define LCD_D6_PIN     2
#define LCD_D7_PORT    GPIOB
#define LCD_D7_PIN     3

void lcd_init(void);
void lcd_write_cmd(unsigned char data);
void lcd_write_data(unsigned char data);
void lcd_write_str(char str[]);
void itoa(int data, char temp[]);
int atoi(unsigned char str[], int length);
int str_str(unsigned char str1[], unsigned char str2[], int str1_size);
int str_cmp(char str1[], char str2[]);

#endif