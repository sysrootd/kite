// lcd.c
#include "lcd.h"

static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++)
        for (uint32_t j = 0; j < 8000; j++)
            __asm("nop");
}

static void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us; i++)
        for (uint32_t j = 0; j < 8; j++)
            __asm("nop");
}

static void lcd_pulse_enable(void) {
    gpio_write(LCD_EN_PORT, LCD_EN_PIN, 1);
    delay_us(2);
    gpio_write(LCD_EN_PORT, LCD_EN_PIN, 0);
    delay_us(2);
}

static void write_nibble(unsigned char nibble) {
    gpio_write(LCD_D4_PORT, LCD_D4_PIN, (nibble >> 0) & 1);
    gpio_write(LCD_D5_PORT, LCD_D5_PIN, (nibble >> 1) & 1);
    gpio_write(LCD_D6_PORT, LCD_D6_PIN, (nibble >> 2) & 1);
    gpio_write(LCD_D7_PORT, LCD_D7_PIN, (nibble >> 3) & 1);
    lcd_pulse_enable();
}

static void write_byte(unsigned char data, unsigned char is_cmd) {
    gpio_write(LCD_RS_PORT, LCD_RS_PIN, is_cmd ? 0 : 1);
    gpio_write(LCD_RW_PORT, LCD_RW_PIN, 0);
    write_nibble(data >> 4);
    write_nibble(data & 0x0F);
}

void lcd_write_cmd(unsigned char data) {
    write_byte(data, 1);
    if (data == 0x01 || data == 0x02)
        delay_ms(2);
    else
        delay_us(40);
}

void lcd_write_data(unsigned char data) {
    write_byte(data, 0);
    delay_us(40);
}

void lcd_write_str(char str[]) {
    int i = 0;
    while (str[i])
        lcd_write_data(str[i++]);
}

void lcd_init(void) {
    gpio_clk(LCD_RS_PORT);
    gpio_init(LCD_RS_PORT, LCD_RS_PIN, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP, GPIO_SPEED_MEDIUM, GPIO_PULL_NONE, 0);
    gpio_init(LCD_RW_PORT, LCD_RW_PIN, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP, GPIO_SPEED_MEDIUM, GPIO_PULL_NONE, 0);
    gpio_init(LCD_EN_PORT, LCD_EN_PIN, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP, GPIO_SPEED_MEDIUM, GPIO_PULL_NONE, 0);
    gpio_init(LCD_D4_PORT, LCD_D4_PIN, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP, GPIO_SPEED_MEDIUM, GPIO_PULL_NONE, 0);
    gpio_init(LCD_D5_PORT, LCD_D5_PIN, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP, GPIO_SPEED_MEDIUM, GPIO_PULL_NONE, 0);
    gpio_init(LCD_D6_PORT, LCD_D6_PIN, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP, GPIO_SPEED_MEDIUM, GPIO_PULL_NONE, 0);
    gpio_init(LCD_D7_PORT, LCD_D7_PIN, GPIO_MODE_OUTPUT, GPIO_OTYPE_PP, GPIO_SPEED_MEDIUM, GPIO_PULL_NONE, 0);

    delay_ms(15);
    write_nibble(0x03);
    delay_ms(5);
    write_nibble(0x03);
    delay_us(150);
    write_nibble(0x03);
    delay_us(150);
    write_nibble(0x02);
    delay_us(150);

    lcd_write_cmd(0x28);
    lcd_write_cmd(0x0C);
    lcd_write_cmd(0x01);
    delay_ms(2);
    lcd_write_cmd(0x06);
}

void itoa(int data, char temp[]) {
    int i = 0, n;
    if (data == 0) {
        temp[0] = '0';
        temp[1] = '\0';
        return;
    }
    while (data > 0) {
        n = data % 10;
        temp[i++] = n + '0';
        data /= 10;
    }
    temp[i] = '\0';
    i = 0;
    n = 0;
    while (temp[n]) n++;
    n--;
    while (i < n) {
        char t = temp[i];
        temp[i] = temp[n];
        temp[n] = t;
        i++;
        n--;
    }
}

int atoi(unsigned char str[], int length) {
    int result = 0;
    int sign = 1;
    int i = 0;
    while (i < length && (str[i] == ' ' || str[i] == '\t' || str[i] == '\n')) i++;
    if (i < length && str[i] == '-') {
        sign = -1;
        i++;
    } else if (i < length && str[i] == '+') {
        i++;
    }
    while (i < length && str[i] >= '0' && str[i] <= '9') {
        result = result * 10 + (str[i] - '0');
        i++;
    }
    return sign * result;
}

int str_str(unsigned char str1[], unsigned char str2[], int str1_size) {
    int i = 0, j = 0;
    while (i < str1_size) {
        while (str1[i] == str2[j++]) {
            if (str2[j] == '\0')
                return 0;
            i++;
        }
        i++;
        j = 0;
    }
    return 1;
}

int str_cmp(char str1[], char str2[]) {
    int i = 0;
    while (str1[i] == str2[i]) {
        if (str2[i] == '\0')
            return 0;
        i++;
    }
    return str1[i] - str2[i];
}