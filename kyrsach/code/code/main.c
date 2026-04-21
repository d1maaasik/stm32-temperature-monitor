#include "stm32f10x.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#define RX_BUFFER_SIZE      128
#define HISTORY_SIZE        8
#define HISTORY_STR_LEN     64

/* ????????? ??????????? */
#define TEMP_WARNING        30      /* ????? ?????????????? 30°C */
#define TEMP_CRITICAL       40      /* ??????????? ????? 40°C */

static volatile char rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_index = 0;
static volatile uint8_t line_ready = 0;

static char history[HISTORY_SIZE][HISTORY_STR_LEN];
static uint8_t history_count = 0;
static uint8_t history_start = 0;

typedef enum
{
    OP_NONE = 0,
    OP_ADD,
    OP_SUB,
    OP_MUL,
    OP_DIV
} Operation_t;

/* ?????????? ?????????? ??? ??????????? */
static volatile int16_t current_temperature = 0;
static volatile uint8_t update_temp = 1;

/* ===================== Delay ===================== */
static void Delay(volatile uint32_t value)
{
    while (value--);
}

/* ===================== UART (??????? ??????????) ===================== */
static void USART2_SendChar(char c);
static void USART2_SendString(const char *str);

/* ===================== LED ===================== */
static void LED_Init(void)
{
    /* ???????????? GPIOA */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* PA0, PA1 -> ????? Push-Pull, 50 MHz */
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
                    GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    
    GPIOA->CRL |= (GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1 |
                   GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1);

    /* ????????? ??? LED */
    GPIOA->ODR &= ~(GPIO_ODR_ODR0 | GPIO_ODR_ODR1);
}

static void LED_Off(void)
{
    GPIOA->ODR &= ~(GPIO_ODR_ODR0 | GPIO_ODR_ODR1);
}

static void LED_Success(void)
{
    GPIOA->ODR |= GPIO_ODR_ODR0;   /* ??????? ??????? */
    GPIOA->ODR &= ~GPIO_ODR_ODR1;  /* ??????? ???????? */
}

static void LED_Error(void)
{
    GPIOA->ODR |= GPIO_ODR_ODR1;   /* ??????? ??????? */
    GPIOA->ODR &= ~GPIO_ODR_ODR0;  /* ??????? ???????? */
}

static void LED_TempUpdate(int16_t temp)
{
    static uint32_t blink_counter = 0;
    
    if (temp >= TEMP_CRITICAL * 10)
    {
        /* ??????????? ??????????? - ??????? ????? ????????? */
        GPIOA->ODR |= GPIO_ODR_ODR1;
        GPIOA->ODR &= ~GPIO_ODR_ODR0;
    }
    else if (temp >= TEMP_WARNING * 10)
    {
        /* ?????????????? - ??????? ?????? */
        blink_counter++;
        if (blink_counter > 30000)
        {
            blink_counter = 0;
            if (GPIOA->ODR & GPIO_ODR_ODR1)
                GPIOA->ODR &= ~GPIO_ODR_ODR1;
            else
                GPIOA->ODR |= GPIO_ODR_ODR1;
            GPIOA->ODR &= ~GPIO_ODR_ODR0;
        }
    }
    else
    {
        /* ?????????? ??????????? - ??????? ????? */
        GPIOA->ODR |= GPIO_ODR_ODR0;
        GPIOA->ODR &= ~GPIO_ODR_ODR1;
    }
}

/* ===================== ADC ??? LM35 ===================== */
static void ADC_Init(void)
{
    /* ???????? ???????????? ADC1 ? GPIOA */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    
    /* ????????? PA4 ??? ?????????? ???? */
    GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
    GPIOA->CRL |= GPIO_CRL_CNF4_0;  /* ?????????? ????? */
    
    /* ????? ADC */
    RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
    
    /* ????????? ADC */
    ADC1->CR1 = 0;                   
    ADC1->CR2 = ADC_CR2_ADON;       /* ???????? ADC */
    
    /* ?????????? */
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);
    
    /* ???????? ????? 4 (PA4) */
    ADC1->SQR3 = 4;
    ADC1->SQR2 = 0;
    ADC1->SQR1 = 0;
    
    /* ????? ??????? 55.5 ?????? */
    ADC1->SMPR2 |= ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2;
}

static uint16_t ADC_Read(void)
{
    /* ????????? ?????????????? */
    ADC1->CR2 |= ADC_CR2_ADON;
    
    /* ???? ?????????? */
    while (!(ADC1->SR & ADC_SR_EOC));
    
    /* ?????????? ????????? */
    return (uint16_t)ADC1->DR;
}

static int16_t GetTemperature(void)
{
    uint16_t adc_value;
    int16_t temperature;
    uint32_t sum = 0;
    int i;
    
    /* ?????? 20 ????????? ??? ???????????? */
    for (i = 0; i < 20; i++)
    {
        sum += ADC_Read();
        Delay(500);
    }
    adc_value = (uint16_t)(sum / 20);
    
    /* 
     * ??????? ??? LM35:
     * ??????????? (? ??????? ?????) = adc_value * 3300 * 10 / 4095 / 10
     * ????????: temperature = (adc_value * 33) / 41
     */
    temperature = (int16_t)((adc_value * 33 * 151) / (41 * 100));
    
    return temperature;
}

/* ===================== UART ===================== */
static void USART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

static void USART2_SendString(const char *str)
{
    while (*str)
    {
        USART2_SendChar(*str++);
    }
}

static void USART2_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOA->CRL |= GPIO_CRL_MODE2_1;
    GPIOA->CRL |= GPIO_CRL_CNF2_1;

    GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
    GPIOA->CRL |= GPIO_CRL_CNF3_0;

    USART2->BRR = 0x0EA6;

    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART2->CR1 |= USART_CR1_UE;

    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE)
    {
        char c = (char)USART2->DR;

        if ((c == '\b' || c == 127) && rx_index > 0)
        {
            rx_index--;
            USART2_SendString("\b \b");
            return;
        }

        if (line_ready)
            return;

        if (c == '\r' || c == '\n')
        {
            USART2_SendString("\r\n");
            rx_buffer[rx_index] = '\0';
            rx_index = 0;
            line_ready = 1;
            return;
        }

        if (rx_index < RX_BUFFER_SIZE - 1)
        {
            rx_buffer[rx_index++] = c;
            USART2_SendChar(c);
        }
    }
}

/* ===================== LCD ===================== */
#define LCD_RS_HIGH()   (GPIOB->BSRR = GPIO_BSRR_BS0)
#define LCD_RS_LOW()    (GPIOB->BSRR = GPIO_BSRR_BR0)
#define LCD_E_HIGH()    (GPIOB->BSRR = GPIO_BSRR_BS1)
#define LCD_E_LOW()     (GPIOB->BSRR = GPIO_BSRR_BR1)
#define LCD_D4_HIGH()   (GPIOB->BSRR = GPIO_BSRR_BS12)
#define LCD_D4_LOW()    (GPIOB->BSRR = GPIO_BSRR_BR12)
#define LCD_D5_HIGH()   (GPIOB->BSRR = GPIO_BSRR_BS13)
#define LCD_D5_LOW()    (GPIOB->BSRR = GPIO_BSRR_BR13)
#define LCD_D6_HIGH()   (GPIOB->BSRR = GPIO_BSRR_BS14)
#define LCD_D6_LOW()    (GPIOB->BSRR = GPIO_BSRR_BR14)
#define LCD_D7_HIGH()   (GPIOB->BSRR = GPIO_BSRR_BS15)
#define LCD_D7_LOW()    (GPIOB->BSRR = GPIO_BSRR_BR15)

static void LCD_SetData4(uint8_t nibble)
{
    if (nibble & 0x01) LCD_D4_HIGH(); else LCD_D4_LOW();
    if (nibble & 0x02) LCD_D5_HIGH(); else LCD_D5_LOW();
    if (nibble & 0x04) LCD_D6_HIGH(); else LCD_D6_LOW();
    if (nibble & 0x08) LCD_D7_HIGH(); else LCD_D7_LOW();
}

static void LCD_EnablePulse(void)
{
    LCD_E_HIGH();
    Delay(3000);
    LCD_E_LOW();
    Delay(3000);
}

static void LCD_Send4Bits(uint8_t nibble)
{
    LCD_SetData4(nibble & 0x0F);
    LCD_EnablePulse();
}

static void LCD_SendCommand(uint8_t cmd)
{
    LCD_RS_LOW();
    LCD_Send4Bits((cmd >> 4) & 0x0F);
    LCD_Send4Bits(cmd & 0x0F);
    Delay(30000);
}

static void LCD_SendData(uint8_t data)
{
    LCD_RS_HIGH();
    LCD_Send4Bits((data >> 4) & 0x0F);
    LCD_Send4Bits(data & 0x0F);
    Delay(30000);
}

static void LCD_Clear(void)
{
    LCD_SendCommand(0x01);
    Delay(100000);
}

static void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_SendCommand(addr);
}

static void LCD_Print(const char *str)
{
    while (*str)
    {
        LCD_SendData((uint8_t)*str++);
    }
}

static void LCD_PrintLine(uint8_t row, const char *str)
{
    char buf[17];
    uint8_t i = 0;

    while (i < 16)
    {
        if (*str)
            buf[i++] = *str++;
        else
            buf[i++] = ' ';
    }
    buf[16] = '\0';

    LCD_SetCursor(row, 0);
    LCD_Print(buf);
}

static void LCD_ShowTwoLines(const char *line1, const char *line2)
{
    LCD_PrintLine(0, line1);
    LCD_PrintLine(1, line2);
}

static void LCD_ShowTemp(int16_t temp)
{
    char line1[17];
    char line2[17];
    char temp_str[10];
    
    sprintf(temp_str, "%d.%d C", temp / 10, temp % 10);
    sprintf(line1, "Temp: %s", temp_str);
    
    if (temp >= TEMP_CRITICAL * 10)
        sprintf(line2, "!!!CRITICAL!!!");
    else if (temp >= TEMP_WARNING * 10)
        sprintf(line2, "WARNING! >%dC", TEMP_WARNING);
    else
        sprintf(line2, "Normal < %dC", TEMP_WARNING);
    
    LCD_ShowTwoLines(line1, line2);
}

static void LCD_InitGPIO(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
                    GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOB->CRL |=  (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE1_1);

    GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12 |
                    GPIO_CRH_MODE13 | GPIO_CRH_CNF13 |
                    GPIO_CRH_MODE14 | GPIO_CRH_CNF14 |
                    GPIO_CRH_MODE15 | GPIO_CRH_CNF15);
    GPIOB->CRH |=  (GPIO_CRH_MODE12_1 |
                    GPIO_CRH_MODE13_1 |
                    GPIO_CRH_MODE14_1 |
                    GPIO_CRH_MODE15_1);

    LCD_RS_LOW();
    LCD_E_LOW();
    LCD_D4_LOW();
    LCD_D5_LOW();
    LCD_D6_LOW();
    LCD_D7_LOW();
}

static void LCD_Init(void)
{
    LCD_InitGPIO();
    Delay(300000);
    LCD_RS_LOW();
    LCD_Send4Bits(0x03);
    Delay(200000);
    LCD_Send4Bits(0x03);
    Delay(100000);
    LCD_Send4Bits(0x03);
    Delay(100000);
    LCD_Send4Bits(0x02);
    Delay(100000);
    LCD_SendCommand(0x28);
    LCD_SendCommand(0x0C);
    LCD_SendCommand(0x06);
    LCD_Clear();
}

/* ===================== Utils ===================== */
static void ToUpperStr(char *str)
{
    while (*str)
    {
        *str = (char)toupper((unsigned char)*str);
        str++;
    }
}

static void TrimSpaces(char *str)
{
    char *src = str;
    char *dst = str;
    uint8_t prev_space = 1;

    while (*src)
    {
        if (*src == ' ' || *src == '\t')
        {
            if (!prev_space)
            {
                *dst++ = ' ';
                prev_space = 1;
            }
        }
        else
        {
            *dst++ = *src;
            prev_space = 0;
        }
        src++;
    }

    if (dst > str && *(dst - 1) == ' ')
        dst--;

    *dst = '\0';
}

/* ===================== Command parser ===================== */
static int ParseCommand(char *line, Operation_t *op, int *a, int *b)
{
    char *token;
    char *endptr;
    char *tokens[4];
    int count = 0;
    char temp[RX_BUFFER_SIZE];
    int i;
    
    strncpy(temp, line, RX_BUFFER_SIZE - 1);
    temp[RX_BUFFER_SIZE - 1] = '\0';
    
    for(i = 0; temp[i] != '\0'; i++) {
        if(temp[i] == '\r' || temp[i] == '\n' || temp[i] == '\t') {
            temp[i] = ' ';
        }
    }
    
    TrimSpaces(temp);
    ToUpperStr(temp);
    
    token = strtok(temp, " ");
    while (token != NULL && count < 4)
    {
        tokens[count++] = token;
        token = strtok(NULL, " ");
    }
    
    if (count == 1)
    {
        if (strcmp(tokens[0], "HELP") == 0) return 2;
        if (strcmp(tokens[0], "HISTORY") == 0) return 3;
        if (strcmp(tokens[0], "CLEAR") == 0) return 4;
        if (strcmp(tokens[0], "TEMP") == 0) return 5;
        return -1;
    }
    
    if (count != 3)
        return -1;
    
    if (tokens[0][0] == 'A')
        *op = OP_ADD;
    else if (tokens[0][0] == 'S')
        *op = OP_SUB;
    else if (tokens[0][0] == 'M')
        *op = OP_MUL;
    else if (tokens[0][0] == 'D')
        *op = OP_DIV;
    else
        return -1;
    
    *a = (int)strtol(tokens[1], &endptr, 10);
    if (*endptr != '\0')
        return -1;
    
    *b = (int)strtol(tokens[2], &endptr, 10);
    if (*endptr != '\0')
        return -1;
    
    return 1;
}

/* ===================== History ===================== */
static void History_Add(const char *entry)
{
    uint8_t index;

    if (history_count < HISTORY_SIZE)
    {
        index = (history_start + history_count) % HISTORY_SIZE;
        history_count++;
    }
    else
    {
        index = history_start;
        history_start = (history_start + 1) % HISTORY_SIZE;
    }

    strncpy(history[index], entry, HISTORY_STR_LEN - 1);
    history[index][HISTORY_STR_LEN - 1] = '\0';
}

static void History_Print(void)
{
    char out[96];
    uint8_t i, index;

    if (history_count == 0)
    {
        USART2_SendString("History is empty\r\n");
        LCD_ShowTwoLines("History", "empty");
        return;
    }

    USART2_SendString("History:\r\n");
    for (i = 0; i < history_count; i++)
    {
        index = (history_start + i) % HISTORY_SIZE;
        sprintf(out, "%d) %s\r\n", i + 1, history[index]);
        USART2_SendString(out);
    }

    LCD_ShowTwoLines("History", "see UART");
}

static void History_Clear(void)
{
    uint8_t i;
    for (i = 0; i < HISTORY_SIZE; i++)
        history[i][0] = '\0';

    history_count = 0;
    history_start = 0;
}

static void PrintHelp(void)
{
    USART2_SendString("Available commands:\r\n");
    USART2_SendString("ADD a b     - addition\r\n");
    USART2_SendString("SUB a b     - subtraction\r\n");
    USART2_SendString("MUL a b     - multiplication\r\n");
    USART2_SendString("DIV a b     - division\r\n");
    USART2_SendString("TEMP        - show temperature\r\n");
    USART2_SendString("HISTORY     - show history\r\n");
    USART2_SendString("CLEAR       - clear history\r\n");
    USART2_SendString("HELP        - show help\r\n");

    LCD_ShowTwoLines("HELP", "see UART");
}

/* ===================== Command processor ===================== */
static void ProcessCommand(char *line)
{
    Operation_t op = OP_NONE;
    int a = 0, b = 0, res = 0;
    int status;
    char result[96];
    char lcd1[17];
    char lcd2[17];
    char work[RX_BUFFER_SIZE];

    strncpy(work, line, RX_BUFFER_SIZE - 1);
    work[RX_BUFFER_SIZE - 1] = '\0';

    status = ParseCommand(work, &op, &a, &b);

    if (status == 2)  /* HELP */
    {
        PrintHelp();
        LED_Off();
        return;
    }

    if (status == 3)  /* HISTORY */
    {
        History_Print();
        LED_Off();
        return;
    }

    if (status == 4)  /* CLEAR */
    {
        History_Clear();
        USART2_SendString("History cleared\r\n");
        LCD_ShowTwoLines("History", "cleared");
        LED_Off();
        return;
    }
    
    if (status == 5)  /* TEMP */
    {
        char temp_msg[32];
        current_temperature = GetTemperature();
        sprintf(temp_msg, "Temperature: %d.%d C", current_temperature / 10, abs(current_temperature % 10));
        USART2_SendString(temp_msg);
        USART2_SendString("\r\n");
        LCD_ShowTemp(current_temperature);
        LED_TempUpdate(current_temperature);
        return;
    }

    if (status != 1)
    {
        USART2_SendString("Invalid command. Type HELP\r\n");
        LCD_ShowTwoLines("Invalid command", "Type HELP");
        LED_Error();
        return;
    }

    switch (op)
    {
        case OP_ADD:
            res = a + b;
            sprintf(result, "%d + %d = %d", a, b, res);
            sprintf(lcd1, "ADD %d %d", a, b);
            sprintf(lcd2, "RES=%d", res);
            break;

        case OP_SUB:
            res = a - b;
            sprintf(result, "%d - %d = %d", a, b, res);
            sprintf(lcd1, "SUB %d %d", a, b);
            sprintf(lcd2, "RES=%d", res);
            break;

        case OP_MUL:
            res = a * b;
            sprintf(result, "%d * %d = %d", a, b, res);
            sprintf(lcd1, "MUL %d %d", a, b);
            sprintf(lcd2, "RES=%d", res);
            break;

        case OP_DIV:
            if (b == 0)
            {
                USART2_SendString("Error: division by zero\r\n");
                LCD_ShowTwoLines("DIV ERROR", "b=0");
                LED_Error();
                return;
            }
            res = a / b;
            sprintf(result, "%d / %d = %d", a, b, res);
            sprintf(lcd1, "DIV %d %d", a, b);
            sprintf(lcd2, "RES=%d", res);
            break;

        default:
            USART2_SendString("Invalid command. Type HELP\r\n");
            LCD_ShowTwoLines("Invalid command", "Type HELP");
            LED_Error();
            return;
    }

    USART2_SendString(result);
    USART2_SendString("\r\n");
    History_Add(result);
    LCD_ShowTwoLines(lcd1, lcd2);
    LED_Success();
}

/* ===================== main ===================== */
int main(void)
{
    char local_buffer[RX_BUFFER_SIZE];
    uint32_t last_temp_read = 0;
    
    SystemInit();
    USART2_Init();
    LCD_Init();
    LED_Init();
    ADC_Init();

    USART2_SendString("\r\nSTM32 Calculator + Temperature Monitor\r\n");
    USART2_SendString("Type HELP for commands\r\n");
    LCD_ShowTwoLines("STM32 Ready", "Temp + Calc");

    while (1)
    {
        /* ????????? ??????????? ?????? 500 ?? */
        if (last_temp_read++ > 50000)
        {
            last_temp_read = 0;
            current_temperature = GetTemperature();
            LCD_ShowTemp(current_temperature);
            LED_TempUpdate(current_temperature);
        }
        
        if (line_ready)
        {
            strncpy(local_buffer, (char*)rx_buffer, RX_BUFFER_SIZE - 1);
            local_buffer[RX_BUFFER_SIZE - 1] = '\0';
            line_ready = 0;
            ProcessCommand(local_buffer);
        }
    }
}