/**
 *******************************************************************************
 * @file       user_config.h
 * @version    V0.1.1
 * @date       2020.01.19
 * @brief      User define file.
 *******************************************************************************
 */ 
#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

/* ANSI define */
#define RED_BOLD                "\x1b[;31;1m"
#define YEL_BOLD                "\x1b[;33;1m"
#define BLU_BOLD                "\x1b[;34;1m"
#define MAN_BOLD                "\x1b[;35;1m"
#define GRN_BOLD                "\x1b[;32;1m"
#define WHITE_BG                "\x1b[;47;1m"
#define CYAN_BOLD_ITALIC        "\x1b[;36;1;3m"
#define ARESET                  "\x1b[0;m"

#define STM_LED_OFF(pin)		GPIO_WriteBit(GPIOG, (pin), Bit_RESET);
#define STM_LED_ON(pin)			GPIO_WriteBit(GPIOG, (pin), Bit_SET);

#define LED_GREEN               GPIO_Pin_13
#define LED_RED                 GPIO_Pin_14
#define LED_BOTH                GPIO_Pin_13 | GPIO_Pin_14


/* hardware initial function */
void UART_Init(void);
void LED_Init(uint8_t led_pin);
void Delay_Init(uint8_t frec);

/* printf */
int debugs(const char *str,...);
void print_details(void);

/*hal extend function*/
uint8_t NRF24L01_get_pa_power(void);
NRF24L01_DataRate_t NRF24L01_get_datarate(void);
NRF24L01_CRCLength_t NRF24L01_get_crc_length(void);



#endif /* __USER_CONFIG_H__ */
