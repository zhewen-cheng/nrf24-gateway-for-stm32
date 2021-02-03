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

#define __DEBUG

/* ANSI define */
#define RED_BOLD                "\x1b[;31;1m"
#define YEL_BOLD                "\x1b[;33;1m"
#define BLU_BOLD                "\x1b[;34;1m"
#define MAN_BOLD                "\x1b[;35;1m"
#define GRN_BOLD                "\x1b[;32;1m"
#define WHITE_BG                "\x1b[;47;1m"
#define CYAN_BOLD_ITALIC        "\x1b[;36;1;3m"
#define ARESET                  "\x1b[0;m"


/* GPIO LED define */
#define STM_LED_TOGGLE(pin)		GPIO_ToggleBits(GPIOG, (pin))
#define STM_LED_OFF(pin)		GPIO_WriteBit(GPIOG, (pin), Bit_RESET)
#define STM_LED_ON(pin)			GPIO_WriteBit(GPIOG, (pin), Bit_SET)

#define LED_GREEN               GPIO_Pin_13
#define LED_RED                 GPIO_Pin_14

/* hardware initial function */
void UART_Init(void);
void ETH_GpioInit(void);
void LED_Init(void);
void Button_Init(void);
void Delay_Init(void);

/* printf */
int debugs(const char *str,...);

#endif /* __USER_CONFIG_H__ */
