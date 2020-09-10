/**
 *******************************************************************************
 * @file       user_config.c
 * @version    V0.1.1
 * @date       2020.01.19
 * @brief      Some initialization functions.
 *******************************************************************************
 */ 

#include <stdio.h>
#include <stdarg.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "nrf24l01.h"
#include "user_config.h"



/**
 *******************************************************************************
 * @brief      Initial USART 1 for printing output.
 * @param[in]  None
 * @param[out] None
 * @retval     None
 *******************************************************************************
 */
void UART_Init(void){

	USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate            = 9600;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
			
	USART_Cmd(USART1, ENABLE);
}		


/**
 *******************************************************************************
 * @brief      Initial LED pins.
 * @param[in]  None
 * @param[out] None
 * @retval     None
 *******************************************************************************
 */
void LED_Init(uint8_t led_pin)
{
    GPIO_InitTypeDef LED_InitStruct;

    /* Setup LED GPIO */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    GPIO_StructInit(&LED_InitStruct);
    LED_InitStruct.GPIO_Pin   = led_pin ;
    LED_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    LED_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOG, &LED_InitStruct);
}

void Delay_Init(uint8_t frec)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);    /* get system clock */
    /* while loop takes 4 cycles */
    /* Enable SysTick for Delay function */
    if (SysTick_Config(SystemCoreClock / frec)) //1ms
    { 
        /* Capture error */ 
        while (1);
    }
}

/**
 *******************************************************************************
 * @brief      Print character via USART1.
 * @param[in]  ch       Character to print.
 * @param[out] None
 * @retval     None
 *******************************************************************************
 */
void stm_putchar(const char ch)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART1->DR = (uint8_t) ch;
}

/**
 *******************************************************************************
 * @brief      Print string via USART1.
 * @param[in]  *str     String to print.
 * @param[out] None
 * @retval     None
 *******************************************************************************
 */
void stm_putstr(const char *str)
{
    while (*str) {
        if (*str == '\n') {
            stm_putchar('\n');
            str++;
        }
        else if (*str == '\r') {
            stm_putchar('\r');
            str++;
        }
        else {
            stm_putchar(*str++);
        }
    }
}

int debugs(const char *fmt,...)
{
	int32_t printed;
	char printf_buf[300];
	va_list args;

	va_start(args, fmt);
	printed = vsprintf(printf_buf, fmt, args);
	va_end(args);

	stm_putstr(printf_buf);

	return printed;
}

static void print_status(void)
{
    uint8_t status = NRF24L01_ReadRegister(NRF24L01_REG_STATUS);

    debugs("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n",
            status,
			(status & 1<<(6)) ? 1 : 0,      //NRF24L01_RX_DR
			(status & 1<<(5)) ? 1 : 0,      //NRF24L01_TX_DS
			(status & 1<<(4)) ? 1 : 0,      //NRF24L01_MAX_RT
			(status >> 1) & 0x07,           //NRF24L01_RX_P_NO
			(status & 1<<(0)) ? 1 : 0);     //NRF24L01_TX_FULL
}

static void print_address_register(const char *name, uint8_t reg, uint8_t counts)
{
	debugs("%s\t =", name);
	while (counts--) {
		debugs(" 0x");
		debugs("%02x", NRF24L01_ReadRegister(reg++));
	}
	debugs("\r\n");
}

static void print_byte_register(const char *name, uint8_t reg, uint8_t counts)
{
    uint8_t buf;
	debugs("%s\t = ", name);
	while (counts--) {
		debugs("0x%02x ", NRF24L01_ReadRegisterMulti(reg++,&buf,1));
	}
	debugs("\r\n");
}

uint8_t NRF24L01_get_pa_power(void)
{
	return (NRF24L01_ReadRegister(NRF24L01_REG_RF_SETUP) & (1<<1 | 1<<2)) >> 1;
}

NRF24L01_DataRate_t NRF24L01_get_datarate(void) 
{
	NRF24L01_DataRate_t result;
	uint8_t rf_setup = NRF24L01_ReadRegister(NRF24L01_REG_RF_SETUP) & (1<<(NRF24L01_RF_DR_LOW) | 1<<(NRF24L01_RF_DR_HIGH));

    if (rf_setup == 1<<(NRF24L01_RF_DR_LOW)) {
        // '10' = 250KBPS
        result = NRF24L01_DataRate_250k;
    } else if (rf_setup == 1<<(NRF24L01_RF_DR_HIGH)) {
        // '01' = 2MBPS
        result = NRF24L01_DataRate_2M;
    } else {
        // '00' = 1MBPS
        result = NRF24L01_DataRate_1M;
    }

    return result;
}

NRF24L01_CRCLength_t NRF24L01_get_crc_length(void)
{
    NRF24L01_CRCLength_t result = NRF24L01_CRC_DISABLED;

    uint8_t config = NRF24L01_ReadRegister(NRF24L01_REG_CONFIG) & (1<<(NRF24L01_CRCO) | 1<<(NRF24L01_EN_CRC));
    uint8_t AA = NRF24L01_ReadRegister(NRF24L01_REG_EN_AA);

    if (config & 1<<(NRF24L01_EN_CRC) || AA) {
        if (config & 1<<(NRF24L01_CRCO)) {
            result = NRF24L01_CRC_16;
        } else {
            result = NRF24L01_CRC_8;
        }
    }

    return result;
}


void print_details(void)
{
	const char data_rate[][8] = {"1Mbps", "2Mbps", "250Kbps"};
	const char crc_len[][8] = {"OFF", "8BIT", "16BIT"};
	const char pa_power[][8] = {"PA_MIN", "PA_LOW", "PA_HIGH", "PA_MAX"};

	debugs("\r\n================ NRF Configuration ===============\r\n");
	print_status();

	print_address_register("RX_ADDR_P0-1", NRF24L01_REG_RX_ADDR_P0, 2);
	print_byte_register("RX_ADDR_P2-5", NRF24L01_REG_RX_ADDR_P2, 4);
	print_address_register("TX_ADDR\t", NRF24L01_REG_TX_ADDR, 1);

	print_byte_register("RX_PW_P0-P5", NRF24L01_REG_RX_PW_P0, 6);
	print_byte_register("EN_AA\t\t", NRF24L01_REG_EN_AA, 1);
	print_byte_register("EN_RXADDR\t", NRF24L01_REG_EN_RXADDR, 1);
	print_byte_register("RF_CH\t\t", NRF24L01_REG_RF_CH, 1);
	print_byte_register("RF_SETUP\t", NRF24L01_REG_RF_SETUP, 1);
	print_byte_register("CONFIG\t", NRF24L01_REG_CONFIG, 1);
	print_byte_register("DYNPD/FEATURE", NRF24L01_REG_DYNPD, 2);

    debugs("ABC\t\t = 0x%02x\r\n",NRF24L01_ReadRegister(0x05));
	debugs("Data Rate\t\t = %s\r\n", data_rate[NRF24L01_get_datarate()]);
	debugs("Model\t\t\t = nRF24L01\r\n");
	debugs("CRC Length\t\t = %s\r\n", crc_len[NRF24L01_get_crc_length()]);
	debugs("PA Power\t\t = %s\r\n", pa_power[NRF24L01_get_pa_power()]);
}