/**
 *******************************************************************************
 * @file       user_config.c
 * @version    V0.1.1
 * @date       2020.01.19
 * @brief      Some initialization functions.
 *******************************************************************************
 */ 

#include <stdarg.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "nrf24l01.h"
#include "user_config.h"
#include "stm32f429_eth.h"


extern ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
extern ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */
extern uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; /* Ethernet Receive Buffer */
extern uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE];

/**
 *******************************************************************************
 * @brief      Initial USART 1 for printing output.
 * @param[in]  None
 * @param[out] None
 * @retval     None
 *******************************************************************************
 */
void UART_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
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
void LED_Init(void)
{
    GPIO_InitTypeDef LED_InitStruct;

    /* Setup LED GPIO */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    GPIO_StructInit(&LED_InitStruct);
    LED_InitStruct.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14;
    LED_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    LED_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOG, &LED_InitStruct);
}

void Button_Init(void)
{
    GPIO_InitTypeDef Button_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_StructInit(&Button_InitStruct);
    Button_InitStruct.GPIO_Pin  = GPIO_Pin_0;
    Button_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    //Button_InitStruct.GPIO_OType = GPIO_OType_PP;
    Button_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    //Button_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &Button_InitStruct);
}

void Delay_Init(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);    /* get system clock */
    /* while loop takes 4 cycles */
    /* Enable SysTick for Delay function */
    if (SysTick_Config(SystemCoreClock / 1000)) //1ms
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
void _putchar(const char ch)
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
void _putstr(const char *str)
{
    while (*str) {
        if (*str == '\n') {
            _putchar('\n');
            str++;
        } else if (*str == '\r') {
            _putchar('\r');
            str++;
        } else {
            _putchar(*str++);
        }
    }
}

uint64_t _pow(int32_t base, int32_t exp)
{
    uint64_t sum = 1;

    /* multiplicative base */
    while (exp--) {
        sum *= base;
    }

    return sum;
}

uint64_t _strlen(const char *str)
{
    const char *s;

    /* goto the end of the string */
    for (s = str; *s != '\0'; ++s) {
        /* passing */
    }
    
    /* return memory address's differece */
    return s - str;
}

/**
 *******************************************************************************
 * @brief      Print format string via USART1.
 * @param[in]  fmt       Format to print.
 * @param[out] None
 * @retval     printed   Printed string length.
 *******************************************************************************
 */
int debugs(const char *fmt,...)
{
#ifdef __DEBUG
    va_list ap;
    int val,r_val,space=0;
	char count, ch;
	char *s = 0;
    int res = 0;

    va_start(ap,fmt);
    while ('\0' != *fmt) { 
        switch (*fmt) {
        case '%':
            fmt++;
            while (*fmt >= '0' && *fmt <= '9') {
                space *= 10;
                space += *fmt - '0';
                fmt++;
            }

            switch (*fmt) { 
            /* handle integer var */
            case 'd':
                val = va_arg(ap, int); 
 			    
                /* if val is negative or zero */
                if (val < 0) {
                    _putchar('-');
                    val = 0 - val;
                } else if (val == 0) {
                    _putchar('0');
                }
                        
                r_val = val; 
                count = 0; 
				while (r_val) {
                    count++;
                    r_val /= 10;
                }
				res += count;
                r_val = val; 
                while (count) { 
                    ch = r_val / _pow(10,count - 1);
					r_val %= _pow(10, count - 1);
					_putchar(ch + '0');
					count--;
				}
                break;

            /* handle integer var with hex output */
            case 'x':
                val = va_arg(ap, int);

                /* if val is negative or zero */
                if (val<0) {
                    _putchar('-');
                    val = 0 - val;
                } else if (val == 0) {
                    _putchar('0');
                }
                        
                r_val = val; 
                count = 0;
				while (r_val) {
                    count++;
                    r_val /= 16; 
                }
                res += count;
                r_val = val; 
                while(count) { 
                    ch = r_val / _pow(16, count - 1);
					r_val %= _pow(16, count - 1);
					if (ch <= 9) {
                        _putchar(ch + '0');
                    } else {
						_putchar(ch - 10 + 'a');
                    }
					count--;
				}
				break;

            /* handle string var */
            case 's':
				s = va_arg(ap, char *);
                
                if (space) {
                    int len = _strlen(s);
                    while (len < space) {
                        _putchar(' ');
                        space --;
                    }
                }

				_putstr(s);
                res += _strlen(s);
				break;
					
            /* handle character var */
            case 'c':
                _putchar( (char)va_arg(ap, int ));
				res += 1;
                break;

            default:
				break;
		    }
			break;
            
            /* handle escape character: newline */
            case '\n':
				_putchar('\n');
				res += 1;
				break;
			
            /* handle escape character: return */
			case '\r':
				_putchar('\r');
				res += 1;
				break;
			
            /* just output character */
			default:
				_putchar(*fmt);
				res += 1;
                break;
		}
		fmt++;
     }
    va_end(ap);

	return res;
#else
    return -1;
#endif
}


void ETH_NVIC_Config(void) {
    NVIC_InitTypeDef   NVIC_InitStructure;
 
    /* Enable the Ethernet global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void DP83848Init(uint8_t* HWADDR){
    int i;
    /* Configure ethernet (GPIOs, clocks, MAC, DMA) */
    ETH_BSP_Config();
 
    /* initialize MAC address in ethernet MAC */
    ETH_MACAddressConfig(ETH_MAC_Address0, HWADDR);
    /* Initialize Tx Descriptors list: Chain Mode */
    ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
    /* Initialize Rx Descriptors list: Chain Mode  */
    ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
 
    /* Enable the TCP, UDP and ICMP checksum insertion for the Tx frames */
    for(i = 0; i < ETH_TXBUFNB; i++) {
        ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
    }
    ETH_Start();
}
