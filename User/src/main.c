#include "stm32f4xx_conf.h"
#include "user_config.h"
#include "stm32f4xx_gpio.h"
#include "main.h"
#include "coocox.h"

OS_STK led13_stk[TASK_STK_SIZE];
OS_STK button_stk[TASK_STK_SIZE];

extern P_OSTCB DlyList;
OS_EventID mb;
OS_EventID ack;   

void bling_led13(void *par)
{   
    int toggle = 0;
    StatusType perr;
    mb = CoCreateMbox(EVENT_SORT_TYPE_FIFO);
    while (1) {
        CoPendMail(mb,0,&perr);
        toggle = !toggle;
        GPIO_WriteBit(GPIOG,GPIO_Pin_13, toggle);
        debugs("%d\r\n",toggle);
        CoPostMail(ack,&toggle);
    }
    CoExitTask();
}

void button(void *par)
{   
    StatusType perr;
    ack = CoCreateMbox(EVENT_SORT_TYPE_FIFO);
    while (1) {
        int key = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
        int val = 1;
        if (key == 1) {
            CoPostMail(mb, &val);
            CoPendMail(ack,0,&perr);
        }
    }
    CoExitTask();
}

int main(void)
{
    UART_Init();
    debugs("\n\n\n\r\n");

    //debugs("[System] Initial system tick ...");
    //Delay_Init();
	//debugs(GRN_BOLD"[OK]"ARESET"\r\n");

	debugs("[System] Initial LEDs ...");
    LED_Init();
    Button_Init();
	debugs(GRN_BOLD"[OK]"ARESET"\r\n");

	debugs("[System] Initial CoOS ...");
    CoInitOS();
	debugs(GRN_BOLD"[OK]"ARESET"\r\n");

	debugs("[System] Create Task ...");
	CoCreateTask(bling_led13, (void*)0, 20, &led13_stk[TASK_STK_SIZE-1], TASK_STK_SIZE);
    CoCreateTask(button, (void*)0, 20, &button_stk[TASK_STK_SIZE-1], TASK_STK_SIZE);
    debugs(GRN_BOLD"[OK]"ARESET"\r\n");

	debugs("[System] Start CoOS ...\r\n");
    CoStartOS();

	debugs(RED_BOLD"[System] System initial failed!\r\n"ARESET);

    return 0;
}

