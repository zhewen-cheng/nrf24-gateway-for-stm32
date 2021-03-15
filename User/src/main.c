
#include "stm32f4xx_conf.h"
#include "user_config.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4x7_eth_bsp.h"
#include "stm32f4x7_eth.h"
#include "stm32f4xx.h"
#include "init.h"
#include "main.h"
#include "coocox.h"
#include <stdlib.h>
#include "netconf.h"


//#include <check.h>
__IO uint32_t LocalTime = 0;

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
void lwip_test()
{
    while(1){
    if(ETH_CheckFrameReceived())
    {
      LwIP_Pkt_Handle();
    }
    LwIP_Periodic_Handle(LocalTime);
  }
  CoExitTask();
}


int main(void)
{   
   UART_Init();
  debugs("\n\n\n\r\n");  
    LED_Init();
  debugs("[System] Initial LEDs ...\r\n");
    Button_Init();
  debugs("[System] Initial Ethernet ...\r\n");
    ETH_BSP_Config();
  debugs("[System] Initial LWip ...\r\n");
    LwIP_Init();
  debugs("local IP and port:%d.%d.%d.%d\r\n",IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
  debugs("dest IP and port:%d.%d.%d.%d:%d\r\n",DEST_ADDR0,DEST_ADDR1,DEST_ADDR2,DEST_ADDR3,DEST_PORT);


  //debugs("[System] Initial system tick ...");
  //Delay_Init();
	//debugs(GRN_BOLD"[OK]"ARESET"\r\n");
	debugs(GRN_BOLD"[OK]"ARESET"\r\n");

	debugs("[System] Initial CoOS ...\r\n");
    CoInitOS();
	debugs(GRN_BOLD"[OK]"ARESET"\r\n");

	debugs("[System] Create Task ...\r\n");
	//CoCreateTask(bling_led13, (void*)0, 20, &led13_stk[TASK_STK_SIZE-1], TASK_STK_SIZE);
  CoCreateTask(lwip_test, (void*)0, 20, &button_stk[TASK_STK_SIZE-1], TASK_STK_SIZE);
  debugs(GRN_BOLD"[OK]"ARESET"\r\n");

	debugs("[System] Start CoOS ...\r\n");
    CoStartOS();

	debugs(RED_BOLD"[System] System initial failed!\r\n"ARESET);

    return 0;
}

