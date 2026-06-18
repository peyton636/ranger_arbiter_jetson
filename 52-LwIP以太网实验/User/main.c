#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "tftlcd.h" 
#include "key.h"
#include "rtc.h"
#include "beep.h"
#include "adc.h"
#include "adc_temp.h"
#include "time.h"
#include "sram.h"
#include "malloc.h"
#include "lan8720.h"
#include "lwip/netif.h"
#include "lwip_comm.h"
#include "lwipopts.h"
#include "tcp_client_demo.h"
#include "tcp_server_demo.h"
#include "udp_demo.h"
#include "httpd.h"



extern void Adc_Temperate_Init(void);	//声明内部温度传感器初始化函数
//加载UI
//mode:
//bit0:0,不加载;1,加载前半部分UI
//bit1:0,不加载;1,加载后半部分UI
void lwip_test_ui(u8 mode)
{
	u8 speed;
	u8 buf[30]; 
	FRONT_COLOR=RED;
	if(mode&1<<0)
	{
		LCD_Fill(30,30,tftlcd_data.width,110,WHITE);	//清除显示
		LCD_ShowString(30,30,200,16,16,"PRECHIN STM32F4");
		LCD_ShowString(30,50,200,16,16,"Ethernet lwIP Test");
		LCD_ShowString(30,70,200,16,16,"www.prechin.net"); 	
	}
	if(mode&1<<1)
	{
		LCD_Fill(30,110,tftlcd_data.width,tftlcd_data.height,WHITE);	//清除显示
		LCD_ShowString(30,110,200,16,16,"lwIP Init Successed");
		if(lwipdev.dhcpstatus==2)sprintf((char*)buf,"DHCP IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//打印动态IP地址
		else sprintf((char*)buf,"Static IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//打印静态IP地址
		LCD_ShowString(30,130,210,16,16,buf); 
		speed=LAN8720_Get_Speed();//得到网速
		if(speed&1<<1)LCD_ShowString(30,150,200,16,16,"Ethernet Speed:100M");
		else LCD_ShowString(30,150,200,16,16,"Ethernet Speed:10M");
		LCD_ShowString(30,170,200,16,16,"KEY0:TCP Server Test");
		LCD_ShowString(30,190,200,16,16,"KEY1:TCP Client Test");
		LCD_ShowString(30,210,200,16,16,"KEY2:UDP Test");
	}
}


int main()
{	
	u8 t;
	u8 key;
	
	SysTick_Init(168);       	//延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	USART1_Init(115200);   	//串口波特率设置
	LED_Init();  			//LED初始化
	KEY_Init();  			//按键初始化
	TFTLCD_Init(); 			//LCD初始化
	FSMC_SRAM_Init();		//初始化外部SRAM  
	BEEP_Init();			//蜂鸣器初始化
	RTC_Config();  			//RTC初始化
	ADCx_Init();  			//ADC初始化 
	ADC_Temp_Init(); 		//内部温度传感器初始化
	TIM3_Init(999,839); 	//100khz的频率,计数1000为10ms
	
	my_mem_init(SRAMIN);		//初始化内部内存池
	my_mem_init(SRAMEX);		//初始化外部内存池
	my_mem_init(SRAMCCM);	//初始化CCM内存池
	
	FRONT_COLOR = RED; 		//红色字体
	printf("Init...\r\n");
	lwip_test_ui(1);		//加载前半部分UI
	
	//先初始化lwIP(包括LAN8720初始化),此时必须插上网线,否则初始化会失败!! 
	LCD_ShowString(30,110,200,16,16,"lwIP Initing...");
	while(lwip_comm_init()!=0)
	{
		LCD_ShowString(30,110,200,16,16,"lwIP Init failed!");
		delay_ms(1200);
		LCD_Fill(30,110,230,110+16,WHITE);//清除显示
		LCD_ShowString(30,110,200,16,16,"Retrying...");  
		printf("Retrying...\r\n");
	}
	LCD_ShowString(30,110,200,16,16,"lwIP Init Successed");
	printf("lwIP Init Successed\r\n");
#if LWIP_DHCP
	/* 仅 DHCP 模式：等待获取成功(2)或超时(0xFF) */
	LCD_ShowString(30,130,200,16,16,"DHCP IP configing...");
	while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))
	{
		lwip_periodic_handle();
		printf("DHCP IP configing...\r\n");
	}
	printf("DHCP IP configing OK\r\n");
#else
	/* 静态 IP：lwip_comm_init 已配好，无需等待 DHCP */
	printf("Static IP %d.%d.%d.%d, skip DHCP\r\n",
		lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
#endif
	lwip_test_ui(2);//加载后半部分UI 
	httpd_init();	//HTTP初始化(默认开启websever)
	while(1)
	{
		key=KEY_Scan(0);
		switch(key)
		{
			case KEY0_PRESS://TCP Server模式
				tcp_server_test();
				lwip_test_ui(3);//重新加载UI
				break;
			case KEY1_PRESS://TCP Client模式
				tcp_client_test();
				lwip_test_ui(3);//重新加载UI
				break; 
			case KEY2_PRESS://UDP模式
				udp_demo_test();
				lwip_test_ui(3);//重新加载UI
				break; 
		}
		lwip_periodic_handle();
		delay_ms(2);
		t++;
		if(t==100)LCD_ShowString(30,230,200,16,16,"Please choose a mode!");
		if(t==200)
		{ 
			t=0;
			LCD_Fill(30,230,230,230+16,WHITE);//清除显示
			LED1=!LED1;
		} 
	}
}


