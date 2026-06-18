#ifndef _beep_H
#define _beep_H

#include "system.h"

/*  蜂鸣器时钟端口、引脚定义 */
#define BEEP_PORT 			GPIOF   
#define BEEP_PIN 			GPIO_Pin_8
#define BEEP_PORT_RCC		RCC_AHB1Periph_GPIOF

#define BEEP PFout(8)

void BEEP_Init(void);


#endif
