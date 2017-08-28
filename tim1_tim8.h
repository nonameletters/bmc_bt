#ifndef TIM1_TIM8_H
#define TIM1_TIM8_H

#include "ch.h"

#if defined(BOARD_BE_BT_NUCLEO_207ZG)
    #include "stm32f207xx.h"
#else
    #include "stm32f205xx.h"
#endif

void enableDMATim1(void);

void configTim8ToGeneratePWM(void);

void configTim1WithPE9Input(void);

void configTim1ECM1_ETR_Trigger(void);
#endif
