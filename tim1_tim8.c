#include "tim1_tim8.h"

uint16_t dmaTim1_buff[2] = {0x00, 0x00};

// ---------- ---------- ---------- ---------- ---------- ----------
void enableDMATim1(void)
{
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);

	// DMA2 stream1 channel6 for timer1 ch1
	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2); // channel6  is chosen

	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_PL); // Priority very height
	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_MSIZE_0); // Memory size 16 bit
	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_PSIZE_0); // Peripheral size 16 bit

	SET_BIT(DMA2_Stream1->CR,   DMA_SxCR_MINC); // Incrementing memory address
	CLEAR_BIT(DMA2_Stream1->CR, DMA_SxCR_PINC); // Do not incrementing peripheral address
	SET_BIT(DMA2_Stream1->CR,   DMA_SxCR_CIRC); // Circular mode
	CLEAR_BIT(DMA2_Stream1->CR, DMA_SxCR_DIR);  // direction from Peripheral to memory
	SET_BIT(DMA2_Stream1->CR,   DMA_SxCR_TCIE); // Enable transfer complete interrupt


	DMA2_Stream1->NDTR = 2; // Buffer size (or Number of data to be transfered)
	DMA2_Stream1->M0AR = (uint32_t) dmaTim1_buff;
	DMA2_Stream1->M1AR = (uint32_t) dmaTim1_buff;
	DMA2_Stream1->PAR  = (uint32_t) &(TIM1->CCR1);// (0x40010000 + 0x34);

	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_EN); // Enable stream2
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void configTim8ToGeneratePWM(void)
{
	// Starting TIM8
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);
	CLEAR_BIT(TIM8->CR1, TIM_CR1_DIR); // Up counter
	CLEAR_BIT(TIM8->CR1, TIM_CR1_OPM);
	SET_BIT(TIM8->DIER, TIM_DIER_UIE); // Enable Update Interrupt event
	SET_BIT(TIM8->DIER, TIM_DIER_CC1IE);

	SET_BIT(TIM8->CR1, TIM_CR1_CKD_1);
	TIM8->CNT = 500;
	TIM8->ARR = 500;

	TIM8->PSC = 60000;

	//TIM8->CNT = 500;
	//TIM8->ARR = 65000;

	//TIM8->PSC = 0;

    CLEAR_BIT(TIM8->BDTR, TIM_BDTR_DTG);
    // SET_BIT(TIM8->BDTR, TIM_BDTR_DTG_7 | TIM_BDTR_DTG_6 | TIM_BDTR_DTG_5 | TIM_BDTR_DTG_4 | TIM_BDTR_DTG_3 | TIM_BDTR_DTG_1);
    //SET_BIT(TIM8->BDTR, TIM_BDTR_DTG_3 | TIM_BDTR_DTG_2 | TIM_BDTR_DTG_1 | TIM_BDTR_DTG_0);
    //SET_BIT(TIM8->BDTR, TIM_BDTR_DTG_5 | TIM_BDTR_DTG_4 | TIM_BDTR_DTG_1);
    //TIM8->BDTR = 255;

//    This lines enables channe 3 for PWM
//    SET_BIT(TIM8->CCER, TIM_CCER_CC3E); // Enable chanel 3
    CLEAR_BIT(TIM8->CCER, TIM_CCER_CC3P);
//    SET_BIT(TIM8->CCER, TIM_CCER_CC3NE);
//    SET_BIT(TIM8->CCER, TIM_CCER_CC3NP);

    CLEAR_BIT(TIM8->CCMR1, TIM_CCMR1_CC1S);
    SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC1PE);
    //SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // PWM mode 1
    SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1); // PWM mode 1
    SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC1CE);

    CLEAR_BIT(TIM8->CCMR1, TIM_CCMR1_CC2S);
    SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC2PE);
//    SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); // PWM mode 1
    SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1); // PWM mode 1
    SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC2CE);

    CLEAR_BIT(TIM8->CCMR2, TIM_CCMR2_CC3S);
    SET_BIT(TIM8->CCMR2, TIM_CCMR2_OC3PE);
//    SET_BIT(TIM8->CCMR2, TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM mode 1
    SET_BIT(TIM8->CCMR2, TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1); // PWM mode 1
    SET_BIT(TIM8->CCMR2, TIM_CCMR2_OC3CE);

    //SET_BIT(TIM8->CR1, TIM_CR1_ARPE);

    SET_BIT(TIM8->EGR, TIM_EGR_UG);
    SET_BIT(TIM8->EGR, TIM_EGR_CC3G);

    SET_BIT(TIM8->BDTR, TIM_BDTR_MOE);
    SET_BIT(TIM8->BDTR, TIM_BDTR_AOE);
    SET_BIT(TIM8->BDTR, TIM_BDTR_BKE);
    SET_BIT(TIM8->BDTR, TIM_BDTR_BKP);
    SET_BIT(TIM8->BDTR, TIM_BDTR_OSSI);
    SET_BIT(TIM8->BDTR, TIM_BDTR_OSSR);


    SET_BIT(TIM8->DIER, TIM_DIER_CC3IE);

	//TIM8->CCR3 = 32000;
    TIM8->CCR1 = 100;
    TIM8->CCR2 = 200;
    TIM8->CCR3 = 300;
    //TIM8->RCR  = 1;

    CLEAR_BIT(TIM8->SR, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);

    SET_BIT(TIM8->CCER, TIM_CCER_CC1E);
    SET_BIT(TIM8->CCER, TIM_CCER_CC2E);
    SET_BIT(TIM8->CCER, TIM_CCER_CC3E);
    SET_BIT(TIM8->CCER, TIM_CCER_CC1NE);

	SET_BIT(TIM8->CR1, TIM_CR1_CEN);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void configTim1WithPE9Input(void)
{
	// Starting TIM1
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
	CLEAR_BIT(TIM1->CR1, TIM_CR1_DIR); // Up counter
	CLEAR_BIT(TIM1->CR1, TIM_CR1_OPM);
	//SET_BIT(TIM1->DIER, TIM_DIER_CC1IE); // Enable Capture/Compare Interrupt event
	SET_BIT(TIM1->DIER, TIM_DIER_UIE); // Enable Update Interrupt event
    SET_BIT(TIM1->DIER, TIM_DIER_CC1DE); // Capture/Compare DMA	 interrupt enable
    SET_BIT(TIM1->DIER, TIM_DIER_UDE);
    SET_BIT(TIM1->DIER, TIM_DIER_COMDE);
    SET_BIT(TIM1->DIER, TIM_DIER_TDE);
	// TIM1->CNT = 2000;
	// TIM1->ARR = 2000;
	TIM1->PSC = 60000;

	SET_BIT(TIM1->CCMR1, TIM_CCMR1_CC1S_0); // Set TIM1 as Input with mapped to TI1
	//SET_BIT(TIM1->CCMR1, TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1);
    SET_BIT(TIM1->EGR, TIM_EGR_UG);
    SET_BIT(TIM1->EGR, TIM_EGR_CC1G);

    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1P);
    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1NE);
    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1NP);
    SET_BIT(TIM1->CCER, TIM_CCER_CC1E); // Enable chanel 1

//    SET_BIT(TIM1->CCMR1, TIM_CCMR1_CC1S);
//    SET_BIT(TIM1->CCMR1, TIM_CCMR2_OC3PE);
//    SET_BIT(TIM1->CCMR1, TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM mode 1
//    SET_BIT(TIM1->CCMR1, TIM_CCMR2_OC3CE);

    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void configTim1ECM1_ETR_Trigger(void)
{
	// Starting TIM1
//	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
//	CLEAR_BIT(TIM1->CR1, TIM_CR1_DIR); // Up counter
//	CLEAR_BIT(TIM1->CR1, TIM_CR1_OPM);
//	SET_BIT(TIM1->DIER, TIM_DIER_CC1IE); // Enable Capture/Compare Interrupt event
//	SET_BIT(TIM1->DIER, TIM_DIER_UIE); // Enable Update Interrupt event
//    SET_BIT(TIM1->DIER, TIM_DIER_CC1DE); // Capture/Compare DMA	 interrupt enable
//    SET_BIT(TIM1->DIER, TIM_DIER_UDE);
//    SET_BIT(TIM1->DIER, TIM_DIER_COMDE);
//    SET_BIT(TIM1->DIER, TIM_DIER_TDE);
	// TIM1->CNT = 2000;
	// TIM1->ARR = 2000;
//	TIM1->PSC = 60000;

// Settings for trigger mode. It means when timer gets rising edge it's starts.
//    CLEAR_BIT(TIM1->SMCR, TIM_SMCR_ETF);
//    CLEAR_BIT(TIM1->SMCR, TIM_SMCR_ETPS);
//    CLEAR_BIT(TIM1->SMCR, TIM_SMCR_ETP);
//    SET_BIT  (TIM1->SMCR, TIM_SMCR_ECE);

    // This settings for using ECM1 with ETR - but in seems to me it is wrong
//    SET_BIT(TIM1->CCMR1, TIM_CCMR1_CC1S_0); // 01 - set TI1
//    CLEAR_BIT(TIM1->CCMR1, TIM_CCMR1_IC1F); // No filters is used
//    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1P);   // Catching on rising edge
//    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1NE);
//    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1NP);
    //SET_BIT(TIM1->SMCR, TIM_SMCR_SMS); // 111 - trigger mode
    //SET_BIT(TIM1->SMCR, TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0);
    //SET_BIT(TIM1->SMCR, TIM_SMCR_TS); // 111 mode - means ETR input


	//SET_BIT(TIM1->CCMR1, TIM_CCMR1_CC1S_0); // Set TIM1 as Input with mapped to TI1
	//SET_BIT(TIM1->CCMR1, TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1);
    //SET_BIT(TIM1->EGR, TIM_EGR_UG);
    //SET_BIT(TIM1->EGR, TIM_EGR_CC1G);

	// Starting TIM1
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
	CLEAR_BIT(TIM1->CR1, TIM_CR1_DIR); // Up counter
	CLEAR_BIT(TIM1->CR1, TIM_CR1_OPM);
	//SET_BIT(TIM1->DIER, TIM_DIER_CC1IE); // Enable Capture/Compare Interrupt event
	SET_BIT(TIM1->DIER, TIM_DIER_UIE); // Enable Update Interrupt event
    SET_BIT(TIM1->DIER, TIM_DIER_CC1DE); // Capture/Compare DMA	 interrupt enable
    //SET_BIT(TIM1->DIER, TIM_DIER_UDE); // Update DMA Enable
    SET_BIT(TIM1->DIER, TIM_DIER_COMDE);
    SET_BIT(TIM1->DIER, TIM_DIER_TDE);
    SET_BIT(TIM1->DIER, TIM_DIER_TIE);
	// TIM1->CNT = 2000;
	// TIM1->ARR = 2000;
	TIM1->PSC = 60000;

	SET_BIT(TIM1->CCMR1, TIM_CCMR1_CC1S_0); // 01 Set TIM1 as Input with mapped to TI1
	//SET_BIT(TIM1->CCMR1, TIM_CCMR1_CC1S_1); // 10 Set TIM1 as Input with mapped to TI2
	//SET_BIT(TIM1->CCMR1, TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1);
    //SET_BIT(TIM1->EGR, TIM_EGR_UG);
    //SET_BIT(TIM1->EGR, TIM_EGR_CC1G);

    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1P);
    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1NE);
    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1NP);

    SET_BIT(TIM1->SMCR, TIM_SMCR_TS); // 111 mode - means ETR input
    SET_BIT(TIM1->SMCR, TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0); // 101 - Gated mode
    //SET_BIT(TIM1->SMCR, TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1); // 110 - Trigger mode mode


    SET_BIT(TIM1->CCER, TIM_CCER_CC1E); // Enable chanel 1

    SET_BIT(TIM1->CR1, TIM_CR1_CEN);    // Enable timer
}
