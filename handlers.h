#ifndef HANDLERS_H
#define HANDLERS_H

// ---------- ---------- ---------- ---------- ---------- ----------
CH_FAST_IRQ_HANDLER(VectorA8)
{
	static uint32_t testCounter = 0;
	testCounter++;
}

// ---------- ---------- ---------- ---------- ---------- ----------
// STM32_TIM1_UP_HANDLER
CH_FAST_IRQ_HANDLER(STM32_TIM1_CC_HANDLER)
{
//	static uint16_t prev = 0;
//	static uint16_t cur  = 0;
//	static uint16_t res  = 0;
//
//	res = 0;
//	prev = cur;
//    cur  = TIM1->CCR1;
//
//    res = (cur > prev) ? (cur - prev) : (65535+ cur - prev);
//    CLEAR_BIT(TIM1->SR, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
//    CLEAR_BIT(TIM1->SR, TIM_SR_CC1IF);
//    CLEAR_BIT(TIM1->SR, TIM_SR_CC1OF);
//    SET_BIT(TIM1->EGR, TIM_EGR_CC1G);

//	if (capture >= 10)
//	{
//		CLEAR_BIT(TIM8->CCER, TIM_CCER_CC3E);
//		capture = 0;
//	}
}

// ---------- ---------- ---------- ---------- ---------- ----------
CH_FAST_IRQ_HANDLER(STM32_TIM8_UP_HANDLER)
{
//	static uint8_t delay = 0;
//
//	CLEAR_BIT(TIM8->SR, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
	CLEAR_BIT(TIM8->SR, TIM_SR_UIF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
//	if (delay == 5)
//	{
//		CLEAR_BIT(TIM8->CCER, TIM_CCER_CC3P);
//		SET_BIT(TIM8->CCER, TIM_CCER_CC3E); // Enable chanel 3
//			CLEAR_BIT(TIM8->CCER, TIM_CCER_CC3NP);
//		    SET_BIT(TIM8->CCER, TIM_CCER_CC3NE);
//
//
//	}
//	else if (delay == 10)
//	{
//		CLEAR_BIT(TIM8->CCER, TIM_CCER_CC1P);
//		SET_BIT(TIM8->CCER, TIM_CCER_CC1E); // Enable chanel 1
//			CLEAR_BIT(TIM8->CCER, TIM_CCER_CC1NP);
//		    SET_BIT(TIM8->CCER, TIM_CCER_CC1NE);
//
//	}
//	else if (delay == 15)
//	{
//		CLEAR_BIT(TIM8->CCER, TIM_CCER_CC2P);
//		SET_BIT(TIM8->CCER, TIM_CCER_CC2E); // Enable chanel 2
//			CLEAR_BIT(TIM8->CCER, TIM_CCER_CC2NP);
//		    SET_BIT(TIM8->CCER, TIM_CCER_CC2NE);
//
//	}
//	else if (delay > 20)
//	{
//		delay = 20;
//	}
//
//	delay++;
//	if (READ_BIT(TIM8->SR, TIM_SR_UIF))
//	{
//		palTogglePad(GPIOB, 0);
//		// CLEAR_BIT(TIM8->SR, TIM_SR_UIF);
//		CLEAR_BIT(TIM8->SR, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
//		//CLEAR_BIT(TIM8->CCER, TIM_CCER_CC3E);
//	}
}

// ---------- ---------- ---------- ---------- ---------- ----------
CH_FAST_IRQ_HANDLER(STM32_TIM8_CC_HANDLER)
{
	static uint8_t capture1 = 0;
	static uint8_t capture2 = 0;
	static uint8_t capture3 = 0;

	if (READ_BIT(TIM8->SR, TIM_SR_CC1IF) == TIM_SR_CC1IF)
	{
		CLEAR_BIT(TIM8->SR, TIM_SR_CC1IF);
		if (capture1 == 0)
		{
			TIM8->CCR1 = 50;
			capture1 = 1;
		}
		else
		{
			TIM8->CCR1 = 200;
			capture1 = 0;
		}
	}

	if (READ_BIT(TIM8->SR, TIM_SR_CC2IF) == TIM_SR_CC2IF)
	{
		CLEAR_BIT(TIM8->SR, TIM_SR_CC2IF);
		if (capture2 == 0)
		{
			TIM8->CCR2 = 150;
			capture2 = 1;
		}
		else
		{
			TIM8->CCR2 = 300;
			capture2 = 0;
		}
	}

	if (READ_BIT(TIM8->SR, TIM_SR_CC3IF) == TIM_SR_CC3IF)
	{
		CLEAR_BIT(TIM8->SR, TIM_SR_CC3IF);
		if (capture3 == 0)
		{
			TIM8->CCR3 = 250;
			capture3 = 1;
		}
		else
		{
			TIM8->CCR3 = 400;
			capture3 = 0;
		}
	}
}

// ---------- ---------- ---------- ---------- ---------- ----------
CH_FAST_IRQ_HANDLER(STM32_TIM1_UP_HANDLER)
{
	static uint32_t testCounter = 0;
	testCounter++;
}

#endif //HANDLERS_H
