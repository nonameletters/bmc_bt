/*
 * Board connections
 *
 * Copyright (C) 2016, 2017 Baikal Electronics.
 *
 * Author: Alexander Osipenko <Alexander.Osipenko@baikalelectronics.ru>
 */

#ifndef _BOARD_H_
#define _BOARD_H_

// Board type is defined in Makefile
// See UDEFS += -D"BOARD_TYPE" lint \" this symbol must be removed
#if !defined(BOARD_BE_BT_BFK20) && !defined(BOARD_BE_BT_BFK16) && \
	!defined(BOARD_BE_BT_NUCLEO_207ZG) && !defined(BOARD_BE_BT_BFK30)
	#error "Choose which board you use."
#endif

#if defined(BOARD_BE_BT_BFK20)
	#include "board_bfk20.h"
#elif defined(BOARD_BE_BT_BFK16)
	#include "board_bfk16.h"
#elif defined(BOARD_BE_BT_NUCLEO_207ZG)
	#include "board_nuc207zg.h"
#elif defined(BOARD_BE_BT_BFK30)
	#include "board_bfk30.h"
#endif



#if !defined(_FROM_ASM_)
	#ifdef __cplusplus
		extern "C" {
	#endif
			void boardInit(void);
	#ifdef __cplusplus
		}
	#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
