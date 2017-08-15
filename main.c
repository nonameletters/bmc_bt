/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>
#include <stdlib.h>		// for strtol
#include <limits.h>

#include <ch.h>
#include <hal.h>

#include "shell.h"
#include "chprintf.h"

#include "dfu.h"
#include "usbcfg.h"
#include "spi_flash.h"
#include "x.h"
#include "clk156.h"
#include "tps56.h"
#include "chipdrivers.h"

#include "hal_usb_hid.h"
#include "report.h"

#include "shell_cmds.h"
#include "pac1720.h"

#include "board.h"
#include "spi_flash.h"

#include "adc.h"
#include "adc_lld.h"
#include "tim1_tim8.h"

#include "os_threads.h"
#include "handlers.h"
#include "power_ctl.h"
#include "common_bmc.h"


thread_t *shelltp = NULL;
bool smb_isShellThread(void) { return shelltp == chThdGetSelfX(); }

/*
 * Application entry point.
 */
int main(void) {
    halInit();

//	atx_present = !ATX_POWER_GOOD_STATE();
//	bool mfg_mode = !palReadPad(RESET_BTN_PORT,RESET_BTN_PIN);

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread (???) and the
     *   RTOS is active.
     */
    chSysInit();

//    pwc_init();
    // Sequencer is disabled
    // sequencer_ctl(0);
	// release SQNR reset
    //palSetPad(SEQ_nRESET_PORT,SEQ_nRESET_PIN);


//#ifdef BOARD_BE_BT_BFK20
//	// Power supply compatibility
//	ATX_POWER_ON();
//	chThdSleepMilliseconds(100);
//	// palSetPad(SEQ_NOTIFY_PORT,SEQ_NOTIFY_PIN);
//	chThdSleepMilliseconds(50);
//	pwc_state = 1;
//#endif

// System hangs, if I2CD3 initialized...
    //i2cStart(&I2CD3, &i2cfg3);
//	systime_t sqnr_rst = chVTGetSystemTime();
//	unsigned startfault;
	
//	while(1) {
//		//check_i2c_ready()
//		startfault = (!palReadPad(I2C1_SCL_PORT,I2C1_SCL_PIN) || !palReadPad(I2C1_SDA_PORT,I2C1_SDA_PIN));
//		startfault |= (!palReadPad(I2C2_SCL_PORT,I2C2_SCL_PIN) || !palReadPad(I2C2_SDA_PORT,I2C2_SDA_PIN)) ? 2:0;
//		if( !startfault ) break;
//reinit_clk25:
//		palClearPad(SEQ_NOTIFY_PORT,SEQ_NOTIFY_PIN);
//	    palClearPad(SEQ_nRESET_PORT,SEQ_nRESET_PIN);
//		chThdSleepMilliseconds(400);
//	    palSetPad(SEQ_nRESET_PORT,SEQ_nRESET_PIN);
//		sqnr_rst = chVTGetSystemTime();
//		palSetPad(LED_PORT, LED_PIN);
//		chThdSleepMilliseconds(100);
//		palClearPad(LED_PORT, LED_PIN);
//		palSetPad(SEQ_NOTIFY_PORT,SEQ_NOTIFY_PIN);
//
//		chThdSleepMilliseconds(500);
//
//		for( unsigned i = 0; i < 2+startfault*2; ++i) {
//			palTogglePad(LED_PORT, LED_PIN);
//			chThdSleepMilliseconds(i&1?110:40);
//		}
//		palClearPad(LED_PORT, LED_PIN);
//		if( mfg_mode ) break;
//	}
//	chThdSleepMilliseconds(2);		// 50us I2C idle time



/* This section of code assimilated with thread "wapwcThread"
 * if you comment or uncomment these lines without thread "wapwcThread"
 * the program enter in infinite cycle
#ifdef BOARD_BE_BT_BFK20
    i2cStart(&I2CD1, &i2cfg1);
    i2cStart(&I2CD2, &i2cfg2);
#else
    i2cStart(&I2CD1, &i2cfg2);
    i2cStart(&I2CD2, &i2cfg2);
#endif


    //palSetPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN);

	// UCD9090 reset time
	chThdSleepUntilWindowed(sqnr_rst, sqnr_rst+MS2ST(200));
#ifdef BOARD_BE_BT_BFK20
	// BFK2.0: requires 12Vin + rail 3.3V enabled
	if( !mfg_mode && !clk25_init() ) {
		startfault=4;
		i2cStop(&I2CD1);
		i2cStop(&I2CD2);
		goto reinit_clk25;
	}
#endif
	if( !mfg_mode && !ucd9090_init_pageinfo() ) {
		startfault=5;
		i2cStop(&I2CD1);
		i2cStop(&I2CD2);
		goto reinit_clk25;
	}

    check operation status and...
    for(unsigned i = 0; i < (sizeof(ina)/sizeof(ina[0])); i++) {
        ina226_config(&ina[i]);
    }
*/

    /*Start ADC to get inside temperature of BMC*/
    ADCConfig tempSensor;
    adcStart(&ADCD1, &tempSensor);
    adcSTM32EnableTSVREFE();

/* Test code for I2C2 on NUCLEO */
    i2cStart(&I2CD1, &i2cfg2);
    i2cStart(&I2CD2, &i2cfg2);

    usbSerialInit();

    /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg1);

    /*
     * Initializes uart1
     */
    //sdStart(&SD3, 0);

// I commented next lines to switch of
    // Init SPI flash interface on SPI1
//    sFLASH_Init(&SFDU1, &sFlash1);
//    // flash is not powered on, probe_spi will fail
//    sFLASH_Start(&SFDU1);   // start spi driver
//#if DFU_INTERFACES > 1
//    sFLASH_Init(&SFDU2, &sFlash2);
//#endif
//
//#ifdef VIDEO_SPI_PORT
//    // Init SPI flash interface on SPI2
//    sFLASH_Init(&SFDU3, &sFlash3);
//    sFLASH_Start(&SFDU3);
//#endif

    /*
     * Initializes a USB DFU driver.
     */
    dfuObjectInit(&DFU1);
    dfuStart(&DFU1, &dfuusbcfg1);
    // DFU unable to recover state after incorrect requests
    // DFU programming can take a long time; should use a thread

    /*
     * Initializes a USB HID driver.
     */
    hidObjectInit(&HID1);
    hidStart(&HID1, &hidusbcfg1);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
//  usbDisconnectBus(&USBD1);
//  chThdSleepMilliseconds(1500);

//  chThdSleepMilliseconds(500);
    usbStart(&USBD1, &usbcfg);
    usbConnectBus(&USBD1);
    USBD1.otg->GCCFG = GCCFG_NOVBUSSENS|GCCFG_PWRDWN;   // fixed ChibiOS patch

    /*
     * Creates the blinker thread.
     */
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
    // USB must be syncronized first;
    chThdCreateStatic(ld1Thread, sizeof(ld1Thread), NORMALPRIO-1, trThread, NULL);

//    chThdCreateStatic(wapwcThread, sizeof(wapwcThread), NORMALPRIO+1, pwcThread, NULL);

// no messages & i2c access
//#if 1
//    chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO-1, Thread3, NULL);
//
//    chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, Thread4, NULL);
//#endif
//
//#ifdef I2C_ADDR_SC18IS602
//    chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, Thread5, NULL);
//#endif

    /*
     * Shell manager initialization.
     */
    shellInit();

    /*
     * Normal main() thread activity, in this demo it does nothing except
     * sleeping in a loop and check the button state.
     */
    while (true) {
		#if defined(BMC_WITH_SYSTEM_CONSOLE)
        	if(password_good()) {
        		if (!shelltp && (usbGetDriverStateI(SDU1.config->usbp) == USB_ACTIVE))
        		{
        			shelltp = shellCreateStatic(&shell_cfg1, waShell, sizeof(waShell), NORMALPRIO-1);
        		}
        		else if (chThdTerminatedX(shelltp))
        		{
        			//chThdRelease(shelltp);    /* Recovers memory of the previous shell - if not static   */
        			shelltp = NULL;           /* Triggers spawning of a new shell.        */
        			password_reset();

            	}

            	chThdSleepMilliseconds(500);
        	}
        	else
        	{
            	password_check(chSequentialStreamGet(&SDU1));
            	chThdSleepMilliseconds(100);
        	}
		#else
        	chThdSleepMilliseconds(1000);
		#endif
    }
}

// vim:sw=4:ts=4:et
