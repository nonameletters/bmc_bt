#ifndef OS_THREADS_H
#define OS_THREADS_H

#include <ch.h>
#include <hal.h>

#include "board.h"
#include "shell_cmds.h"

extern volatile unsigned atx_state;
extern volatile unsigned sequencer_state;
extern volatile bool dfu_active;
extern volatile bool atx_present;

// atomic
extern volatile bool pwc_locked;
extern volatile int  pwc_state;

extern const zl2102_dev_t vtg[3];

extern struct usb_hid_in_report_s usb_hid_in_report;
extern const ina226_t ina[INA_SENSOR_COUNT];
extern virtual_timer_t bvt[2];

extern thread_t *printMessageThread;
thread_reference_t printMessageThreadRef;

#define PWC_MB_SIZE 8
static msg_t pwc_msgbuf[PWC_MB_SIZE];
MAILBOX_DECL(pwc_mb, pwc_msgbuf, sizeof(pwc_msgbuf)/sizeof(pwc_msgbuf[0]));

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// Shell main thread
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
THD_WORKING_AREA(waShell, SHELL_WA_SIZE);

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// Thread that can print to console some text from Interrupt
static THD_WORKING_AREA(waPrintToConsoleThread, 256);
static __attribute__((noreturn)) THD_FUNCTION(printToConsoleThread, arg)
{
	(void) arg;
	chRegSetThreadName("printToConsoleThread");

	BaseSequentialStream *outChannel2 = shell_cfg1.sc_channel;
	chSysLock();
	for(;;)
	{
		chSysUnlock();
			chprintf(outChannel2, " Power on was pressed \r\n");

		chSysLock();
			chThdSuspendTimeoutS(&printMessageThreadRef, TIME_INFINITE);
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// This function handles the press to service buttons
static void btn_do(void *arg)
{
  (void)arg;
  chSysLockFromISR();

	//   assert POWER_BTN_PORT == RESET_BTN_PORT
	//   assert RESET_BTN_PIN == 0
	//   assert POWER_BTN_PIN == 1
	//  palReadGroup(RESET_BTN_PORT, 0, 3); //
	  palTogglePad(LED_PORT,LED_PIN);
	  if (printMessageThread != NULL)
	  {
		  chThdResumeI(&printMessageThreadRef, MSG_OK);
		  //chSchWakeupS(printMessageThread, MSG_OK);
	  }
  chSysUnlockFromISR();
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
/* Triggered when the button is pressed or released. */
// POWER ON BUTTON handler GPIOB_1
static void extcb_rb(EXTDriver *extp, expchannel_t channel)
{
  (void)extp;
  chSysLockFromISR();

	  if (!chVTIsArmedI(&bvt[channel])) {   // first press
		// signal
		chMBPostI(&pwc_mb, (msg_t)(powerctl_rb+channel));
	  }
	  /* debounce 200mS.*/
	  chVTResetI(&bvt[channel]);
	  chVTDoSetI(&bvt[channel], MS2ST(200), btn_do, (void*)channel);

	  SET_BIT(EXTI->PR, EXTI_PR_PR1);
  chSysUnlockFromISR();
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// RESET BUTTON handler GPIOB_0
//void Vector58(void)
CH_IRQ_HANDLER(Vector58)
{
	CH_IRQ_PROLOGUE();
	  chSysLockFromISR();

//		  if (!chVTIsArmedI(&bvt[channel])) {   // first press
//			// signal
//			chMBPostI(&pwc_mb, (msg_t)(powerctl_rb+channel));
//		  }
//		  /* debounce 200mS.*/
//		  chVTResetI(&bvt[channel]);
//		  chVTDoSetI(&bvt[channel], MS2ST(200), btn_do, (void*)channel);

		  SET_BIT(EXTI->PR, EXTI_PR_PR0);
	  chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// MIPS reset or PowerGOOD
static void extcb_mr(EXTDriver *extp, expchannel_t channel) {
  (void)extp; (void)channel;
  chSysLockFromISR();
  uint32_t s = palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN) ? 0x100 : 0;
  if ( palReadPad(ATX_POWER_OK_PORT, ATX_POWER_OK_PIN) ) s |= 0x200;
  //volatile systime_t t = chVTGetSystemTimeX();
  if ( chMBGetUsedCountI(&pwc_mb) > 0 ) {
	  if ( (chMBPeekI(&pwc_mb) & 0xFF) == (msg_t)(powerctl_none) ) {
		  msg_t dummy;
		  chMBFetchI(&pwc_mb, &dummy);
	  }
  }
  chMBPostAheadI(&pwc_mb, (msg_t)(powerctl_none|s));	// power state change
  chSysUnlockFromISR();
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// VBUS_FS
void extcb_vusb(EXTDriver *extp, expchannel_t channel) {
  (void)extp; (void)channel;
  //chSysLockFromISR();
  //USBD1.state = USB_READY;
  //chSysUnlockFromISR();
  sysDfuStop(USBD1.alt_setting);
  _usb_reset(&USBD1);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, extcb_rb}, // 0 NOTE: extcb_rb use channels 0/1 only!
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, extcb_rb},	//
    {EXT_CH_MODE_BOTH_EDGES   | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD, extcb_mr},  // MIPS R# (D) SYS_OK (A)
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}, //4
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_BOTH_EDGES   | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, extcb_mr}, // 7 ATX Power GOOD
    {EXT_CH_MODE_DISABLED, NULL}, //8
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb_vusb},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}, //12
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}, //16
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void pwc_init(void) {
    pwc_state = 0;
    pwc_locked = 0;
    //chMBObjectInit (&pwc_mb, pwc_msgbuf, sizeof(pwc_msgbuf)/sizeof(pwc_msgbuf[0]));
    for (unsigned i=0; i < sizeof(bvt)/sizeof(bvt[0]); ++i) {
        chVTReset(&bvt[i]);
    }
    //initExtChannels();
    extStart(&EXTD1, &extcfg);
}

//static EXTChannelConfig channel0 = {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, extcb_rb};
//static EXTChannelConfig channel1 = {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, extcb_rb};
//static EXTChannelConfig channel2 = {EXT_CH_MODE_BOTH_EDGES   | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOD, extcb_mr};
//static EXTChannelConfig channel7 = {EXT_CH_MODE_BOTH_EDGES   | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, extcb_mr};
//static EXTChannelConfig channel9 = {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb_vusb};
//static EXTChannelConfig channelDisabled = {EXT_CH_MODE_DISABLED, NULL};
//
//static EXTConfig extcfg;

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
//static void initExtChannels(void)
//{
//	extcfg.channels[0]  = channel0;
//	extcfg.channels[1]  = channel1;
//	extcfg.channels[2]  = channel2;
//	extcfg.channels[3]  = channelDisabled;
//	extcfg.channels[4]  = channelDisabled;
//	extcfg.channels[5]  = channelDisabled;
//	extcfg.channels[6]  = channelDisabled;
//	extcfg.channels[7]  = channel7;
//	extcfg.channels[8]  = channelDisabled;
//	extcfg.channels[9]  = channel9;
//	extcfg.channels[10] = channelDisabled;
//	extcfg.channels[11] = channelDisabled;
//	extcfg.channels[12] = channelDisabled;
//	extcfg.channels[13] = channelDisabled;
//	extcfg.channels[14] = channelDisabled;
//	extcfg.channels[15] = channelDisabled;
//	extcfg.channels[16] = channelDisabled;
//	extcfg.channels[17] = channelDisabled;
//	extcfg.channels[18] = channelDisabled;
//	extcfg.channels[19] = channelDisabled;
//	extcfg.channels[20] = channelDisabled;
//	extcfg.channels[21] = channelDisabled;
//	extcfg.channels[22] = channelDisabled;
//}

#ifdef I2C_ADDR_SC18IS602
	unsigned rtd_error;
	volatile int rtd_t;
	/*
	const uint8_t rtd_init[] = {0x02,0xF6,0x0E,  0x02,0xF7,0x80, 0x02,0xF0,0x0D, 0x03,0x01,0x80,0x83, 0x06,0x01,3,0x55,0x40,0x36,0x00};
	const uint8_t rtd_autocheck[] = {0x03,0x01,0x80,0x85};
	const uint8_t rtd_status[]	= {0x03,0x01,0,0, 	0x20};
	const uint8_t rtd_faults[]	= {0x03,0x01,7,0, 	0x20};
	const uint8_t rtd_conv[]	= {0x03,0x01,0x80,0xA3};
	const uint8_t rtd_gpio[]	= {0x02,0xF5,0, 	0x10};
	const uint8_t rtd_rtd[]		= {0x04,0x01,1,0,0, 0x30};
	*/
	static THD_WORKING_AREA(waThread5, 256);
	static __attribute__((noreturn)) THD_FUNCTION(Thread5, arg) {
		(void)arg;
		chRegSetThreadName("rtd");
		while( pwc_state < 1 ) {
			chThdSleepMilliseconds(1000);
			continue;
		}
		smbWrite(0,I2C_ADDR_SC18IS602,0xF6,0x0E,1);  	// GPIO 1,2,3 CS0; GPIO3=RTD_READY#
		smbWrite(0,I2C_ADDR_SC18IS602,0xF7,0x80,1);  	// GPIO3-input-only
		smbWrite(0,I2C_ADDR_SC18IS602,0xF0,0xC|0x01,1); // SPI config: CPOL=1 MSB
		smbWrite(0,I2C_ADDR_SC18IS602,0x01,0x8380,2);   // Vbias,50Hz
		chThdSleepMilliseconds(100);    // Vbias settle
		uint8_t data[4];
		smbWrite(0,I2C_ADDR_SC18IS602,0x01,0x8580,2);    // Start self_check
		do {
			chThdSleepMilliseconds(100);
			smbWrite(0,I2C_ADDR_SC18IS602,0x01,0x00,2); // read config
			i2cReadBlock(I2C_ADDR_SC18IS602,data,2);	// get buffer
		} while (data[1] & 0xC);
		// read status
		smbWrite(0,I2C_ADDR_SC18IS602,0x01,0x07,2);		// read faults
		i2cReadBlock(I2C_ADDR_SC18IS602,data,2);   		// get buffer
		rtd_error = data[1] & ~0xC3;
		if( rtd_error != 0 ) {
			rtd_t = INT_MIN;
			chThdExit(-1);
		}

		uint16_t td;
		systime_t t0 = chVTGetSystemTime();
		while(1) {
			if( pwc_state < 1 ) {
				chThdSleepMilliseconds(1000);
				continue;
			}
			if( !smbWrite(0,I2C_ADDR_SC18IS602,0x01,0xA380,2) ) goto ec;    // Start conv
			do {
				chThdSleepMilliseconds(80); // Conv time
				smbWrite(0,I2C_ADDR_SC18IS602,0xF5,0,1);    // read gpio
				i2cReadBlock(I2C_ADDR_SC18IS602,data,1);	// get buffer
			} while ( !(data[0] & 0x8) );						// until data ready
			if( !smbWrite(0,I2C_ADDR_SC18IS602,0x01,1,3) ) goto ec;	// read temp to buffer
			i2cReadBlock(I2C_ADDR_SC18IS602,data,3);	// get buffer
			td = smbGetData_be(data+1, 2);
			//fault = td & 1;
			rtd_t = (td>>1) - 256*32;
			//if (usbGetDriverStateI(SDU1.config->usbp) == USB_ACTIVE) {
			//	chprintf((BaseSequentialStream*)&SDU1,"\r\nTrtd: %d\r\n", rtd_t);
			//}
	ec:
			t0 = chThdSleepUntilWindowed(t0, t0 + MS2ST(200));
		}
	}
#endif

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
/*
 * Power monitor thread
 */
static THD_WORKING_AREA(waThread4, 384);
// thread local
static ina226_data ina_d;
static __attribute__((noreturn)) THD_FUNCTION(Thread4, arg) {
	(void)arg;
	unsigned i;
	int skip=0;
	chRegSetThreadName("monitor");
	usb_hid_in_report.sequence_number = 0;
	systime_t t0 = chVTGetSystemTime();
	while(true) {
		usb_hid_in_report.sequence_number++;
#if HID_REPORT_TIME
		usb_hid_in_report.systime = ST2MS(t0);
#endif
		if(!dfu_active && sequencer_state && pwc_state>=1 && !pwc_locked) {
			for(i = 0; i < MIN_DEF(HID_REPORT_SENSOR_CHANNELS,(sizeof(ina)/sizeof(ina[0]))); ++i) {
				if ( ina226_read_data(&ina[i],&ina_d,ina_all) ) {
					chSysLock();
					usb_hid_in_report.report.ina[i].current.actual = ina_d.current;
#if HID_REPORT_POWER
					usb_hid_in_report.report.ina[i].power.actual = ina_d.power;
#endif
					usb_hid_in_report.report.ina[i].voltage.actual = ina_d.voltage;
					chSysUnlock();
				}
				//TODO: read error processing ?
			}
			if(skip) {
				--skip;
			} else {
				// In update_filtered_value function we expect future value of filter_current, so we increment the value in advance
#if HID_REPORT_STAT
				filter_current++;
				if(filter_current == FILTER_HISTORY_LENGTH) {
					filter_current = 0;
				}
				for(i = 0; i < MIN_DEF(HID_REPORT_SENSOR_CHANNELS,(sizeof(ina)/sizeof(ina[0]))); ++i) {
					update_filtered_value(&ina_history[i].current, &usb_hid_in_report.report.ina[i].current);
					update_filtered_value(&ina_history[i].power, &usb_hid_in_report.report.ina[i].power);
					update_filtered_value(&ina_history[i].voltage, &usb_hid_in_report.report.ina[i].voltage);
				}
#endif
			}
#if HID_REPORT_VTG
			for(i = 0; i < (sizeof(vtg)/sizeof(vtg[0])); ++i) {
				usb_hid_in_report.report.vtg_i[i] = smbRead(0, vtg[i].dev.addr, PMBUS_CMD_READ_IOUT, 2);
			}
#endif
		} else {
			// We skip a few first values from sensors to avoid bogus data in statistics
			skip = 10;
			memset(&usb_hid_in_report.report, 0, sizeof(usb_hid_in_report.report));
#if HID_REPORT_STAT
			filter_current = 0;
			memset(&ina_history, 0, sizeof(ina_history));
			for(i = 0; i < MIN_DEF(HID_REPORT_SENSOR_CHANNELS,(sizeof(ina)/sizeof(ina[0]))); ++i) {
				usb_hid_in_report.report.ina[i].current.min = INT32_MAX;
				usb_hid_in_report.report.ina[i].power.min = INT32_MAX;
				usb_hid_in_report.report.ina[i].voltage.min = INT32_MAX;
			}
#endif
		}

		if( (usbGetDriverStateI(&USBD1) == USB_ACTIVE) && (pwc_state == 2) && !(pwc_locked) ) {
			hidWriteReportt(&HID1, (uint8_t*)&usb_hid_in_report, HID_IN_REPORT_SIZE, TIME_IMMEDIATE);
		}
		t0 = chThdSleepUntilWindowed(t0, t0 + MS2ST(200));
	}
}

void double_reset_pulse(void)
{
	#ifdef BOARD_BE_BT_BFK20_CPU0
		//for(int i=200; --i && !palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN); ) { chThdSleepMilliseconds(2); }
		chThdSleepMilliseconds(200);
		MIPS_RESET_ASSERT();
		//for(int i=200; --i && palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN); ) { chThdSleepMilliseconds(2); }
		chThdSleepMilliseconds(200);	// 10ms->fault
		MIPS_RESET_RELEASE();
	#endif
}
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
/*
 * Power control thread
 */
static THD_WORKING_AREA(wapwcThread, 512);
static __attribute__((noreturn)) THD_FUNCTION(pwcThread, arg) {
  (void)arg;
  chRegSetThreadName("pwrctl");
  while(1) {
	systime_t timeout = TIME_INFINITE;
    while(1) {
        msg_t msg=powerctl_none;
        if ( MSG_OK != chMBFetch(&pwc_mb, &msg, timeout) )
            break;
        if ( (msg&0xFF) == powerctl_none ) break;
		timeout = TIME_INFINITE;
        if (pwc_locked) {
            if (msg == powerctl_on_unlock) {
                palSetPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN);	// release reset
                for(int i=5; pwc_state == 2 && --i && !palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN); ) {
                    chThdSleepMilliseconds(2);
                }
                double_reset_pulse();
                pwc_locked = 0;
                break;
            }
            continue;
        }
        if (msg == powerctl_on_lock) {
            pwc_locked = 1;
            palClearPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN);
            for(int i=5; --i && palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN); ) {
                chThdSleepMilliseconds(2);
            }
            msg = powerctl_on;
        }
        if (msg == powerctl_pb) {
			//BFK2.0
            if ((pwc_state == 0 || pwc_state==1)&&palReadPad(SEQ_CNTRL_PORT,SEQ_CNTRL_PIN))
                msg = powerctl_on;
            else
                msg = powerctl_off;
        } else if (msg == powerctl_rb) {
            if (pwc_state != 2)
				continue;
			palClearPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN);
			clk25_disable();
			sequencer_ctl(0);
			palSetPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN);
			pwc_state = 1;
			timeout=MS2ST(200);
			msg = powerctl_none;
			if ( MSG_OK != chMBFetch(&pwc_mb, &msg, MS2ST(200)) || (msg&0x1FF) == powerctl_none ) {
				chThdSleepMilliseconds(500);
				msg = powerctl_on;
			} else {
				if ( msg >= powerctl_on_lock ) {
					chSysLock();
					chMBResetI(&pwc_mb);
					chMBPostAheadI(&pwc_mb,msg);
					chSysUnlock();
				}
				break;
			}
        }

        if (msg == powerctl_on || (pwc_state!=2 && msg == powerctl_in)) {
            if (pwc_state == 0) {
                ATX_POWER_ON();
                for(int i=50; --i && !ATX_POWER_GOOD_STATE(); )
                    chThdSleepMilliseconds(10);
                if ((atx_state=ATX_POWER_GOOD_STATE())) {
					// remove notification from queue??
                    chThdSleepMilliseconds(15);		// extra delay for regulators power-up
                    pwc_state = (!smbBadBus(1)&&!smbBadBus(2)) ? 1 : -1;
                    if (pwc_state == 1) {
						if( !clk25_check_init() )
							pwc_state = -1;
					}
                } else
                    pwc_state = -1;
            } else  if (pwc_state==2 && msg == powerctl_in) {
				chSysLock();
                sequencer_ctl(0);
                timeout = MS2ST(100);
				chSysUnlock();
                continue;
            }
        }
        if (pwc_state == 1 && msg == powerctl_on) {
			MIPS_RESET_ASSERT();
			chThdSleepMilliseconds(2);		// wait for inv pullup
			clk25_disable();				// should be already disabled,
			chSysLock();
			chSysPolledDelayX(US2RTC(STM32_HCLK,50*4));	// >= 40uS
            sequencer_ctl(1);
            timeout = MS2ST(100);
			chSysUnlock();
			for(int i=50; i--;) {
				chThdSleepMilliseconds(10);	// no events: RESET# is overwritten, and SYS_OK have no interrupt
				if (palReadPad(SYSTEM_OK_PORT,SYSTEM_OK_PIN) ) {
					pwc_state = 2;		// before restore_vreg_vout
					break;
				} else if(!i) {
					pwc_state = -2;
				}
			}
			if( pwc_locked ) {
				continue;
			}
			restore_vreg_vout();
			// wait for voltage transition before enabling clock
			//  - extended from 5 to 100 seems to slightly increase stability on BFK2.0
			chThdSleepMilliseconds(100);
			clk25_enable();
			chThdSleepMilliseconds(100);					// >= 40uS*10periods (CLK), Total >200mS (Power)
			//chSysPolledDelayX(US2RTC(STM32_HCLK,40*10));	// >= 40uS*10periods

			//TODO: remove
			//NOTE: for testing ZL power on
			//chThdSleepMilliseconds(1000);

			MIPS_RESET_RELEASE();
			chMBFetch(&pwc_mb, &msg, MS2ST(2));
#ifdef BOARD_BE_BT_BFK20
			double_reset_pulse();
			chMBFetch(&pwc_mb, &msg, MS2ST(2));
#endif
			continue;
        }
        if (pwc_state == 2 && msg == powerctl_in) {
			chSysLock();
            sequencer_ctl(0);
            timeout = MS2ST(100);
			chSysUnlock();
			clk25_disable();
			continue;
        }
        if (msg == powerctl_off) {
			chSysLock();
            pwc_state = 1;
            sequencer_ctl(0);
			chSysUnlock();
			clk25_disable();
			if( atx_present ) {
				chThdSleepMilliseconds(100);
				chSysLock();
				atx_state = 0;
				pwc_state = 0;
				ATX_POWER_OFF();
				timeout = MS2ST(100);
				chSysUnlock();
			}
			continue;
        }
    }

	if( pwc_state < 0 ) continue;		// reset errors only by commands
    //int os = pwc_state;
    // recalc power state
    if ( !atx_present || (atx_state=ATX_POWER_GOOD_STATE()) != 0 ) {
        if ( palReadPad(SYSTEM_OK_PORT, SYSTEM_OK_PIN) && palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN) ) {
            pwc_state = 2;
        } else {
            if ( !(palReadLatch(MIPS_RESETn_PORT) & (1<<MIPS_RESETn_PIN)) ) {
                // also CPU_RESET button may affect the state
                uint32_t st = smbRead(0, I2C_ADDR_SQNR, PMBUS_CMD_STATUS_WORD, 2);
                if ( st & 0xFFFF0000 ) {	// read error
					if ( pwc_state > 0 ) pwc_state = -pwc_state;
					else pwc_state = -1;
				} else if ( !(st & PMBUS_ST_PWGDn) ) {
                    pwc_state = 2;
                } else {
                    if ( !(palReadLatch(SEQ_CNTRL_PORT) & (1<<SEQ_CNTRL_PIN)) ) {
                        pwc_state = -2;
                    } else {
                        pwc_state = 1;
                    }
                }
            } else {
                pwc_state = 1;
            }
        }
    } else if (ATX_POWER_ON_STATE()) {
        pwc_state = -1; // ATX FAIL
    } else {
        pwc_state = 0;
    }

    //if (usbGetDriverStateI(SDU1.config->usbp) == USB_ACTIVE && os!=pwc_state) {
    //  chprintf((BaseSequentialStream*)&SDU1,"\r\npower state change: %d -> %d\r\n", os, pwc_state);
    //}

  }
}
// ---------- ---------- ---------- ---------- ---------- ----------
extern uint16_t dmaTim1_buff[2];
// ---------- ---------- ---------- ---------- ---------- ----------

void dma2Stream1Channel6_Handler(void *p, uint32_t flags)
{
	volatile uint16_t res  = 0;

	if (READ_BIT(DMA2->LISR, DMA_LISR_TCIF1) == DMA_LISR_TCIF1)
	{
		res = (dmaTim1_buff[1] > dmaTim1_buff[0]) ? (dmaTim1_buff[1] - dmaTim1_buff[0]) : (dmaTim1_buff[0] - dmaTim1_buff[1]);
		if (res > 1000000)
		{
			res = 0;
		}

		SET_BIT(DMA2->LIFCR, DMA_LISR_TCIF1);
		SET_BIT(DMA2->LIFCR, DMA_LISR_HTIF1);
		//TIM1->SR = 0x00000000;
		//CLEAR_BIT(TIM1->SR, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);
	}

	TIM1->SR = 0x00000000;
	//CLEAR_BIT(TIM1->SR, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);
	if(STM32_DMA2_STREAM1->stream->NDTR == 0)
	{
		STM32_DMA2_STREAM1->stream->NDTR = 2;
		dmaStreamEnable(STM32_DMA2_STREAM1);
	}
}
// ---------- ---------- ---------- ---------- ---------- ----------
static THD_WORKING_AREA(ld1Thread, 256);
static __attribute__((noreturn)) THD_FUNCTION(trThread, arg)
{
  //	  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  //	  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  //	  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  // 	  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */

	uint32_t crSettings = 0x00000000;
	SET_BIT(crSettings, DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2); // channel6  is chosen

	SET_BIT(crSettings, DMA_SxCR_PL); // Priority very height
	SET_BIT(crSettings, DMA_SxCR_MSIZE_0); // Memory size 16 bit
	SET_BIT(crSettings, DMA_SxCR_PSIZE_0); // Peripheral size 16 bit

	SET_BIT(crSettings,   DMA_SxCR_MINC); // Incrementing memory address
	CLEAR_BIT(crSettings, DMA_SxCR_PINC); // Do not incrementing peripheral address
	//SET_BIT(crSettings,   DMA_SxCR_CIRC); // Circular mode
	CLEAR_BIT(crSettings, DMA_SxCR_DIR);  // direction from Peripheral to memory
	SET_BIT(crSettings,   DMA_SxCR_TCIE); // Enable transfer complete interrupt


	dmaInit(); // Resets to 0
	dmaStreamAllocate(STM32_DMA2_STREAM1, 5, dma2Stream1Channel6_Handler, NULL);
	STM32_DMA2_STREAM1->stream->CR = crSettings;
	STM32_DMA2_STREAM1->stream->M0AR = (uint32_t) dmaTim1_buff;
	STM32_DMA2_STREAM1->stream->M0AR = (uint32_t) dmaTim1_buff;
	STM32_DMA2_STREAM1->stream->PAR  = (uint32_t) &TIM1->CCR1;
	STM32_DMA2_STREAM1->stream->NDTR = 2;

	dmaStreamEnable(STM32_DMA2_STREAM1);

	//enableDMATim1();

	//configTim1WithPE9Input(); // - This function works;
	configTim1ECM1_ETR_Trigger();

	configTim8ToGeneratePWM();

	nvicEnableVector(TIM8_UP_TIM13_IRQn, CORTEX_PRIO_MASK(STM32_GPT_TIM1_IRQ_PRIORITY));
	nvicEnableVector(TIM8_CC_IRQn, CORTEX_PRIO_MASK(STM32_GPT_TIM1_IRQ_PRIORITY));
	nvicEnableVector(TIM1_CC_IRQn, CORTEX_PRIO_MASK(STM32_GPT_TIM1_IRQ_PRIORITY));
	nvicEnableVector(TIM1_TRG_COM_TIM11_IRQn, CORTEX_PRIO_MASK(STM32_GPT_TIM1_IRQ_PRIORITY));
	// nvicEnableVector(DMA2_Stream1_IRQn, CORTEX_PRIO_MASK(7));


	while(1)
	{

//		if (READ_BIT(TIM8->CCER, TIM_CCER_CC3E) != TIM_CCER_CC3E)
//		{
//			chThdSleepMilliseconds(500);
//			SET_BIT(TIM8->CCER, TIM_CCER_CC3E);
//		}
	}
}

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/
static const uint16_t led_idle[]       = {100, 900, 0};
static const uint16_t led_conn[]       = {200, 800, 0};
static const uint16_t led_pon[]        = {800, 200, 0};
static const uint16_t led_fail[]       = {100, 200, 100, 700, 0};
static const uint16_t led_prog[]       = {400, 100, 0};

/*
 * Blinker thread, times are in milliseconds.
 */

static THD_WORKING_AREA(waThread1, 256);
static __attribute__((noreturn)) THD_FUNCTION(Thread1, arg)
{
    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
    	const uint16_t *player = ((pwc_state < 0 ||
			(!palReadPad(SEQ_CNTRL_PORT,SEQ_CNTRL_PIN)&&!palReadPad(SYSTEM_OK_PORT,SYSTEM_OK_PIN))) ? led_fail
                : ((dfu_active|pwc_locked) ? led_prog :
                    (pwc_state == 2 ? led_pon : (usbGetDriverStateI(serusbcfg1.usbp) != USB_ACTIVE ? led_idle : led_conn))));
    	palSetPad(LED_PORT, LED_PIN);
    	while(*player) {
    		chThdSleepMilliseconds(*player);
    		palTogglePad(LED_PORT, LED_PIN);
    		player++;
    	}
    }
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
/*
 * HID input report processing thread
 */
uint8_t my_in_report[HID_OUT_REPORT_SIZE];
static THD_WORKING_AREA(waThread3, 256);
static __attribute__((noreturn)) THD_FUNCTION(Thread3, arg) {
    (void)arg;
    chRegSetThreadName("hid_input");
    while(true) {
        size_t n = hidReadReport(&HID1, my_in_report, sizeof(my_in_report));
        if (!n) {
            chThdSleepMilliseconds(350);
        }
    }
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// This functions added there because they are closely kneed
int powerctlI(powerctl_req req)
{
    if (req >= powerctl_on_lock) {
        chMBPostAheadI(&pwc_mb, (msg_t)(req));
    } else {
    if (pwc_locked) return 1;
        chMBPostI(&pwc_mb, (msg_t)(req));
    }
    return 0;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
int powerctl(powerctl_req req)
{
    if ( pwc_locked && req != powerctl_on_unlock ) {
        return 1;
    }
    chMBPost(&pwc_mb, (msg_t)(req), TIME_INFINITE);
    return 0;
}

/*
 * DFU connectors
 * power switching should be in separate process; mutex buttons and commands
 * do not rely on the ATX/sequencer
 * flash aquire/release
 */
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
bool sysDfuReady(unsigned alt)
{
    if( alt == 1 ) return true;
    return ( dfu_active && pwc_locked && (!atx_present||ATX_POWER_GOOD_STATE()) && !palReadPad(SEQ_CNTRL_PORT,SEQ_CNTRL_PIN) );
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void sysDfuStart(unsigned alt)
{
    if( alt == 1 || (dfu_active && pwc_locked) ) return;
    dfu_active = 1;
#if 1
    //powerctl(powerctl_on_lock);
    chSysLockFromISR();
    powerctlI(powerctl_on_lock);
    chSysUnlockFromISR();
#else
    // ATX power must be ON
    // CPU must be in RESET state
    while( palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN) )
    {
        palClearPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN);
    }
    //TODO: should be a ON/CHECK phase
    ATX_POWER_ON();
    sequencer_ctl(1);
    //sFLASH_AQUIRE(SFDU);
#endif
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void sysDfuStop(unsigned alt)
{
    if( alt == 1 || !pwc_locked ) return;
    //sFLASH_RELEASE(SFDU);
	#if 1
		chSysLockFromISR();
		powerctlI(powerctl_on_unlock);
		chSysUnlockFromISR();
		dfu_active = 0;
	#else
		int i = 20;
		while( !palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN) && i-- )
		{
			palSetPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN);
		}
	#endif
}
#endif //OS_THREADS_H
