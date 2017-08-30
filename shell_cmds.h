/*
 * shell_cmds.h
 *
 *  Created on: Jun 7, 2017
 *      Author: root
 */

#ifndef SHELL_CMDS_H_
#define SHELL_CMDS_H_

#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <string.h>

//#include "pal_lld.h"
//#include "adc_lld.h"
#include "smbus.h"
#include "pac1720.h"
#include "x.h"

#include "board.h"
#include "report.h"
#include "dfu.h"
#include "shell.h"

// This ShellConfig contains "sc_channel", use it to write data to stream
extern const ShellConfig shell_cfg1;

#if defined(BOARD_BE_BT_NUCLEO_207ZG)
	#include "stm32f207xx.h"
#else
	#include "stm32f205xx.h"
#endif


#define VREG_FLAG_PMBUS		1	// supports
#define VREG_FLAG_STNDBY	2	// can be accessed on standby power
#define VREG_FLAG_PWRIN		4	// can be accessed on input power (12V)
#define VREG_FLAG_PERM		8	// permanent voltage setting

#define TH02_REG_STATUS  0x00
#define TH02_REG_DATAH   0x01
#define TH02_REG_DATAL   0x02
#define TH02_REG_CONFIG  0x03
#define TH02_REG_ID      0x11

#define TH02_DEV_ID_7    0x40

#define TH02_TEMP_CONV   0x11
#define TH02_HUM_CONV    0x01



typedef enum
{
	TEMP,
	HUM
} mesurment_t;

void cmd_start(BaseSequentialStream *chp, int argc, char *argv[]);


void cmd_temp(BaseSequentialStream *chp, int argc, char *argv[]);


void cmd_pac1720(BaseSequentialStream *chp, int argc, char *argv[]);

void cmd_insideTemperature(BaseSequentialStream *chp, int argc, char *argv[]);

uint16_t getTH02Value(BaseSequentialStream *chp, mesurment_t value, I2CDriver* i2cp);

void printMessageToConsole(char *msg);

// ---------- ---------- ---------- ---------- ---------- ----------
// This function is uses only in cmd_gpio
const char* okz(unsigned t);

// ---------- ---------- ---------- ---------- ---------- ----------
// Danger: could hang CPU on invalid address read
void cmd_gpio(BaseSequentialStream *chp, int argc, char *argv[]);

/*
 * Use this command to control sequencer.
 */
void cmd_sqnr(BaseSequentialStream *chp, int argc, char *argv[]);

void cmd_flash_id(BaseSequentialStream *chp, int argc, char *argv[]);

void cmd_atx(BaseSequentialStream *chp, int argc, char *argv[]);

/*
 *  Use this command to control/check state of on board voltage regulators.
 * format:
 *   vtg ([regulator] [xx.xxx]|store)?
 * - without argument should print list of available regulators
 * - if only regulator is provided, print voltage regulator state (VIN, VOUT, IOUT, TEMP)
 * - set [xx.xxx] sets voltage output in volts
 * - store saves current settings to regulator's nvram
 */
void cmd_vtg(BaseSequentialStream *chp, int argc, char *argv[]);

/*
 *  Use this command to control/check state of on board voltage regulators.
 * format:
 *   vreg ([regulator] [xx.xxx]|store)?
 * - without argument should print list of available regulators
 * - if only regulator is provided, print voltage regulator state (VIN, VOUT, IOUT, TEMP)
 * - set [xx.xxx] sets voltage output in volts
 * - store saves current settings to regulator's nvram
 *snapshots for ZL2102
 */

void cmd_vreg(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_clk156(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_clk25(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_vpll(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_smb(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_l11(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_ascii(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_factory(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_bootcfg(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_cpureset(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_dfu(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_sensors(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_power(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_ina(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_tps56(BaseSequentialStream *chp, int argc, char *argv[]);


/* ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- */
/* Auxiliary functions                                                                     */
bool        tps_setmv(const vreg_t* vr, unsigned mv);
bool        tps_show(const vreg_t* vr, BaseSequentialStream* chp);
bool        vpll_setmv(const vreg_t* vr, unsigned mv);
bool        vpll_show(const vreg_t* vr, BaseSequentialStream* chp);
bool        ltc_setmv(const vreg_t* vr, unsigned mv);
bool        tps_setmv(const vreg_t* vr, unsigned mv);
bool        zl2102_show(const vreg_t* vreg, BaseSequentialStream* chp);
bool        zl2102_store(const vreg_t* vreg);
bool        zl2102_setmv(const vreg_t* vreg, unsigned mv);
int         isTrue(const char* str);
void        sequencer_ctl(int st);
bool        sequencer_alert(void);
const char* sequencer_status(void);
unsigned    system_power_good_state(void);
unsigned    sqnr_state(void);
bool        ucd9090_init_pageinfo(void);
int         ucd_get_page(const char* str);
void        zl2102_setv(const zl2102_dev_t* vtg, const fixed_val_t* f_val);
void        zl2102_save(const zl2102_dev_t* vtg);
void        restore_vreg_vout(void);
void        ina_display(BaseSequentialStream *chp, size_t i);

#endif /* SHELL_CMDS_H_ */
