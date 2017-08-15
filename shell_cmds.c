/*
 * shell_cmds.c
 *
 *  Created on: Jun 7, 2017
 *      Author: root
 */
#include "shell_cmds.h"
//#include "common_bmc.h"

volatile unsigned atx_state;
volatile unsigned sequencer_state;
volatile bool     dfu_active;
volatile bool     atx_present;

// atomic
volatile bool pwc_locked;
volatile int  pwc_state;

SerialUSBDriver SDU1;
DFU_USBDriver DFU1;
USBHIDDriver HID1;

virtual_timer_t bvt[2];

extern const I2CConfig i2cfg3;
extern uint16_t vpll_code;

char con_str_buf[20];
static const uint8_t sqnr_reset_config[] = { 0xD2, 0x09, 0x03, 0xFF, 0x03, 0x4F, 0x00, 0x06, 0x00, 0x00, 0x03 };

const char* const ucd9090_common_faults[] = {
    "LOG_NOT_EMPTY","SYSTEM_WATCHDOG_TIMEOUT","RESEQUENCE_ERROR","WATCHDOG_TIMEOUT","RESERVED4","RESERVED5","RESERVED6","RESERVED7","Common: "
};
const char* const ucd9090_page_faults[] = {
    "VOUT_OV","VOUT_UV","TON_MAX","IOUT_OC","IOUT_UC","TEMPERATURE_OT","SEQ_ON_TIMEOUT","SEQ_OFF_TIMEOUT","Page#%02d"
};
const char* const ucd9090_gpi_faults[] = {
    "GPI1","GPI2","GPI3","GPI4","GPI5","GPI6","GPI7","GPI8","GPI:    "
};

const char* const ucd9090_fault_types_paged[] = {
	"VOUT_OV", "VOUT_UV", "TON_MAX", "IOUT_OC", "IOUT_UC", "OT", "SEQ_ON", "SEQ_OFF",
};

#if defined(BOARD_BE_BT_BFK20) || defined(BOARD_BE_BT_NUCLEO_207ZG)
	// DDR1.8  GPIO19(11)  - GPI1(8)
	// SATA1.8 GPIO20(14)  - GPI2(9)
	const char* const ucd9090_rails[] = {
		"PLL", "0.9V", "1.5V",    "1.8V",    "0.9V_SATA", "0.95V_PCIe", "0.95V_10G", "3.3V",    "1.5V_PCIe", "1.5V_10G", "Unused"
	};
	//	 M10     M4      M5        M6        M7            M8           M9            M3        M1            M2
	static const uint8_t ucd9090_rails_en[] = {
	UCD_GPIO1, UCD_GPIO2,UCD_GPIO3,UCD_GPIO4,UCD_GPIO13,   UCD_GPIO14,  UCD_GPIO15,  UCD_GPIO16,UCD_GPIO17,   UCD_GPIO18, UCD_GPIO9
	};
#else
	const char* const ucd9090_rails[] = {
		"12V", "5V", "3.3V", "0.9V", "1.5V", "1.8V", "0.95V", "Unused", "Unused", "Unused", "Unused"
	};
#endif

const zl2102_dev_t vtg[3] = {
	    {{NULL, 0x20|I2C_BUS2, "v0_9"}, {1, 200,0}, {0, 901,0} },
	    {{NULL, 0x21|I2C_BUS2, "v1_5"}, {1, 900,0}, {1, 500,0} },
	    {{NULL, 0x38|I2C_BUS2, "v1_8"}, {2, 200,0}, {1, 800,0} }
	};

typedef struct {
	int8_t vout_mode;
} ucd9090_pageinfo_t;
// gain: 3.3/2 5.0/2 12/2

ucd9090_pageinfo_t ucd9090_pageinfo[UCD9090_PAGES];

const vreg_t vreg[] = {
    {0x20|I2C_BUS2, 	"v09",  	0, QN_VAL(0.700), QN_VAL(0.900), QN_VAL(1.200), VREG_FLAG_PMBUS|VREG_FLAG_PWRIN|VREG_FLAG_PERM,	zl2102_setmv, zl2102_show, zl2102_store},
    {0x21|I2C_BUS2, 	"v15",  	0, QN_VAL(1.200), QN_VAL(1.500), QN_VAL(1.800), VREG_FLAG_PMBUS|VREG_FLAG_PWRIN|VREG_FLAG_PERM,	zl2102_setmv, zl2102_show, zl2102_store},
    {0x38|I2C_BUS2, 	"v18",  	0, QN_VAL(1.200), QN_VAL(1.800), QN_VAL(2.300), VREG_FLAG_PMBUS|VREG_FLAG_PWRIN|VREG_FLAG_PERM,	zl2102_setmv, zl2102_show, zl2102_store},
#ifdef BOARD_BE_BT_BFK20
    {0,             	"pll",  	0, QN_VAL(0.900), QN_VAL(0.900), QN_VAL(1.150), VREG_FLAG_STNDBY, 					vpll_setmv,   vpll_show,   0},
    {I2C_ADDR_TPS65263, "sata090",	0, QN_VAL(0.700), QN_VAL(0.900), QN_VAL(1.200), 0, 									tps_setmv,    tps_show,    0},
    {I2C_ADDR_TPS65263, "pcie095",	1, QN_VAL(0.700), QN_VAL(0.950), QN_VAL(1.200), 0,									tps_setmv,    tps_show,    0},
    {I2C_ADDR_TPS65263, "xgbe095", 	2, QN_VAL(0.700), QN_VAL(0.950), QN_VAL(1.200), 0,									tps_setmv,    tps_show,    0},
{I2C_ADDR_LTC_1p5_PCIE, "pcie15",	0, QN_VAL(1.200), QN_VAL(1.500), QN_VAL(1.800), 0, ltc_setmv, 0, 0},
{I2C_ADDR_LTC_1p5_XGBE, "xgbe15",	0, QN_VAL(1.200), QN_VAL(1.500), QN_VAL(1.800), 0, ltc_setmv, 0, 0},
{I2C_ADDR_LTC_1p8_SATA, "sata18",	0, QN_VAL(1.200), QN_VAL(1.800), QN_VAL(2.000), 0, ltc_setmv, 0, 0},
{I2C_ADDR_LTC_1p8_DDR,  "ddr18",	0, QN_VAL(1.200), QN_VAL(1.800), QN_VAL(2.000), 0, ltc_setmv, 0, 0},
#endif
};

unsigned vreg_data[sizeof(vreg)/sizeof(vreg[0])];

static uint16_t buf[4] = {0x00, 0x00, 0x00, 0x00};
ADCConversionGroup group = { 0,                       \
                             1,                       \
                             NULL,                    \
                             NULL,                    \
                             0x00,                    \
                             ADC_CR2_SWSTART,         \
                             ADC_SMPR1_SMP_SENSOR(0), \
                             0x00,                    \
                             ADC_SQR1_NUM_CH(1),      \
                             0x00,                    \
                             ADC_SQR3_SQ1_N(16) };
// static void adcCallBack(ADCDriver *adcp, adcsample_t *buffer, size_t n);

struct usb_hid_in_report_s usb_hid_in_report;

const ina226_t ina[INA_SENSOR_COUNT] = {
#if defined(BOARD_BE_BT_BFK16)
    {{&I2CD1, 0x41, "0.9V CPU core"},	250,3},		//R003
    {{&I2CD1, 0x40, "1.5V CPU PLL"},	250,3},		//R003
    {{&I2CD1, 0x44, "1.8V CPU IO"},	20,30},		//R030 (655mA)
#elif defined(BOARD_BE_BT_BFK20)
    {{&I2CD1, 0x41, "0.90V CPU core"},	250,3},		//R003 2850mA
    {{&I2CD1, 0x46, "0.90V CPU PLL"},	 5,100},	//R100 50mA
    {{&I2CD1, 0x44, "1.80V CPU IO"},    10,30},		//R030
    {{&I2CD1, 0x40, "1.50V DDR"},		100,10},	//R010 380mA
    {{&I2CD1, 0x49, "1.80V DDR PLL"},	 5,100},	//R100 68mA
    {{&I2CD1, 0x45, "0.90V SATA"},		 5,100},	//R100 121mA
    {{&I2CD1, 0x4A, "1.80V SATA"},		 5,100},	//R100 38mA
    {{&I2CD1, 0x42, "0.95V PCIe"},		200,10},	//R010 585mA
    {{&I2CD1, 0x47, "1.50V PCIe"},		 5,100},	//R100 118mA
    {{&I2CD1, 0x43, "0.95V 10G"},		200,10},	//R010 635mA
    {{&I2CD1, 0x48, "1.50V 10G"},		 5,100},	//R100 115mA
#endif
};
// ---------- ---------- ---------- ---------- ---------- ----------
void cmd_start(BaseSequentialStream *chp, int argc, char *argv[])
{
	palTogglePad(GPIOB, 7);
	palTogglePad(GPIOC, 3);
	palTogglePad(GPIOC, 13);

	chThdSleepMilliseconds(20);
	palTogglePad(GPIOC, 4);
	palTogglePad(GPIOC, 5);

	chThdSleepMilliseconds(100);
	palTogglePad(GPIOD, 0);

	chThdSleepMilliseconds(1000);
	palTogglePad(GPIOD, 2);
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint16_t getTH02Value(BaseSequentialStream *chp, mesurment_t value, I2CDriver* i2cp)
{
    uint8_t statusReg = 0x00;
    uint8_t dataHReg  = 0x01;
    uint8_t buf[3]    = {0x00, 0x00, 0x00};
    uint8_t shifter   = 0x00;
    uint8_t devider   = 0x00;
    uint8_t subtrac   = 0x00;
    char    strRes[20];

    uint8_t  startSequence[2];
    if (value == HUM)
    {
    	startSequence[0] = TH02_REG_CONFIG;
    	startSequence[1] = TH02_HUM_CONV;
    	shifter          = 4;
        devider          = 16;
        subtrac          = 24;
    	strcpy(strRes, "humidity: %d\n\r");
    }
    else
    {
    	startSequence[0] = TH02_REG_CONFIG;
    	startSequence[1] = TH02_TEMP_CONV;
    	shifter          = 2;
        devider          = 32;
        subtrac          = 50;
    	strcpy(strRes, "temperature: %d\n\r");
    }

    // Starting measurement
	i2cMasterTransmitTimeout(i2cp, TH02_DEV_ID_7, startSequence, 2, NULL, 0, 20);

	// Waiting measurement is finished (minimum measurement time is 18 milliseconds)
	// chThdSleepMilliseconds(35);
	i2cMasterTransmitTimeout(i2cp, TH02_DEV_ID_7, &statusReg, 1, buf, 1, 20);
	while ((buf[0] & 0x01) != 0x00)
	{
		i2cMasterTransmitTimeout(i2cp, TH02_DEV_ID_7, &statusReg, 1, buf, 1, 20);
	}

	// Get measurement value
	i2cMasterTransmitTimeout(i2cp, TH02_DEV_ID_7, &dataHReg, 1, buf, 2, 20);

	// Transform value from 2 bytes
	uint16_t valueTH02 = 0x0;
	valueTH02 |= buf[0];
	valueTH02  = valueTH02 << 8;
	valueTH02 |= buf[1];
	valueTH02  = valueTH02 >> shifter;

	valueTH02 = (valueTH02 / devider) - subtrac;

	chprintf(chp, strRes, valueTH02);

	return valueTH02;
}

// ---------- ---------- ---------- ---------- ---------- ----------
void cmd_temp(BaseSequentialStream *chp, int argc, char *argv[])
{
	(void)  argc;
	(char*) argv;
	getTH02Value(chp, TEMP, &I2CD2);
	getTH02Value(chp, HUM, &I2CD2);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void cmd_pac1720(BaseSequentialStream *chp, int argc, char *argv[])
{
	(void)  argc;
	(char*) argv;
	i2caddr_t vdd_0v95_3v3 = 0x2D;
	getProductId(&I2CD1, vdd_0v95_3v3);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void cmd_insideTemperature(BaseSequentialStream *chp, int argc, char *argv[])
{
	adcStartConversion(&ADCD1, &group, buf, 4);

	float Vsense = buf[0]/4096*1.21;
	float res = ((0.76 - Vsense) / 2.5) + 25;
	chprintf(chp,"Inside temperature: %f \r\n", res);
	//adcStopConversion(&ADCD1);
	asm("nop"); // that was test string
}

// ---------- ---------- ---------- ---------- ---------- ----------
//static void adcCallBack(ADCDriver *adcp, adcsample_t *buffer, size_t n)
//{
//	  (void) buffer; (void) n;
//	  /* Note, only in the ADC_COMPLETE state because the ADC driver fires an
//	     intermediate callback when the buffer is half full.*/
//	  if (adcp->state == ADC_COMPLETE) {
//	    adcsample_t avg_ch1, avg_ch2;
//
//	    /* Calculates the average values from the ADC samples.*/
//	    avg_ch1 = (samples[0] + samples[2] + samples[4] + samples[6]) / 4;
//	    avg_ch2 = (samples[1] + samples[3] + samples[5] + samples[7]) / 4;
//
//
//
//}

// ---------- ---------- ---------- ---------- ---------- ----------
const char* okz(unsigned t)
{
	return t?"fail":"ok";
}

// ---------- ---------- ---------- ---------- ---------- ----------
// Danger: could hang CPU on invalid address read
void cmd_gpio(BaseSequentialStream *chp, int argc, char *argv[])
{
	if( argc == 3 ) {
		int i = (argv[0][0]|0x20)-'a';
		if( i >= 4 ) {
			chprintf(chp,"Only A-D\r\n");
			return;
		}
		GPIO_TypeDef* addr = (GPIO_TypeDef*)(GPIOA_BASE+i*(GPIOB_BASE-GPIOA_BASE));
		unsigned mask = strtol(argv[1], NULL, 16);
		unsigned val  = strtol(argv[2], NULL, 16);
		chSysLock();
		addr->ODR = (((addr->ODR)&~mask)|(val&mask));
		chSysUnlock();
	} else if (argc==0) {
		for( int i=0; i < 4; ++i) {
			//GPIOA->IDR == 0x40020010
			//GPIOA->ODR == 0x40020014
			//+0x400
			GPIO_TypeDef* addr = (GPIO_TypeDef*)(GPIOA_BASE+i*(GPIOB_BASE-GPIOA_BASE));
			chprintf(chp,"GPIO%c IDR %04X ODR %04X\r\n", 'A'+i, addr->IDR, addr->ODR);
		}
		unsigned sb;
		sb = smbBadBus(1);
		chprintf(chp,"I2C1 SCL:%s SDA:%s\r\n",okz(sb&1), okz(sb&2));
		sb = smbBadBus(2);
		chprintf(chp,"I2C2 SCL:%s SDA:%s\r\n",okz(sb&1), okz(sb&2));
	} else {
		chprintf(chp, "Usage: gpio a|b|c|d mask value\r\n");
	}
}

/*
 * Use this command to control sequencer.
 */
void cmd_sqnr(BaseSequentialStream *chp, int argc, char *argv[])
{
    int print_help = 0;
    if(argc) {
        switch ( isTrue(argv[0]) ) {
          case 1:
            if ( !sequencer_state ) {
              chprintf(chp, "Turn Sequencer on.\r\n");
              sequencer_ctl(1);
            }
            return;
          case 0:
            if(sequencer_state) {
              chprintf(chp, "Turn Sequencer off.\r\n");
              sequencer_ctl(0);
            }
            return;
          default:
            if( !strcmp(argv[0],"hreset") && argc==1 ) { // reset
				chSysLock();
				palClearPad(SEQ_nRESET_PORT,SEQ_nRESET_PIN);
				sequencer_ctl(0);
				// required delay 2uS
				chSysPolledDelayX(US2RTC(STM32_HCLK,20));
				palSetPad(SEQ_nRESET_PORT,SEQ_nRESET_PIN);
				chSysUnlock();
				chprintf(chp,"done\r\n");
            } else if( !strcmp(argv[0],"sreset") && argc==1 ) { // reset
				bool ok = smbWrite(0,I2C_ADDR_SQNR, UCD90_CMD_SOFT_RESET, 0,0);
				chprintf(chp,"%s\r\n",ok ? "done":"fail");
#ifdef BOARD_BE_BT_BFK20
            } else if( !strcmp(argv[0],"factory_config") && argc==1 ) {
				if ( ucd9090_config(chp) && ucd9090_init_pageinfo() ) chprintf(chp,"done\r\n");
				else chprintf(chp, "fail\r\n");
#endif
            } else if( !strcmp(argv[0],"cf") && argc==1 ) { // clear faults
                if( !ucd9090_clear_log(I2C_ADDR_SQNR) ) goto err;
                if( !smbWrite(0,I2C_ADDR_SQNR, PMBUS_CMD_CLEAR_FAULTS,0,0) ) goto err;
            } else if( !strcmp(argv[0],"gpio") && argc==3 ) { // GPIO control
				int index = ucd_get_page(argv[1]);
				if( index < 0 ) goto errarg;
				unsigned state = 0;
				switch ( argv[2][0] | 0x20 ) {
					case '0': state = (0<<2)|3; break;	// 0 Active-Driven
					case '1': state = (1<<2)|3; break;	// 1 Active-Driven
					case 'l': state = (0<<2)|3; break;	// 0 OpenDrain
					case 'h':
					case 'z': state = (1<<2)|1; break;	// 1 OpenDrain
					default: goto errarg;
				}
                if( !smbWrite(0,I2C_ADDR_SQNR, UCD90_CMD_GPIO_SELECT,index,1) ) goto err;
                if( !smbWrite(0,I2C_ADDR_SQNR, UCD90_CMD_GPIO_CONFIG,state,1) ) goto err;
                if( !smbWrite(0,I2C_ADDR_SQNR, UCD90_CMD_GPIO_CONFIG,state&~1,1) ) goto err;
                return;
            } else if( !strcmp(argv[0],"gpo_direct") && argc==3 ) { // GPO direct control configuration
				int gpo_index = ucd_get_page(argv[1]);
				if( gpo_index < 0 ) goto errarg;
				unsigned state = 0;
				switch ( argv[2][0] | 0x20 ) {
					case '0': state = (1<<2)|2; break;	// Active-High Active-Driven
					case '1': state = (0<<2)|2; break;	// Active-Low Active-Driven
					case 'l': state = (1<<2)|3; break;	// Active-Low OpenDrain
					case 'h': state = (0<<2)|3; break;	// Active-High OpenDrain
					case 'i': state = (0<<2)|1; break;	// Input
					case 'u': state = 0; break;	// Unused
					default: goto errarg;
				}
				uint8_t iobuf[0x15+2];
                if( !smbWrite(0,I2C_ADDR_SQNR, UCD90_CMD_GPO_CONFIG_INDEX,gpo_index,1) )  goto err;
                iobuf[0] = UCD90_CMD_GPO_CONFIG;
                iobuf[1] = 0x15;
                memset(iobuf+2,0,0x15);
                iobuf[2] = state; // pin config
                if( !smbWriteBlock(0,I2C_ADDR_SQNR, iobuf) ) goto err;
                return;
            } else if( !strcmp(argv[0],"log") && argc==1 ) {    // read log
                uint8_t iobuf[16];
                memset(iobuf,0xFF,16);
                if( !smbReadBlock(0,I2C_ADDR_SQNR,UCD9090_CMD_LOGGED_FAULTS,iobuf,UCD9090_CMD_LOGGED_FAULTS_LENGTH) )
					goto err;
                for(int i=0;i<UCD9090_CMD_LOGGED_FAULTS_LENGTH;++i) {
                    unsigned flags = iobuf[1+i]&0xFF;
                    if( flags == 0 ) continue;
                    const char* const* m_flags = (i==0?ucd9090_common_faults:(i==1?ucd9090_gpi_faults:ucd9090_page_faults));
                    chprintf(chp,m_flags[8],i-2);
                    for(int j=0;j<8;j++ ) {
                        if( flags & (1<<j) ) chprintf(chp," %s", m_flags[j]);
                    }
                    //chprintf(chp," VS=%d\r\n",ucd9090_pageinfo[i].vout_mode);
                    chprintf(chp,"\r\n");
                }
                uint16_t logindex = smbRead(0,I2C_ADDR_SQNR, UCD9090_CMD_LOGGED_FAULT_DETAIL_INDEX,2);
                if ( logindex ) {
                    uint16_t maxlog = logindex >> 8;
                    chprintf(chp,"Log index: %04X\r\n",logindex);
                    for(int i=0; i < maxlog; ++i) {
                        if( !smbWrite(0,I2C_ADDR_SQNR, UCD9090_CMD_LOGGED_FAULT_DETAIL_INDEX, i, 2) ) {
							chprintf(chp, "Communication error\r\n");
							return;
						}
                        //chThdSleepMilliseconds(1);
                        //logindex = smbRead(0,I2C_ADDR_SQNR, UCD9090_CMD_LOGGED_FAULT_DETAIL_INDEX,2);
                        //chprintf(chp,"Log index: %04X\r\n",logindex);
                        //chThdSleepMilliseconds(1);
                        if( !smbReadBlock(0,I2C_ADDR_SQNR, UCD9090_CMD_LOGGED_FAULT_DETAIL, iobuf, UCD9090_CMD_LOGGED_FAULT_DETAIL_LENGTH) )
							goto err;
                        //dumphex(chp,iobuf,UCD9090_CMD_LOGGED_FAULT_DETAIL_LENGTH+1);
                        uint32_t ms=smbGetData_be(iobuf+1,4);
                        uint32_t id_days=smbGetData_be(iobuf+1+4,4);
                        bool fault_ispaged = id_days >> 31;
                        uint16_t fault_page = (id_days >> 23) & 0xF;
                        uint16_t fault_type = (id_days >> 27) & 0xF;
                        id_days &= 0x7FFFFF;
                        uint16_t value=smbGetData_le(iobuf+1+4+4,4);
                        con_str_buf[0] = 0;
                        if( fault_ispaged && fault_type <= 2) {
                            linear_to_str(con_str_buf, value, ucd9090_pageinfo[fault_page].vout_mode);
                        } else if( fault_ispaged && fault_type >= 3 && fault_type <= 5) {
                            l11_to_str(con_str_buf,value);
                        }
                        chprintf(chp,"%8d ms, %d days paged:%d page=%d type=%d %7s %04X %s\r\n",ms, id_days,
							fault_ispaged, fault_page, fault_type,
							fault_ispaged && fault_type < sizeof(ucd9090_fault_types_paged)/sizeof(ucd9090_fault_types_paged[0])
								? ucd9090_fault_types_paged[fault_type] : "",
                            value, con_str_buf);
                    }
                } else {
                    chprintf(chp,"No log entries\r\n");
            	}
            } else if ( argv[0][0] == '?' || !cmd_smb_int(chp, I2C_ADDR_SQNR, argc,argv) ) {
                print_help = 1;
                break;
            }
            return;
        }
    }
    if(print_help) {
        chprintf(chp, " Usage: sqnr (0|off|1|on|log|cf)?\r\n");
        chprintf(chp, "    0 (off)  turn sequencer off\r\n");
        chprintf(chp, "    1 (on)   turn sequencer on\r\n");
        chprintf(chp, "    log      show log entries\r\n");
        chprintf(chp, "    cf       clear faults and log\r\n");
        chprintf(chp, "    hreset   toggle hardware reset pin\r\n");
        chprintf(chp, "    sreset   issue software reset\r\n");
    } else {
		if( !smbWrite(0,I2C_ADDR_SQNR, PMBUS_CMD_PAGE, 0, 1) )
			goto err;
        chprintf(chp, "Sequencer is %s\r\n", sequencer_status());
        uint32_t st = smbRead(0,I2C_ADDR_SQNR, PMBUS_CMD_STATUS_WORD, 2);
        if(st&~0xFFFFul) goto err;
        chprintf(chp, "Sequencer status %x\r\n", st);
        uint16_t n = st;
        while( n ) {
            chprintf(chp, "  %s\r\n", pmbus_status_msg(&n));
        }
        if ( st & PMBUS_ST_MFR ) {
            uint32_t mfst = ucd9090_mfr_status(I2C_ADDR_SQNR);
            chprintf(chp, "Sequencer MFR status %x\r\n", mfst);
            if ( mfst & 0x00FF0000 ) {
                chprintf(chp, "  GPI faults %x\r\n", (mfst>>16) & 0xFF);
            }
            mfst &= ~UCD9090_MFR_STATUS_UNUSED_BITS;
            while( mfst ) {
                chprintf(chp, "  %s\r\n", ucd9090_mfr_status_msg(&mfst));
            }
        }
        unsigned pages = smbRead(0,I2C_ADDR_SQNR, UCD90_CMD_NUM_PAGES, 1);
        if(pages > 11) return; // error
        for (n=0; n < pages; n++) {
            //TODO: page peaks LOGGED_PAGE_PEAKS voltage
			if( !smbWrite(0,I2C_ADDR_SQNR, PMBUS_CMD_PAGE, n, 1) ) {
				return;
			}
            uint16_t val = smbRead(0,I2C_ADDR_SQNR, PMBUS_CMD_READ_VOUT,2);
            linear_to_str(con_str_buf, val, ucd9090_pageinfo[n].vout_mode);
            uint8_t vs  = smbRead(0,I2C_ADDR_SQNR, PMBUS_CMD_STATUS_VOUT,1);
#if 1
            chprintf(chp, "Page#%02d  %10s V=%s", n, ucd9090_rails[n], con_str_buf);
#else
			// With ENABLE state
			smbWrite(0,I2C_ADDR_SQNR, UCD90_CMD_GPIO_SELECT,ucd9090_rails_en[n],1);
			unsigned s = smbRead(0,I2C_ADDR_SQNR, UCD90_CMD_GPIO_CONFIG,1);
            chprintf(chp, "Page#%02d E=%x %10s V=%s", n, ((s>>3)&1)|((s>>1)&s&(s<<1)&2), ucd9090_rails[n], con_str_buf);
#endif
            while( vs ) {
				const char* sm = pmbus_status_sub(PMBUS_CMD_STATUS_VOUT, &vs);
				chprintf(chp, " %s%s", sm, vs==0?"":",");
            }
			chprintf(chp, "\r\n");

        }
        uint16_t val = smbRead(0,I2C_ADDR_SQNR, PMBUS_CMD_READ_TEMPERATURE_1,2);
        l11_to_str(con_str_buf,val);
        chprintf(chp, " TEMP: %sC\r\n", con_str_buf);
    }
	return;
err:
	chprintf(chp, "Sequencer Communication Error\r\n");
	return;
errarg:
	chprintf(chp, "Invalid argument\r\n");
	return;
}

// ---------- ---------- ---------- ---------- ---------- ----------
void cmd_flash_id(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argv;
    if (argc > 0)
    {
        chprintf(chp, " Usage: flash_id\r\n");
        return;
    }
    if ( !system_power_good_state())
    {
        chprintf(chp, "Warning: power is off\r\n");
        return;
    }
    if (dfu_active|pwc_locked)
    {
        chprintf(chp, "Locked\r\n");
		return;
    }
}
// ---------- ---------- ---------- ---------- ---------- ----------
void cmd_atx(BaseSequentialStream *chp, int argc, char *argv[]) {
    if(argc) {
        if (dfu_active|pwc_locked) {
            chprintf(chp,"DFU programming in progress, power control disabled\r\n");
            return;
        }
        if ( !atx_present ) {
            chprintf(chp, "Warning: ATX power supply not detected\r\n");
        }
        switch ( isTrue(argv[0]) ) {
          case 1:
            chprintf(chp, "Turn ATX Power on\r\n");
            ATX_POWER_ON();
            int i;
            for (i=0; i < 10 && !ATX_POWER_GOOD_STATE(); i++) {
              chThdSleepMilliseconds(50);
            }
          break;
          case 0:
            chprintf(chp, "Turn ATX Power off\r\n");
            ATX_POWER_OFF();
            chThdSleepMilliseconds(50);
          break;
        }
    }
    chprintf(chp, "ATX Power ON pin is %s\r\n", ATX_POWER_ON_STATE() ? "on":"off");
    chprintf(chp, "ATX Power OK pin is %s\r\n", ATX_POWER_GOOD_STATE() ? "on":"off");
    powerctl(powerctl_none);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void cmd_vtg(BaseSequentialStream *chp, int argc, char *argv[]) {
    int32_t val;
    if(argc >= 2) {
		const zl2102_dev_t* cur_vtg = 0;
        for(unsigned i = 0; i < (sizeof(vtg)/sizeof(vtg[0])); i++) {
            if(strcmp(argv[0], vtg[i].dev.name) == 0) {
                cur_vtg = &vtg[i];
                break;
            }
        }
        if(cur_vtg == 0) {
            chprintf(chp, "Unable to parse regulator name\r\n");
            return;
        }
        if( strcmp(argv[1], "store") == 0 || strcmp(argv[1], "save") == 0 ) {
            zl2102_save(cur_vtg);
        } else if(argc == 2 && ((argv[1][0] >= '0' && argv[1][0] <= '9') || argv[1][0] == '.')) {
			fixed_val_t f_val;
            scan_val(&f_val, argv[1]);
            chprintf(chp, "Got value is %d.%03d\r\n", f_val.val, f_val.mval);
            if((f_val.val > cur_vtg->max.val) || ((f_val.val == cur_vtg->max.val) && (f_val.mval > cur_vtg->max.mval))) {
                chprintf(chp, "Output voltage is limited to %i.%i in software\r\n", cur_vtg->max.val, cur_vtg->max.mval);
            } else {
                zl2102_setv(cur_vtg,&f_val);
            }
 		} else {
			cmd_smb_int(chp, cur_vtg->dev.addr|I2C_BUS2, argc-1,argv+1);
        }
    } else {
        if( pwc_state < 1 ) {
            chprintf(chp,"Not powered\r\n");
        } else {
            for(unsigned i = 0; i < (sizeof(vtg)/sizeof(vtg[0])); i++) {
                val = smbRead(0, vtg[i].dev.addr, ZL2102_STATUS_WORD, 2);
                chprintf(chp, "ZL2102 Sensor %s status %4x\r\n", vtg[i].dev.name,val&0xFFFF);
                uint16_t n = val;
                while( n ) {
                    chprintf(chp, "  %s\r\n", pmbus_status_msg(&n));
                }
                val = smbRead(0, vtg[i].dev.addr, ZL2102_READ_VIN, 2);
                l11_to_str(con_str_buf,val);
                chprintf(chp, "Vin=%sV", con_str_buf);
                val = smbRead(0, vtg[i].dev.addr, ZL2102_VOUT_COMMAND, 2);
                linear_to_str(con_str_buf, val, -13);
                chprintf(chp, " Vset=%sV", con_str_buf);
                val = smbRead(0, vtg[i].dev.addr, ZL2102_READ_VOUT, 2);
                linear_to_str(con_str_buf, val, -13);
                chprintf(chp, " Vout=%sV", con_str_buf);
                val = smbRead(0, vtg[i].dev.addr, ZL2102_READ_IOUT, 2);
                l11_to_str(con_str_buf,val);
                chprintf(chp, " Iout=%-6.6sA", con_str_buf);

                val = smbRead(0, vtg[i].dev.addr, PMBUS_CMD_VOUT_MAX, 2);
                linear_to_str(con_str_buf, val, -13);
                chprintf(chp, " Vmax=%sV", con_str_buf);

                val = smbRead(0, vtg[i].dev.addr, ZL2102_READ_TEMP, 2);
                l11_to_str(con_str_buf,val);
                chprintf(chp, " TEMP=%sC\r\n", con_str_buf);
            }
        }
    }
}
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_vreg(BaseSequentialStream *chp, int argc, char *argv[])
{
    uint32_t val;
    if(argc >= 2) {
		const vreg_t* cur_vreg=0;
        for(unsigned i = 0; i < (sizeof(vreg)/sizeof(vreg[0])); i++) {
            if( strcmp(argv[0], vreg[i].name) == 0) {
                cur_vreg = &vreg[i];
                break;
            }
        }
        if(cur_vreg == 0) {
            chprintf(chp, "Unable to parse regulator name\r\n");
            return;
        }
        if( strcmp(argv[1], "store") == 0 || strcmp(argv[1], "save") == 0 ) {
			if ( cur_vreg->store == 0 ) {
				chprintf(chp, "No store method\r\n");
			} else {
				cur_vreg->store(cur_vreg);
			}
        } else if(argc == 2 && argv[1][0] >= '0' && argv[1][0] <= '9' ) {
			fixed_val_t f_val;
            scan_val(&f_val, argv[1]);
            //chprintf(chp, "Got value is %d.%03d\r\n", f_val.val, f_val.mval);
            val = f_val.val*1000 + f_val.mval; // sign??
            if( val > cur_vreg->max || val < cur_vreg->min ) {
                chprintf(chp, "Output voltage range violation\r\n");
            } else {
                vreg_data[cur_vreg-vreg] = val;
                if( ((cur_vreg->flags & VREG_FLAG_STNDBY) || (pwc_state==1 && (cur_vreg->flags & VREG_FLAG_PWRIN)) || pwc_state==2) && !cur_vreg->setmv(cur_vreg,val) ) {
					chprintf(chp, "Set error\r\n");
				}
            }
		} else {
			if( cur_vreg->addr == 0 ) {
				chprintf(chp, "Not a SMBus/PMBus device\r\n");
			} else {
				cmd_smb_int(chp, cur_vreg->addr, argc-1,argv+1);
			}
        }
    } else if (argc == 1 && (strcmp(argv[0], "store") == 0 || strcmp(argv[0], "save") == 0) ) {
        for(unsigned i = 0; i < (sizeof(vreg)/sizeof(vreg[0])); i++) {
			if ( vreg[i].store && ((vreg[i].flags & VREG_FLAG_STNDBY) || (pwc_state==1 && (vreg[i].flags & VREG_FLAG_PWRIN)) || pwc_state==2))
				vreg[i].store(&vreg[i]);
		}
    } else if (argc == 1 && (strcmp(argv[0], "list") == 0 )) {
        for(unsigned i = 0; i < (sizeof(vreg)/sizeof(vreg[0])); i++) {
			chprintf(chp, "%s\r\n", vreg[i].name);
		}
    } else {
		for(unsigned i = 0; i < (sizeof(vreg)/sizeof(vreg[0])); i++) {
			if ( vreg[i].show && ((vreg[i].flags & VREG_FLAG_STNDBY) || (pwc_state==1 && (vreg[i].flags & VREG_FLAG_PWRIN)) || pwc_state==2))
				vreg[i].show(&vreg[i],chp);
			else {
				char ext=' ';
				uint32_t v = vreg_data[i];
				if( v == 0 ) { ext='*'; v = vreg[i].nom; }
				chprintf(chp,"%-7s Vset=%d.%03d%c\r\n", vreg[i].name, v/1000, v%1000, ext);
			}
		}
    }
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_smb(BaseSequentialStream *chp, int argc, char *argv[])
{
	if ( !strcmp(argv[0], "silent") ) {
		smb_silent = isTrue(argv[1]);
		return;
	}
    char* end;
    uint16_t saddr = strtol(argv[0], &end, 16);
    if (end != argv[0] && (saddr&0xFF) !=0) {
		cmd_smb_int(chp, saddr, argc-1, argv+1);
		return;
	}
	chprintf(chp,"Usage: smb devaddr pr|pw|r|rw|rd|rx|rn|w command [data0 data1 ...]\r\n");
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_l11(BaseSequentialStream *chp, int argc, char *argv[])
{
	if( argc < 1 ) return;
	if( strchr(argv[0],'.') != NULL ) {
		fixed_val_t f_val;
		scan_val(&f_val,argv[0]);
		unsigned val = fv_l11(&f_val);
		l11_to_str(con_str_buf,val);
		chprintf(chp,"%04x %s\r\n", val, con_str_buf);
	} else {
		uint16_t val = strtol(argv[0], NULL, 16);
		l11_to_str(con_str_buf,val);
		chprintf(chp,"%04x %s\r\n", val, con_str_buf);
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_ascii(BaseSequentialStream *chp, int argc, char *argv[])
{
	for( int i=0; i < argc; i++) {
		unsigned char val = strtol(argv[i], NULL, 16);
		chprintf(chp,"%c", val<=' '|| val & 0x80 ? '.' : val);
	}
	chprintf(chp,"\r\n");
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_factory(BaseSequentialStream *chp, int argc, char *argv[])
{
    size_t i;
    if( argc == 0 ) {
        chprintf(chp,"Usage: factory_reset hard|soft\r\n");
        return;
    }
    if( powerctl(powerctl_in) ) {
        chprintf(chp,"Power control locked\r\n");
        return;
	}
	i = 20;
	do {
		chThdSleepMilliseconds(50);
	} while ( pwc_state != 1 && i-- );
	if( i == 0 ) {
		chprintf(chp, "Fail to turn ON power\r\n");
		return;
	}
    chprintf(chp, "Setting default configuration\r\n");
#ifdef BOARD_BE_BT_BFK20
    smbWrite(0,I2C_ADDR_SQNR,UCD90_CMD_SOFT_RESET, 0, 0);		// RESTORE_DEFAULT_ALL fails?
    chThdSleepMilliseconds(500);
    smbWrite(0,I2C_ADDR_SQNR,PMBUS_CMD_PAGE, 0xff, 1);
    smbWrite(0,I2C_ADDR_SQNR,PMBUS_CMD_ON_OFF_CONFIG, PMBUS_OOC_CONTROLLED|PMBUS_OOC_PIN, 1);
    smbWrite(0,I2C_ADDR_SQNR,PMBUS_CMD_PAGE, 7, 1);				// 3.3V
    smbWrite(0,I2C_ADDR_SQNR,PMBUS_CMD_VOUT_SCALE_MONITOR, 0xB26D, 2);		// 2/(2+1.3) = 0.606
    smbWrite(0,I2C_ADDR_SQNR,PMBUS_CMD_ON_OFF_CONFIG, 0, 1);	// always-on
    smbWrite(0,I2C_ADDR_SQNR,PMBUS_CMD_STORE_DEFAULT_ALL, 0, 0);	//
    chThdSleepMilliseconds(500);
    //reset_config: b6 - reset release delay
	//sqnr w d2 09 03 FF 03 4F 00 06 00 00 03
#endif
    for(i = 0; i < (sizeof(vtg)/sizeof(vtg[0])); i++) {
        smbWrite(0, vtg[i].dev.addr, ZL2102_RESTORE_FACTORY, 0,0);
        chThdSleepMilliseconds(15);
        int32_t val = fv_l16u(&vtg[i].max,-13);
        smbWrite(0, vtg[i].dev.addr, ZL2102_VOUT_MAX, val, 2);                      // MUST write for correct operation
        smbWrite(0, vtg[i].dev.addr, ZL2102_USER_CONFIG, 0x0001, 2);                // SYNC
        smbWrite(0, vtg[i].dev.addr, ZL2102_DDC_CONFIG, 0x20|vtg[i].dev.addr, 2);   // Inhibit DDC
        smbWrite(0, vtg[i].dev.addr, ZL2102_MFR_IOUT_OC_FAULT_RESPONSE, 0x80, 1);   // Shutdown
        smbWrite(0, vtg[i].dev.addr, PMBUS_CMD_VOUT_OV_FAULT_RESPONSE, 0x80, 1);    // Shutdown
        smbWrite(0, vtg[i].dev.addr, PMBUS_CMD_IOUT_OC_FAULT_LIMIT, 0xD200, 2);   	// 8.0A
        if ( argc == 1 && !strcmp(argv[0],"soft") ) {
          zl2102_setv(&vtg[i],&vtg[i].nominal);
          smbWrite(0, vtg[i].dev.addr, PMBUS_CMD_UT_WARN_LIMIT,  (  0*8 & 0x7FF)|(-3<<11), 2);
          smbWrite(0, vtg[i].dev.addr, PMBUS_CMD_UT_FAULT_LIMIT, (-10*8 & 0x7FF)|(-3<<11), 2);
          smbWrite(0, vtg[i].dev.addr, PMBUS_CMD_OT_WARN_LIMIT,  ( 85*8 & 0x7FF)|(-3<<11), 2);
          smbWrite(0, vtg[i].dev.addr, PMBUS_CMD_OT_FAULT_LIMIT, (100*8 & 0x7FF)|(-3<<11), 2);
        }
        smbWrite(0, vtg[i].dev.addr, ZL2102_ON_OFF_CFG, 0x17, 1);                   // not restored by RESTORE_FACTORY
        zl2102_save(&vtg[i]);
        chThdSleepMilliseconds(10);
    }
    powerctl(powerctl_off);
    //NOTE: tps56 settings is not permanent, should be done each time 12V is ON
    //tps56_set_cV(&tps56_xgb, (tps56_xgb.nominal.val*1000 + tps56_xgb.nominal.mval) / 10);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_bootcfg(BaseSequentialStream *chp, int argc, char *argv[])
{
    if(argc) {
        switch (isTrue(argv[0])) {
			case 'f':	// FLASH
            case 0:
                palClearPad(MIPS_nBOOTCFG1_PORT, MIPS_nBOOTCFG1_PIN);
                break;
            case 'b':
            case 'r':	// BROM
            case 1:
                palSetPad(MIPS_nBOOTCFG1_PORT, MIPS_nBOOTCFG1_PIN);
                break;
            default:
                chprintf(chp, "BOOTCFG: %d\r\n", palReadPad(MIPS_nBOOTCFG1_PORT, MIPS_nBOOTCFG1_PIN) != 0);
        }
    } else {
		chprintf(chp, "bootcfg 0|flash|1|brom|status\r\n");
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_cpureset(BaseSequentialStream *chp, int argc, char *argv[])
{
    if(argc) {
        switch (isTrue(argv[0])) {
            case 0:
            case 'a':	// assert
                MIPS_RESET_ASSERT();
                break;
            case 1:
            case 'd':	// deassert
            case 'r':	// release
                MIPS_RESET_RELEASE();
                break;
            case 'c': {	// cycle
                int cycles=1;
                if(argc>=2) cycles=strtol(argv[1],NULL,0);
                int delay=100;
                if(argc>=3) delay=strtol(argv[2],NULL,0);
                if( delay<=0 || delay > 60000 ) delay = 100;
                chprintf(chp,"Baikal reset cycle: %d delay %dms\r\n",cycles,delay);
                for(;cycles>0;--cycles) {
					MIPS_RESET_ASSERT();
                    chThdSleepMilliseconds(20);
					MIPS_RESET_RELEASE();
                    chThdSleepMilliseconds(delay);
                }
                } break;
            default:
                argc = 0;
                break;
        }
    }
    if(argc==0){
        chprintf(chp, "CPURST#: %d\r\n", palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN) != 0);
    }
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_dfu(BaseSequentialStream *chp, int argc, char *argv[])
{
    int print_help = 0;
    if(argc) {
        if(strcmp(argv[0], "wa") == 0) {
            if(argc > 1) {
                DFU1.write_address_start = strtol(argv[1], NULL, 0);
            }
            chprintf(chp, "Current write address is 0x%x.\r\n", DFU1.write_address_start);
        } else if(strcmp(argv[0], "ra") == 0) {
            if(argc > 1) {
                DFU1.read_address_start = strtol(argv[1], NULL, 0);
            }
            chprintf(chp, "Current read address is 0x%x.\r\n", DFU1.read_address_start);
        } else {
            print_help = 1;
        }
    } else {
        print_help = 1;
    }
    if(print_help) {
        chprintf(chp, " Control start read/write address of DFU. \r\n");
        chprintf(chp, " Usage: dfu (wa|ra) (0xaddr)?\r\n");
        chprintf(chp, "    wa       display current start write address, if address is provided parse it and set to wa\r\n");
        chprintf(chp, "    ra       display current start read address, if address is provided parse it and set to ra\r\n");
    }
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_sensors(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argv;
    int print_help = 0;
    unsigned i;
    if(argc) {
        print_help = 1;
    } else {

#if HID_REPORT_VTG
        chprintf(chp, "   ZL2102 voltage regulators\r\n");
        for(i = 0; i < (sizeof(vtg)/sizeof(vtg[0])); i++) {
            l11_to_str(con_str_buf,usb_hid_in_report.report.vtg_i[i]);
            chprintf(chp, "Voltage command #%i: %sV\r\n", i, con_str_buf);
        }
        chprintf(chp, "\r\n");
#endif

		if ( pwc_state >= 1 ) {
#ifdef BOARD_BE_BT_BFK16
			chprintf(chp, "   TPS56 voltage regulator\r\n");
			uint32_t tv = tps56_dump(&tps);
			unsigned dv = ((tv >> 24) & 0x7F) + 60;
			if ( tv & (TPS56_INTERNAL<<16) ) {
				chprintf(chp,"Vset=%d.%d", dv/100,dv%100);
			} else {
				chprintf(chp,"Vset=EXT");
			}
			chprintf(chp," PG=%x",tv&0x7);
			//chprintf(chp," state=%08x", tv);
			chprintf(chp, "\r\n\r\n");
#endif
#ifdef BOARD_BE_BT_BFK20
			chprintf(chp, "TPS65 status %x\r\n", smbRead(0,I2C_ADDR_TPS65263,TPS65263_SYS_STATUS,1));
#endif
		}

        chprintf(chp, "   INA266 current sensors data\r\n");
#if HID_REPORT_STAT
        chprintf(chp, "               actual     max        min        average\r\n");
#endif
        for(i = 0; i < (unsigned)min(HID_REPORT_SENSOR_CHANNELS,(sizeof(ina)/sizeof(ina[0]))); i++) {
            chprintf(chp, "Current #%i: %4d.%04dA ", i, usb_hid_in_report.report.ina[i].current.actual / 1000000, (abs(usb_hid_in_report.report.ina[i].current.actual) % 1000000)/100);
#if HID_REPORT_STAT
            chprintf(chp, "%4d.%04dA ", usb_hid_in_report.report.ina[i].current.max / 1000000, abs((usb_hid_in_report.report.ina[i].current.max) % 1000000)/100);
            chprintf(chp, "%4d.%04dA ", usb_hid_in_report.report.ina[i].current.min / 1000000, (abs(usb_hid_in_report.report.ina[i].current.min) % 1000000)/100);
            chprintf(chp, "%4d.%04dA\r\n", usb_hid_in_report.report.ina[i].current.avg / 1000000, (abs(usb_hid_in_report.report.ina[i].current.avg) % 1000000)/100);
#endif
            chprintf(chp, "Voltage #%i: %4d.%03dV ", i, usb_hid_in_report.report.ina[i].voltage.actual / 1000, usb_hid_in_report.report.ina[i].voltage.actual % 1000);
#if HID_REPORT_STAT
            chprintf(chp, "%5d.%03dV ", usb_hid_in_report.report.ina[i].voltage.max / 1000, usb_hid_in_report.report.ina[i].voltage.max % 1000);
            chprintf(chp, "%5d.%03dV ", usb_hid_in_report.report.ina[i].voltage.min / 1000, usb_hid_in_report.report.ina[i].voltage.min % 1000);
            chprintf(chp, "%5d.%03dV\r\n", usb_hid_in_report.report.ina[i].voltage.avg / 1000, usb_hid_in_report.report.ina[i].voltage.avg % 1000);
#endif
#if HID_REPORT_POWER
            chprintf(chp, "Power   #%i: %4d.%04dW ", i, usb_hid_in_report.report.ina[i].power.actual / 1000000, (usb_hid_in_report.report.ina[i].power.actual % 1000000)/100);
#endif
#if HID_REPORT_STAT
            chprintf(chp, "%4d.%04dW ", usb_hid_in_report.report.ina[i].power.max / 1000000, (usb_hid_in_report.report.ina[i].power.max % 1000000)/100);
            chprintf(chp, "%4d.%04dW ", usb_hid_in_report.report.ina[i].power.min / 1000000, (usb_hid_in_report.report.ina[i].power.min % 1000000)/100);
            chprintf(chp, "%4d.%04dW\r\n", usb_hid_in_report.report.ina[i].power.avg / 1000000, (usb_hid_in_report.report.ina[i].power.avg % 1000000)/100);
#endif
            chprintf(chp, "\r\n");
        }
    }
    if(print_help) {
        chprintf(chp, " Show long power monitor report. \r\n");
    }
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_power(BaseSequentialStream *chp, int argc, char *argv[])
{
    int print_help = 0;
    int i = 0;
    int pf = 0;

    if(argc) {
        if(dfu_active && strcmp(argv[0], "status")) {
            chprintf(chp,"DFU programming in progress, power control disabled\r\n");
            return;
        }
        if(!strcmp(argv[0], "on")) {
            pf=powerctl(powerctl_on);
        } else if (!strcmp(argv[0], "in")) {
            pf=powerctl(powerctl_in);
        } else if(strcmp(argv[0], "off") == 0) {
            pf=powerctl(powerctl_off);
        } else if(strcmp(argv[0], "cycle") == 0) {
            if(pwc_locked) {
                pf=1;
            } else if(pwc_state>=1) {
                int cycles=1;
                if(argc>=2) cycles=strtol(argv[1],NULL,0);
                int delay=600;
                if(argc>=3) delay=strtol(argv[2],NULL,0);
                if( delay<=0 || delay > 60000 ) delay = 100;
                int delay0=1000;
                if(argc>=4) delay0=strtol(argv[3],NULL,0);
                if( delay0<=0 || delay0 > 60000 ) delay0 = 1000;
                chprintf(chp,"Baikal power cycle: %d delay %dms:%dms\r\n",cycles,delay,delay0);
                for(int nc=1 ;nc<=cycles;++nc) {
					if( nc!=1 || pwc_state == 2 ) {		// first OFF only is ON
						//sequencer_ctl(0);
						powerctl(powerctl_in);
						for(i=200;i && palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN); --i ) {
							chThdSleepMilliseconds(10);
						}
						chThdSleepMilliseconds(delay0);
						if(i==0||pwc_state<0) {
							chprintf(chp,"OFF timeout %d\r\n",nc);
							return;
						}
					}
                    //sequencer_ctl(1);	// no CLK25 control inside
                    powerctl(powerctl_on);
                    for(i=1000;i && !palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN); --i ) {
                        chThdSleepMilliseconds(2);
                    }
                    if(i==0||sqnr_state()!=3) {
                        chprintf(chp,"ON timeout %d\r\n",nc);
                        return;
                    }
                    chThdSleepMilliseconds(delay);
                }
                //chprintf(chp, "Baikal power cycle %d delay %d done.\r\n",cycles,delay);
            } else {
                chprintf(chp, "ATX power good signal does not present, turn ATX power supply first.\r\n");
            }
        } else if(strcmp(argv[0], "status") == 0) {
            chprintf(chp, "ATX Power OK pin is %s\r\n", ATX_POWER_GOOD_STATE() ? "on":"off");
            chprintf(chp, "Sequencer  is %s\r\n", sequencer_status());
            chprintf(chp, "SystemOK   is %s\r\n", palReadPad(SYSTEM_OK_PORT,  SYSTEM_OK_PIN)?"on":"off");
            chprintf(chp, "CPU_RESET# is %s\r\n", palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN)?"1":"0");
            chprintf(chp, "I2C1_SMBA# is %s\r\n", palReadPad(I2C1_SMBA_PORT, I2C1_SMBA_PIN)?"1":"0");
            chprintf(chp, "I2C2_SMBA# is %s\r\n", palReadPad(I2C2_SMBA_PORT, I2C2_SMBA_PIN)?"1":"0");
#ifdef BOARD_BE_BT_BFK20
			if( pwc_state >= 1 ) {
				int tamb = smbRead(0, I2C_ADDR_MCP9804, 5, 2);
				tamb = ((tamb & 0xFF) << 8) | ((tamb>>8)&0xFF);
				if( tamb & 0x1000 ) tamb |= 0xF000;
				else tamb &= 0xFFF;
				chprintf(chp, "T board    is %2d.%02dC\r\n", tamb>>4, ((1000/8)*((tamb&0xF)>>1)/10));
				extern volatile int rtd_t;
				extern unsigned rtd_error;
				if( rtd_error == 0 ) {
					tamb = rtd_t>>1;
					chprintf(chp, "T RTD      is %2d.%02dC\r\n", tamb>>4, ((1000/8)*((tamb&0xF)>>1)/10));
				}
			}
#endif
            //chprintf(chp, "%d \r\n",pwc_state);
        } else {
            print_help = 1;
        }
    } else {
        print_help = 1;
    }
    if(pf) {
        chprintf(chp, "Power control disabled\r\n");
    }
    if(print_help) {
        chprintf(chp, " Control state of board power system. \r\n");
        chprintf(chp, " Usage: power (on|off|cycle|status)\r\n");
        chprintf(chp, "    in       input power on, board power off\r\n");
        chprintf(chp, "    on       power full on\r\n");
        chprintf(chp, "    off      board power off, input power off\r\n");
        chprintf(chp, "    cycle [N [on_time] [off_time]]]\r\n");
        chprintf(chp, "    status   show ATX and sequencer status\r\n");
    }
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void cmd_ina(BaseSequentialStream *chp, int argc, char *argv[])
{
    size_t i;
    int print_help = 0;

    if(argc == 0) {
        chprintf(chp,"- = INA226 power sensors = -\r\n");
        for(i = 0; i < sizeof(ina)/sizeof(ina[0]); i++) {
            ina_display(chp,i);
        }
    } else if(argc == 1) {
        if(strcmp(argv[0], "list") == 0) {
            for(i = 0; i < sizeof(ina)/sizeof(ina[0]); i++) {
                chprintf(chp, "%d - %s\r\n",i,ina[i].dev.name);
            }
        } else {
            i = argv[0][0] - '0';
            if(i < sizeof(ina)/sizeof(ina[0])) {
                ina_display(chp,i);
            }
            else {
                print_help = 1;
            }
        }
    } else {
        print_help = 1;
    }
    if(print_help) {
        chprintf(chp, " Use this command to read state of on board current sensors.\r\n");
        chprintf(chp, " Usage:  ina [<sensor>|list]?\r\n");
        chprintf(chp, "             without argument should print all currents.\r\n");
        chprintf(chp, "   list      will list available sensor.\r\n");
        chprintf(chp, "   <sensor>  print current of the sensor (should 0, 1 ...).\r\n");
    }
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
#ifdef BOARD_BE_BT_BFK16
	/*
	 *  Use this command to control/check state of the TPS56720 voltage regulator.
	 * format:
	 *   tps56 [x.xx]?
	 * - without argument should print list of available regulators
	 * - voltage should be in range from 0.60V to 1.87V
	 */
	void cmd_tps56(BaseSequentialStream *chp, int argc, char *argv[])
	{
		(void)argv;
		fixed_val_t f_val;
		if(argc == 1) {
			scan_val(&f_val, argv[0]);
			f_val.mval = f_val.mval/10;
			//chprintf(chp, "Got value %d.%02dV\r\n", f_val.val, f_val.mval);
			if(((f_val.val == 1) && (f_val.mval < 88)) || ((f_val.val == 0) && (f_val.mval > 59))) {
				chprintf(chp, "Going to set %d.%02dV\r\n", f_val.val, f_val.mval);
				tps_val[0] = f_val.val*100 + f_val.mval;
				tps56_set_cV(&tps, tps_val[0]);
			} else {
				chprintf(chp, "Output voltage should be in range [0.60; 1.87]\r\n");
			}
		} else {
			unsigned val = tps56_get_cV(&tps);
			chprintf(chp, "Read value is %d.%02dV\r\n", val/100, val%100);
		}
	}
#endif

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
#ifdef BOARD_BE_BT_BFK20
	bool tps_setmv(const vreg_t* vr, unsigned mv)
	{
		smbWrite(0, vr->addr, TPS65263_VOUT1_SEL+vr->id, TPS65263_VOUT_SET(mv/10-68,1), 1);
		return true;
	}

	// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
	bool tps_show(const vreg_t* vr, BaseSequentialStream* chp)
	{
		unsigned s = smbRead(0, vr->addr, TPS65263_VOUT1_SEL+vr->id, 1);
		if (s & 0xFF00) {
			chprintf(chp, "%s Error\r\n", vr->name);
			return false;
		}
		if ( !(s & 0x80) ) {
			chprintf(chp, "%s Vset=EXT\r\n", vr->name);
		} else {
			s = (s & 0x7F) + 68;
			chprintf(chp, "%s Vset=%d.%02d\r\n", vr->name, s/100,s%100);
		}
		return true;
	}

	// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
	bool vpll_setmv(const vreg_t* vr, unsigned mv)
	{
		(void)vr;
		switch( mv ) {
			case  900: mv = 0; break;
			case  950: mv = 1; break;
			case 1000: mv = 2; break;
			case 1050: mv = 3; break;
			default: { return false; }
		}
		palWriteGroup(GPIOC,3,1,~mv);
		return true;
	}

	// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
	bool vpll_show(const vreg_t* vr, BaseSequentialStream* chp)
	{
		unsigned mv=((~GPIOC->ODR >> 1) & 3)*5u+90;
		//unsigned mv = vreg_data[vr-vreg]*5+90;
		chprintf(chp, "%-7s Vset=%d.%02dV\r\n", vr->name, mv/100, mv%100);
		return true;
	}

	// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
	bool ltc_setmv(const vreg_t* vr, unsigned mv)
	{
		return smbWrite(0, vr->addr, (((mv-690)*10)/216)&0x3F,0,0);
	}
#endif

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
#ifdef BOARD_BE_BT_BFK16
	bool tps_setmv(const vreg_t* vr, unsigned mv)
	{
	/*
	 * Resolution: 0.01V
	 */
		unsigned cV = (mv/10 - 60) & 0x7F;
		// calc odd parity in bit.7
		unsigned par = (cV ^ (cV << 4));	// using native int - shorter code
		par = (par ^ (par << 2));
		par = ~(par ^ (par << 1)) & 0x80;
		smbWrite(0, vr->addr, TPS56_VOUT, cV|par, 1);
		return true;
	}
#endif

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
bool zl2102_show(const vreg_t* vreg, BaseSequentialStream* chp)
{
	uint32_t xs = smbRead(0,vreg->addr, PMBUS_CMD_STATUS_WORD, 2);
	chprintf(chp, "%-7s ", vreg->name);
	if ( xs & 0xFFFF0000 ) {
		chprintf(chp, "no Vin\r\n");
	}
	uint16_t val;
	val = smbRead(0,vreg->addr, PMBUS_CMD_READ_VIN, 2);
	l11_to_str(con_str_buf,val);
	chprintf(chp, "Vin=%sV", con_str_buf);
	val = smbRead(0,vreg->addr, PMBUS_CMD_VOUT_COMMAND, 2);
	linear_to_str(con_str_buf, val, ZL2102_VOUT_MODE_VALUE);
	chprintf(chp, " Vset=%sV", con_str_buf);
	val = smbRead(0,vreg->addr, PMBUS_CMD_READ_VOUT, 2);
	linear_to_str(con_str_buf, val, ZL2102_VOUT_MODE_VALUE);
	chprintf(chp, " Vout=%sV", con_str_buf);
	val = smbRead(0,vreg->addr, PMBUS_CMD_READ_IOUT, 2);
	l11_to_str(con_str_buf,(int16_t)val);
	chprintf(chp, " Iout=%-6.6sA", con_str_buf);
	//val = smbRead(0,vreg->addr, PMBUS_CMD_VOUT_MAX, 2);
	//linear_to_str(con_str_buf, val, ZL2102_VOUT_MODE_VALUE);
	//chprintf(chp, " Vmax=%sV", con_str_buf);
	val = smbRead(0,vreg->addr, PMBUS_CMD_READ_TEMPERATURE_1,2);
	l11_to_str(con_str_buf,(int16_t)val);
	chprintf(chp, " TEMP=%sC", con_str_buf);

	val = xs & 0xFFFF;
	chprintf(chp, " status %4x\r\n", xs & 0xFFFF);
	while( val ) {
		chprintf(chp, "  %s\r\n", pmbus_status_msg(&val));
	}
	return true;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
bool zl2102_store(const vreg_t* vreg)
{
    return smbWrite(0, vreg->addr, ZL2102_STORE_USER_ALL, 0, 0);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
bool zl2102_setmv(const vreg_t* vreg, unsigned mv)
{
    int32_t val = to_l16u(mv, -13);
    //smbWrite(vreg->addr, ZL2102_VOUT_MAX, (int16_t)(val * 110 / 100));
    smbWrite(0,vreg->addr, ZL2102_VOUT_OV_FAULT_LIMIT, val * 115 / 100,2);
    smbWrite(0,vreg->addr, ZL2102_VOUT_UV_FAULT_LIMIT, val * 85 / 100,2);
    smbWrite(0,vreg->addr, ZL2102_VOUT_COMMAND, val,2);
    return true;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
int isTrue(const char* str)
{
    if( !strcmp("1",str) || !strcmp("on", str) ) { return 1; }
    if( !strcmp("0",str) || !strcmp("off",str) ) { return 0; }
    return *str ? (*str&0xFF) : -1;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void sequencer_ctl(int st)
{
    if(st)
    {
        palClearPad(SEQ_CNTRL_PORT,SEQ_CNTRL_PIN);
        sequencer_state = 1;
    } else {
        palSetPad(SEQ_CNTRL_PORT,SEQ_CNTRL_PIN);
        sequencer_state = 0;
    }
}

// ---------- ---------- ---------- ---------- ---------- ----------
bool sequencer_alert(void)
{
    //return !palReadPad(I2C1_SMBA_PORT, I2C2_SMBA_PIN);    // now DNP
    return false;
}

// ---------- ---------- ---------- ---------- ---------- ----------
const char* sequencer_status(void)
{
    return !palReadPad(SEQ_CNTRL_PORT,SEQ_CNTRL_PIN) ? (sequencer_alert() ? "alert" : "on") : "off";
}

// ---------- ---------- ---------- ---------- ---------- ----------
unsigned system_power_good_state(void) {
    return (!atx_present||atx_state) && sequencer_state && (pwc_state == 2);
}

// ---------- ---------- ---------- ---------- ---------- ----------
unsigned sqnr_state(void)
{
	#if defined(BOARD_BE_BT_BFK20) || defined(BOARD_BE_BT_NUCLEO_207ZG)
		return ((!palReadPad(SEQ_CNTRL_PORT,SEQ_CNTRL_PIN)) | (palReadPad(SYSTEM_OK_PORT,SYSTEM_OK_PIN)<<1));
	#else
		return ((!palReadPad(SEQ_CNTRL_PORT,SEQ_CNTRL_PIN)) | ((!palReadPad(MIPS_RESETn_PORT, MIPS_RESETn_PIN))<<1));

		#warning "Please, re-program UCD"
	#endif
}

// ---------- ---------- ---------- ---------- ---------- ----------
bool ucd9090_init_pageinfo(void)
{
    for(int page=0; page < UCD9090_PAGES; ++page)
    {
        if( !smbWrite(0,I2C_ADDR_SQNR,PMBUS_CMD_PAGE,page,1) ) return false;
        uint32_t val = smbRead(0,I2C_ADDR_SQNR,PMBUS_CMD_VOUT_MODE,1);
        if( val & ~0xFFul ) return false;
        ucd9090_pageinfo[page].vout_mode  = smbRead(0,I2C_ADDR_SQNR,PMBUS_CMD_VOUT_MODE,1);
        if( ucd9090_pageinfo[page].vout_mode & 0x10 ) ucd9090_pageinfo[page].vout_mode |= 0xF0;
    }
    return true;
}

// ---------- ---------- ---------- ---------- ---------- ----------
int ucd_get_page(const char* str)
{
	char* end;
	unsigned gpio = strtol(str,&end,0);
	if( *end=='p' || *end=='#' ) {
		if( gpio >= UCD9090_PAGES ) return -1;
		gpio = ucd9090_rails_en[gpio];
	}
	if( gpio > UCD9090_MAX_GPIO_ID ) return -1;
	return gpio;
}

// ---------- ---------- ---------- ---------- ---------- ----------
void zl2102_setv(const zl2102_dev_t* vtg, const fixed_val_t* f_val)
{
    int32_t val = fv_l16u(f_val, -13);
    //smbWrite(0,vtg->dev.addr, ZL2102_VOUT_MAX, (int16_t)(val * 110 / 100), 2);
    smbWrite(0,vtg->dev.addr, ZL2102_VOUT_OV_FAULT_LIMIT, (int16_t)(val * 115 / 100), 2);
    smbWrite(0,vtg->dev.addr, ZL2102_VOUT_UV_FAULT_LIMIT, (int16_t)(val * 85 / 100), 2);
    smbWrite(0,vtg->dev.addr, ZL2102_VOUT_COMMAND, (int16_t)val, 2);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void zl2102_save(const zl2102_dev_t* vtg)
{
    smbWrite(0,vtg->dev.addr, ZL2102_STORE_USER_ALL, 0,0);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void restore_vreg_vout(void)
{
	for(size_t i = 0; i < (sizeof(vreg)/sizeof(vreg[0])); i++) {
		if( vreg_data[i] != 0 ) {
			vreg[i].setmv(&vreg[i],vreg_data[i]);
		}
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void ina_display(BaseSequentialStream *chp, size_t i)
{
	ina226_data d;
    if ( !ina226_read_data(&ina[i],&d,ina_all) ) {
		chprintf(chp, "INA%d(%s) read error\r\n",i,ina[i].dev.name);
		return;
	}
	#ifdef CHPRINTF_USE_FLOAT
		chprintf(chp, "INA%-2d %.3fV %8.4fA %.4fW - %s\r\n",i,
			d.voltage/1000.0, d.current/(double)RPT_I_SCALE, d.power/(double)RPT_P_SCALE,
			ina[i].dev.name);
	#else
		chprintf(chp, "INA%-2d %c%d.%03dV %*c%d.%04dA %2d.%04dW - %s\r\n",i,
			d.voltage < 0 ? '-':' ', abs(d.voltage) / 1000, abs(d.voltage) % 1000,
			(abs(d.current) < RPT_I_SCALE*10)?2:1, d.current < 0 ? '-':' ',
			d.current / RPT_I_SCALE, (abs(d.current) % RPT_I_SCALE)/100,
			d.power   / RPT_P_SCALE, (d.power % RPT_P_SCALE)/100,
			ina[i].dev.name);
	#endif
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
#if defined(BOARD_BE_BT_BFK20) && defined(BMC_WITH_SYSTEM_CONSOLE)
void cmd_clk156(BaseSequentialStream *chp, int argc, char *argv[])
{
  i2caddr_t addr = I2C_ADDR_SI5342;
  if( argc > 1 && (argv[0][0] == '3' || argv[0][0] == '1') ) {
	if( argv[0][0] == '1' ) {
		addr = (I2C_ADDR_SI5342 & 0xFF) | I2C_BUS1;
	}
	argc--; argv++;
  }

  if (SMB_ADDR_BUS_GET(addr) == 3 && I2CD3.state == I2C_STOP)
	i2cStart(&I2CD3, &i2cfg3);

  if( argc == 1 && !strcmp(argv[0], "init") ) {
	  if (!clk156_init(addr)) goto err;
  } else if( argc >= 1 && !strcmp(argv[0], "program_nvm") ) {
	  uint32_t freeBanks = clk156_free_banks(addr);
	  if ( freeBanks == 0 ) {
		  chprintf(chp,"No free NVM banks\r\n");
		  return;
	  }
	  if ( freeBanks == 1 && !(argc == 2 && argv[1][0]=='y') ) {
		  chprintf(chp,"Last NVM bank: confirm!\r\n");
		  return;
	  }
	  if (!clk156_program_nvm(addr)) goto err;
  } else if( argc == 1 && !strcmp(argv[0], "verify") ) {
	  unsigned freg;
	  if (!clk156_verify(addr, &freg)) {
		  chprintf(chp,"Verify failed: %x\r\n", freg);
		  return;
	  }
  } else if( argc == 3 && !strcmp(argv[0], "override") ) {
	  unsigned reg = strtol(argv[1], NULL, 16);
	  unsigned data = strtol(argv[2], NULL, 16);
	  if ( !clk156_set_override(reg, data) ) goto err;
  } else if( argc == 2 && !strcmp(argv[0], "read_nvm") ) {
	  unsigned bank = strtol(argv[1], NULL, 10);
	  if (!clk156_read_nvm_bank(addr, bank)) goto err;
  } else if( argc == 1 && !strcmp(argv[0], "check") ) {
	  if (!clk156_check(chp, addr)) goto err;
  } else if( argc == 1 && !strcmp(argv[0], "disable") ) {
	  if (!clk156_disable( addr)) goto err;
  } else if( argc == 1 && !strcmp(argv[0], "enable") ) {
	  if (!clk156_disable( addr)) goto err;
  } else {
	  chprintf(chp,"clk156 [1|3] init|read_nvm|program_nvm [yes]|verify|check|override reg value\r\n");
	  return;
  }
  chprintf(chp,"done\r\n");
  return;
err:
  chprintf(chp,"failed\r\n");
  return;
}

void cmd_clk25(BaseSequentialStream *chp, int argc, char *argv[]) {
  if( argc != 1 ) {
	  chprintf(chp, "clk init|status|enable|disable\r\n");
	  return;
  }
  if( !strcmp(argv[0], "init") ) {
	  if (!clk25_init()) {
		  chprintf(chp,"CLKEGN config failed\r\n");
	  }
  } else if( !strcmp(argv[0], "disable") ) {
	  	if ( !clk25_disable() ) {
			chprintf(chp, "No access, check J60\r\n");
		}
  } else if( !strcmp(argv[0], "enable") ) {
	  	if ( !clk25_enable() ) {
			chprintf(chp, "No access, check J60\r\n");
		}
  } else if( !strcmp(argv[0], "status") ) {
		uint32_t s = smbRead(0, I2C_ADDR_SI5351, 0, 4);
		if ( (s&0xFC000000)!=0xFC000000 ) {
			chprintf(chp, "unconfigured\r\n");
			return;
		}
		s &= 0x03A0A0A0;
		chprintf(chp,"Status %8x %s %s\r\n", s, (s&SI5351_SYS_INIT)?"INIT":(s&SI5351_LOL_A?"LOL":"Locked"),
		  s&0x03000000?"Disabled":"Enabled");
  }
}

void cmd_vpll(BaseSequentialStream *chp, int argc, char *argv[]) {
  if( argc == 1 ) {
	int v = strtol(argv[0], NULL, 10);
	switch( v ) {
	    case 0:	vpll_code = 0; break;
	    case 50:	vpll_code = 1; break;
	    case 100:	vpll_code = 2; break;
	    case 150:	vpll_code = 3; break;
	    default: goto help;
	}
	set_vpll();
	return;
  }
help:
    chprintf(chp, "Usage: vpll 0|+50|+100|+150\r\n");
}
#endif

/* ---------- ---------- ---------- ---------- ---------- ---------- ---------- ---------- */
/* Add commands to shell list                                                              */
#if defined(BMC_WITH_SYSTEM_CONSOLE)
	// List of shell commands. Description of each command should be in the command print out/near command function in comments
	static const ShellCommand commands[] =
	{
		// Power maintenance commands
		{"atx", cmd_atx},
		{".vtg", cmd_vtg},
		{"vreg", cmd_vreg},
		#ifdef BOARD_BE_BT_BFK16
			{".tps56", cmd_tps56},
		#endif

		#ifdef BOARD_BE_BT_BFK20
			{"clk25", cmd_clk25},
			{"clk156", cmd_clk156},
			{".vpll",  cmd_vpll},
		#endif
		{"ina", cmd_ina},
		{"sqnr", cmd_sqnr},
		{"power", cmd_power},
		{".sensors", cmd_sensors},
		// DFU helpers
		{".dfu", cmd_dfu},
		{"flash_id", cmd_flash_id},
		{"factory_reset", cmd_factory},
		#ifdef BE_INTERNAL
			{"smb", cmd_smb},
			{".l11", cmd_l11},
			{".ascii", cmd_ascii},
			#if defined(MIPS_nBOOTCFG1_PORT)
				{"bootcfg",  cmd_bootcfg},
			#endif
			{"cpureset", cmd_cpureset},
			{"gpio", cmd_gpio},
			{"start", cmd_start},
			{"temp", cmd_temp},
			{"pac1720", cmd_pac1720},
			{"inTemp", cmd_insideTemperature},
	    #endif
		{NULL, NULL}
	};

	const ShellConfig shell_cfg1 =
	{
		.sc_channel = (BaseSequentialStream *)&SDU1,
		.sc_commands = commands
	};
#endif
