#ifndef COMMON_BMC_H
#define COMMON_BMC_H


#define VREG_FLAG_PMBUS		1	// supports
#define VREG_FLAG_STNDBY	2	// can be accessed on standby power
#define VREG_FLAG_PWRIN		4	// can be accessed on input power (12V)
#define VREG_FLAG_PERM		8	// permanent voltage setting

#define UCD90_PIN_MODE_UNUSED		0
#define UCD90_PIN_MODE_INPUT		1
#define UCD90_PIN_MODE_PUSHPULL		2
#define UCD90_PIN_MODE_OPENDRAIN	3

#if CH_CFG_NO_IDLE_THREAD
	#error "Incorrect config: #CH_CFG_NO_IDLE_THREAD required to allow sleep functions in main thread"
#endif

// This comments are saved for unexpected reason
//extern volatile unsigned atx_state;
//extern volatile unsigned sequencer_state;
//extern volatile bool dfu_active;
//extern volatile bool atx_present;

// atomic
//extern volatile bool pwc_locked;
//extern volatile int  pwc_state;

//extern const zl2102_dev_t vtg[3];

///* Virtual serial port over USB.*/
//extern SerialUSBDriver SDU1;
//extern DFU_USBDriver DFU1;
//extern USBHIDDriver HID1;
//
//extern const ShellConfig shell_cfg1;

/* Virtual serial port over USB.*/
extern SerialUSBDriver SDU1;
extern DFU_USBDriver   DFU1;
extern USBHIDDriver    HID1;

extern const ShellConfig shell_cfg1;

/*
 * I2C1/I2C2 config.
 */
const I2CConfig i2cfg1 = {
        OPMODE_I2C, // OPMODE_I2C OPMODE_SMBUS_HOST
        100000,
        STD_DUTY_CYCLE
};

const I2CConfig i2cfg2 = {
        OPMODE_I2C, // OPMODE_I2C OPMODE_SMBUS_HOST
		100000,
		STD_DUTY_CYCLE//FAST_DUTY_CYCLE_16_9
};

const I2CConfig i2cfg3 = {   // SMBA pin used as OTG_FS_VBUS
        OPMODE_I2C, // OPMODE_SMBUS_DEVICE,
        100000,		// 400000
        STD_DUTY_CYCLE //FAST_DUTY_CYCLE_16_9
};


// Flash 1 is Baikal boot flash
const sFLASH_USBConfig sFlash1 = {
    &SPID1,
    /*
     * SPI1 configuration structure.
     * Speed 5.25MHz, CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
     * The slave select line is the pin GPIOE_CS_SPI on the port GPIOE.
     */
    {
        NULL,
        /* HW dependent part.*/
        SPI1_CS_PORT,
        SPI1_CS_PIN,
        SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
    },
    BOOT_SPI_EN_PORT,
    BOOT_SPI_EN_PIN
};

sFLASH_USBDriver SFDU1;

/*
 * DFU USB driver configuration
 */
const DFU_USBConfig dfuusbcfg1 =
{
    &USBD1,
    {
        //temporary set flash 3 to flash 2. 3rd actually should be I2C EEPROM.
        &SFDU1
	#if DFU_INTERFACES == 2
			,&SFDU2
	#elif DFU_INTERFACES == 3
			,&SFDU3
	#endif
    }
};

#if HID_REPORT_STAT
	void update_filtered_value(int_filter_t* filter, report_value_t* report_value) {
		// Update average, we expect pre incremented value of filter_current here
		filter->filter -= filter->history[filter_current];
		filter->history[filter_current] = report_value->actual;
		filter->filter += filter->history[filter_current];
		report_value->avg = filter->filter >> FILTER_SHIFT;
		// Update min/max
		if(report_value->min > report_value->actual) {
			report_value->min = report_value->actual;
		}
		if(report_value->max < report_value->actual) {
			report_value->max = report_value->actual;
		}
	}
#endif

#ifdef BMC_WITH_PASSWORD
	const char password[] = "iddqd";
	const char* password_pos = password;


	static void password_check(char c) {
		if(*password_pos != '\0') {
			if(*password_pos == c) {
				password_pos += 1;
			} else {
				password_pos = password;
				if(*password_pos == c) {
					password_pos += 1;
				}
			}
		}
	}

	static int password_good(void) {
		return *password_pos == '\0';
	}

	static void password_reset(void) {
		password_pos = password;
	}
#else
	static void password_check(char c) {(void)c;}
	static int password_good(void) { return 1; }
	static void password_reset(void) {}
#endif


#if HID_REPORT_STAT
	#define FILTER_SHIFT 7
	// History length should be 2^FILTER_SHIFT
	#define FILTER_HISTORY_LENGTH 128

	// Be warned, that with 128 words filter length 32 bit accumulator will overflow with values > 16.7
	typedef struct {
		int history[FILTER_HISTORY_LENGTH];
		int filter;
	} int_filter_t;

	typedef struct {
		int_filter_t current;
		int_filter_t voltage;
		int_filter_t power;
	} ina_filter_t;

	static unsigned filter_current;
	static ina_filter_t ina_history[HID_REPORT_SENSOR_CHANNELS];
#endif

#if DFU_INTERFACES > 1
	// Flash 2 is External boot flash
	static const sFLASH_USBConfig sFlash2 = {
		&SPID1,
		/*
		 * SPI1 configuration structure.
		 * Speed 5.25MHz, CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
		 * The slave select line is the pin GPIOE_CS_SPI on the port GPIOE.
		 */
		{
			NULL,
			/* HW dependent part.*/
			SPI1_CS_PORT,
			SPI1_CS_PIN,
			SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
		},
		BOOT_SPI_EN_PORT,
		32  // not used
	};
	sFLASH_USBDriver SFDU2;
#endif

#ifdef VIDEO_SPI_PORT
	// Flash 2 is Video BIOS flash
	static const sFLASH_USBConfig sFlash3 = {
		&SPID2,
		/*
		 * SPI1 configuration structure.
		 * Speed 5.25MHz, CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
		 * The slave select line is the pin GPIOE_CS_SPI on the port GPIOE.
		 */
		{
			NULL,
			/* HW dependent part.*/
			GPIOB,
			9,
			SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
		},
		GPIOC,
		14
	};

	sFLASH_USBDriver SFDU3;
#endif

#ifdef BOARD_BE_BT_BFK20
	uint16_t vpll_code = 0;
	void set_vpll(void)
	{
		palWriteGroup(GPIOC,3,1,~vpll_code);
	}

	const pmbus_dev_t tps =
	{
		&I2CD2, 0x30, "0.9V_SATA/0.95V_PCIe/0.95V_XGBE"
	};
	uint16_t tps_val[TPS_CHANNEL_COUNT] = {0,0,0}; // {900,950,950}
#else
	// TPS56 settings is not permanent!!
	const pmbus_dev_t tps =
	{
		&I2CD2, 0x37, "0.95V XGBE"
	};
	uint16_t tps_val[TPS_CHANNEL_COUNT] = {0}; // {950}
#endif

#endif // COMMON_BMC_H
