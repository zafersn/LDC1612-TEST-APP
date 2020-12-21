#ifndef __LDC1612_DRIVER_H__
#define __LDC1612_DRIVER_H__

// LDC COMMANDS
//#define LDC1612_REG_DATA_MSB_CH0		0x00
//#define LDC1612_REG_DATA_LSB_CH0		0x01
#define LDC1612_REG_DATA_MSB_CH1		0x02
#define LDC1612_REG_DATA_LSB_CH1		0x03
//#define LDC1612_REG_REF_COUNT_CH0		0x08
#define LDC1612_REG_REF_COUNT_CH1		0x09
//#define LDC1612_REG_OFFSET_CH0	        	0x0C
#define LDC1612_REG_OFFSET_CH1	        	0x0D
//#define LDC1612_REG_SETTLE_COUNT_CH0		0x10
#define LDC1612_REG_SETTLE_COUNT_CH1		0x11
//#define LDC1612_REG_CLOCK_DIVIDERS_CH0 		0x14
#define LDC1612_REG_CLOCK_DIVIDERS_CH1 		0x15
#define LDC1612_REG_STATUS 	        	0x18
#define LDC1612_REG_ERROR_CONFIG 		0x19
#define LDC1612_REG_CONFIG 	        	0x1A
#define LDC1612_REG_MUX_CONFIG 	        	0x1B
#define LDC1612_REG_RESET_DEVICE 		0x1C
//#define LDC1612_REG_DRIVE_CURRENT_CH0		0x1E
#define LDC1612_REG_DRIVE_CURRENT_CH1 		0x1F
#define LDC1612_REG_MANUFACTID	        	0x7E
#define LDC1612_REG_DEVID	        	0x7F


//status bitmasks

#define LDC1612_BIT_UNREADCONV1  		0x04 //Data Ready Flag //: An unread conversion is present for Channel 1. Read Registers DATA1_MSB and DATA1_LSB to retrieve conversion results.
//CHANNEL 1 ERROR BITMASK
#define LDC1612_BIT_MSB_ERR_UR1  		0x8000 //Channel 1 Conversion Under-range Error Flag
#define LDC1612_BIT_MSB_ERR_OR1  		0x4000 //Channel 1 Conversion Over-range Error Flag
#define LDC1612_BIT_MSB_ERR_WD1  		0x5000 //Channel 1 Conversion Watchdog Timeout Error Flag
#define LDC1612_BIT_MSB_ERR_AE1  		0x1000 //Channel 1 Conversion Amplitude Error Flag





// RESET_DEV Values
#define RESET_DEV                   		0x8000

/*PCB dizayninda biz LDC1612'nin adres ayagini LOW'a cektigimiz zaman gecerli adress */
#define LDC1612_MIN_I2CADDR                	(0x2A<<1)

/*PCB dizayninda biz LDC1612'nin adres ayagini HIGH'a cektigimiz zaman gecerli adress */
#define LDC1612_MAX_I2CADDR_WR             	(0x2B<<1)

//Config Values
#define ACTIVE_CHAN_0				0x0000
#define ACTIVE_CHAN_1				0x4000
#define SLEEP_MODE_EN				0x2000
#define SLEEP_MODE_DISABLE			0x0000
#define RP_OVERRIDE_EN				0x1000
#define RP_OVERRIDE_DISABLE			0x0000

#define SENSOR_ACTIVATE_SEL			0x0000
#define AUTO_AMP_DIS				0x0400
#define REF_CLK_SRC				0x0000
#define CONFIG_RESERVED             		0x0000
#define INTB_DIS             			0x0080
#define HIGH_CURRENT_DRV_EN                     0x0040
#define RESERVED_REG_CONFIG			0x0001

//MUX_CONFIG Values
#define AUTOSCAN_EN				0x0000
#define MUX_CONFIG_RESERVED         		0x0208
#define DEGLITCH_3N3MHZ				0x0004
#define DEGLITCH_10MHZ				0x0005
#define DEGLITCH_33MHZ				0x0006

#define IamLDC1612                      	0x3055


/*******************************************************************************
 * The following macros allow to modify the LDC1612 scan rate and the operation
 * option for polynomial fitting.
 ******************************************************************************/
// The define below allow user to adjust scan period in sec.
#define LDC_SCAN_PERIOD


#endif /* __LDC1612_DRIVER_H__ */
