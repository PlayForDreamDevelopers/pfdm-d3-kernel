#ifndef _MT3503_H_
#define _MT3503_H_


enum mt3503_regs
{
	PRODUCT_ID1 		= 0x00,
	PRODUCT_ID2 		= 0x01,
	MOTION_STATUS 	    = 0x02,
	DELTA_X_LO			= 0x03,
	DELTA_Y_LO			= 0x04,
	OPERATION_MODE      = 0x05,
	CONFIGURATION		= 0x06,
	WRITE_PROTECT		= 0x09,
	SLEEP1              = 0x0a,
	SLEEP2 		        = 0x0b,
	SLEEP3 		        = 0x0c,
	RES_X_LO 		    = 0x0d,
	RES_Y_LO 		    = 0x0e,
	RES_XY_HI 			= 0x0f,
	DELTA_XY_HI 		= 0x12,
	FRAMEAVG		    = 0x1a,
	KEY_WP  		    = 0x7f,
	KEY_DET_CFG 		= 0x28,
	KEY_INT_FLAG 		= 0x29,
	KEY_STATUS 			= 0x2a,
};

#define CHIP_ID 0xa241
#define Motion_Status 0x02
#define Delta_X_Lo    0x03
#define Delta_Y_Lo    0x04
#define Delta_XY_Hi   0x12

#endif
