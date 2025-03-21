#ifndef _TPS92391_BL_
#define _TPS92391_BL_

typedef struct {
	uint8_t reg;
	uint16_t val;
} tps92391_bl_reg;

static tps92391_bl_reg tps92391_bl_init_reg[] = {
	{0x02, 0x7A},
};

static uint8_t reg_dump[] = {
	0x00,
	0x02,
	0x04,
};

#endif