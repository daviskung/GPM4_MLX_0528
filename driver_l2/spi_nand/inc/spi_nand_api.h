#ifndef SPI_NAND_API_H
#define SPI_NAND_API_H

// below apis must call before spi nand init
// cs_io : when 0xFF, cs be assigned inner according to pinmux
// Currently, cs_io = 0xFF have no function on GPL326XXA, GPL326XXB, GPL327XXA
extern INT32S spi_nand_assign_cs(INT8U cs_io);

// pinmux: when 0xFF, auto find the pinmux.
// Currently, this api have no function on GPL326XXA, GPL326XXB, GPL327XXA
// For GPM4 platform, currently, only pinmux = 0xFF  have effect.
extern INT32S spi_nand_assign_pinmux(INT8U pinmux);

// ch: 0 or 1, select spi controller 0, or 1. Some asic have only one spi controller, so allow only 0.
extern INT32S spi_nand_assign_channel(INT8U ch);

// suggest set maximal clock rate not more than half of system clock rate and under device maximal colck rate
// spi clock rate is 1,2,4,8,16,32,64 divider of system clock rate
// tx and rx clock rate could be different, normally rx clock rate lower than tx clock rate
// must arrange burn test(ex do random read write compare burn test through msdc) to confirm clock rate is stable
// for whole system(consider board circuit quality, device and controller maximal clock rate)
extern INT32S spi_nand_assign_clock_rate(INT32U tx_clock_rate, INT32U rx_clock_rate);


// locate : 0 or 1, when 1, spi nand init will try to find maixmal tx, rx clock rate
// use    : 0 or 1, when 1, use the found maximal tx, rx, clock rate
// Note that, the found maximal tx rx clok rate may not stable for the spi nand part number and your board,
// have to do burn test under cold and hot temperature to confirm. Basically, rx clock rate have to down half
// for stable. Suggest that use spi_nand_assign_clock_rate to set your final approved clock rate, and remark
// calling spi_nand_assign_fast_clock_flag api, and set the use to 0.
extern INT32S spi_nand_assign_fast_clock_flag(INT8U locate, INT8U use);

// 0: no change, also mean use device default, 1=100%, 2=75%, 3=50%, 4=25%
extern void spi_nand_assign_user_driver_strength_of_device(INT8U strength);

// above apis must call before spi nand init

extern INT8U spi_nand_user_driver_strength_of_device_get(void);
extern INT32S spi_nand_init(void); // will be called inner
extern INT16U spi_nand_get_mainId(void); // just for purpose to get spi nand id. help:cmd>nandemo data getid 1
extern void spi_nand_set_debug_flag(INT8U debugFlag); // just for purpose to debug
extern INT8U spi_nand_get_debug_flag(void); // just for purpose to debug

extern INT8U spi_nand_channel_get(void);
extern INT8U spi_nand_pinmux_get(void);
extern INT8U spi_nand_cs_get(void);

#endif // SPI_NAND_API_H
