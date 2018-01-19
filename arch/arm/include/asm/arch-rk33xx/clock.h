/*
 * (C) Copyright 2008-2015 Rockchip Electronics
 * Peter, Software Engineering, <superpeter.cai@gmail.com>.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _RKXX_CLOCK_H
#define _RKXX_CLOCK_H

#include <asm/arch/typedef.h>


/* config cpu and general clock in MHZ */
#define KHZ				(1000)
#define MHZ				(1000*1000)

/* define pll mode */
#define RKCLK_PLL_MODE_SLOW		0
#define RKCLK_PLL_MODE_NORMAL		1


/*
 * rkplat clock set pll mode
 */
void rkclk_pll_mode(int pll_id, int pll_mode);

/*
 * rkplat clock set pll rate by id
 */
void rkclk_set_pll_rate_by_id(enum rk_plls_id pll_id, uint32 mHz);

/*
 * rkplat clock get pll rate by id
 */
uint32 rkclk_get_pll_rate_by_id(enum rk_plls_id pll_id);

/*
 * rkplat clock set for arm and general pll
 */
void rkclk_set_pll(void);

/*
 * rkplat clock get arm pll, general pll and so on
 */
void rkclk_get_pll(void);

/*
 * rkplat clock pll dump
 */
void rkclk_dump_pll(void);


/*
 * rkplat clock set npll
 */
int rkclk_set_npll_rate(uint32 pll_hz);


/*
 * rkplat set nandc clock div
 * nandc_id:	nandc id
 * pllsrc: 	nandc clock src;
 * freq:	nandc max freq request.
 */
int rkclk_set_nandc_div(uint32 nandc_id, uint32 pllsrc, uint32 freq);


/*
 * rkplat set mmc clock src
 * sdid:	sdmmc/sdio/emmc id
 * src:		sd clock source
 */
void rkclk_set_mmc_clk_src(uint32 sdid, uint32 src);

/*
 * rkplat get sd/sdmmc/emmc clock source freq
 * sdid:	sdmmc/sdio/emmc id
 */
unsigned int rkclk_get_mmc_clk(uint32 sdid);

/*
 * rkplat set sd clock div, from source input
 * sdid:	sdmmc/sdio/emmc id
 * div:		sd clock div, start from 1
 */
int rkclk_set_mmc_clk_div(uint32 sdid, uint32 div);

/*
 * rkplat set sd clock freq, from source input
 * sdid:	sdmmc/sdio/emmc id
 * freq:	sd clock freq
 */
int32 rkclk_set_mmc_clk_freq(uint32 sdid, uint32 freq);
/*
 * rkplat set sd clock div, from source input
 * sdid:	sdmmc/sdio/emmc id
 * degree:  tuning degree 0/1/2/3
 * delay_num: tuning delay_num 0~255
 */
int rkclk_set_mmc_tuning(uint32 sdid, uint32 degree, uint32 delay_num);

/*
 * rkplat disable mmc clock tuning
 */
int rkclk_disable_mmc_tuning(uint32 sdid);

/*
 * rkplat get PWM clock
 * id:		pwm id
 */
unsigned int rkclk_get_pwm_clk(uint32 id);


/*
 * rkplat get I2C clock
 * i2c_bus_id:	i2c bus id
 */
unsigned int rkclk_get_i2c_clk(uint32 i2c_bus_id);


/*
 * rkplat get spi clock
 * spi_bus:	spi bus id
 */
unsigned int rkclk_get_spi_clk(uint32 spi_bus);


/*
 * rkplat lcdc dclk and aclk parent pll source
 * lcdc_id (lcdc id select) :	start from 0, max depend chip platform.
 * dclk_hz:			dclk rate
 * return dclk rate
 */
int rkclk_lcdc_clk_set(uint32 lcdc_id, uint32 dclk_hz);


/*
 * rkplat pll select by clock
 * clock: device request freq HZ
 * return value:
 * high 16bit:		clock source select
 * low 16bit :		div
 */
uint32 rkclk_select_pll_source(uint32 clock, uint32 even);


#ifdef CONFIG_SECUREBOOT_CRYPTO
/*
 * rkplat set cryto clock
 * here no check clkgate, because chip default is enable.
 */
void rkclk_set_crypto_clk(uint32 rate);
#endif /* CONFIG_SECUREBOOT_CRYPTO */


#endif	/* _RKXX_CLOCK_H */
