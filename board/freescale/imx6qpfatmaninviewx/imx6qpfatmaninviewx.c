/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/pfuze.h"
#include <asm/arch/mx6-ddr.h>
#include <usb.h>
#ifdef CONFIG_CMD_SATA
#include <asm/imx-common/sata.h>
#endif
#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define I2C2_PAD_CTRL    (PAD_CTL_PUS_22K_UP | PAD_CTL_PUE | PAD_CTL_PKE | \
	PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)
	
#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                  \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)	
	
#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define MDIO_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST | PAD_CTL_ODE)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define FASTGPIO_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PMIC	1

#ifdef CONFIG_SYS_I2C_MXC
#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC and EEPROM */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode =  MX6_PAD_CSI0_DAT9__I2C1_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 27),
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 26),
	},
};

static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode =  MX6_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C2_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C2_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12),
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C2_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C2_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13),
	},
};
#endif

int power_init_board(void)
{
	struct pmic *pfuze;
	unsigned int reg;
	int ret;

	pfuze = pfuze_common_init(I2C_PMIC);
	if (!pfuze)
		return -ENODEV;

	if (is_mx6dqp())
		ret = pfuze_mode_init(pfuze, APS_APS);
	else
		ret = pfuze_mode_init(pfuze, APS_PFM);

	if (ret < 0)
		return ret;
	/* VGEN3 and VGEN5 corrected on i.mx6qp board */
	if (!is_mx6dqp()) {
		/* Increase VGEN3 from 2.5 to 2.8V */
		pmic_reg_read(pfuze, PFUZE100_VGEN3VOL, &reg);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_2_80V;
		pmic_reg_write(pfuze, PFUZE100_VGEN3VOL, reg);

		/* Increase VGEN5 from 2.8 to 3V */
		pmic_reg_read(pfuze, PFUZE100_VGEN5VOL, &reg);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_3_00V;
		pmic_reg_write(pfuze, PFUZE100_VGEN5VOL, reg);
	}

	if (is_mx6dqp()) {
		/* set SW1C staby volatage 1.075V*/
		pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1f;
		pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);

		/* set SW2/VDDARM staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW2STBY, &reg);
		reg &= ~0x3f;
		reg |= 0x17;
		pmic_reg_write(pfuze, PFUZE100_SW2STBY, reg);

		/* set SW2/VDDARM step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW2CONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW2CONF, reg);
	} else {
		/* set SW1AB staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW1ABSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(pfuze, PFUZE100_SW1ABSTBY, reg);

		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1ABCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1ABCONF, reg);

		/* set SW1C staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);
	}

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	int is_400M;
	unsigned char vddarm;
	struct pmic *p = pmic_get("PFUZE100");

	if (!p) {
		printf("No PMIC found!\n");
		return;
	}

	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0;	/* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		if (is_mx6dqp()) {
			/* increase VDDARM to 1.425V */
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			value |= 0x29;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		} else {
			/* increase VDDARM to 1.425V */
			pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
			value &= ~0x3f;
			value |= 0x2d;
			pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
		}
		/* increase VDDSOC to 1.425V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x2d;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);
	}
	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		prep_anatop_bypass();
		if (is_mx6dqp()) {
			/* decrease VDDARM for 400Mhz DQP:1.1V*/
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			value |= 0x1c;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		} else {
			/* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
			pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
			value &= ~0x3f;
			if (is_cpu_type(MXC_CPU_MX6DL))
				value |= 0x27;
			else
				value |= 0x20;

			pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
		}
		/* increase VDDSOC to 1.3V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x28;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);

		/*
		 * MX6Q/DQP:
		 * VDDARM:1.15V@800M; VDDSOC:1.175V@800M
		 * VDDARM:0.975V@400M; VDDSOC:1.175V@400M
		 * MX6DL:
		 * VDDARM:1.175V@800M; VDDSOC:1.175V@800M
		 * VDDARM:1.075V@400M; VDDSOC:1.175V@400M
		 */
		is_400M = set_anatop_bypass(2);
		if (is_mx6dqp()) {
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			if (is_400M)
				value |= 0x17;
			else
				value |= 0x1e;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		}

		if (is_400M) {
			if (is_cpu_type(MXC_CPU_MX6DL))
				vddarm = 0x1f;
			else
				vddarm = 0x1b;
		} else {
			if (is_cpu_type(MXC_CPU_MX6DL))
				vddarm = 0x23;
			else
				vddarm = 0x22;
		}
		pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
		value &= ~0x3f;
		value |= vddarm;
		pmic_reg_write(p, PFUZE100_SW1ABVOL, value);

		/* decrease VDDSOC to 1.175V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x23;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);

		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_GPIO_7__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_GPIO_8__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D28__UART2_DTE_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D29__UART2_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart5_pads[] = {
	MX6_PAD_KEY_COL1__UART5_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW1__UART5_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	printf("setup uart1 iomux\n");
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	printf("setup uart2 iomux\n");
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	printf("setup uart5 iomux\n");
	imx_iomux_v3_setup_multiple_pads(uart5_pads, ARRAY_SIZE(uart5_pads));
}


// ALTANEOS USED FOR EMMC1 - CD (CARD DETECT) IS NOT IMPLEMENTED ONBOARD
static iomux_v3_cfg_t const usdhc3_emmc1_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_RST__GPIO7_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL), /* RST_B */
};

// ALTANEOS USED FOR EMMC2
static iomux_v3_cfg_t const usdhc4_emmc2_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_ALE__GPIO6_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL), /* RST_B */
};

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR, 0, 8},
	{USDHC4_BASE_ADDR, 0, 8},
};

#define USDHC3_PWR_GPIO	IMX_GPIO_NR(7, 8)
#define USDHC4_PWR_GPIO	IMX_GPIO_NR(6, 8)

int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno + 1;
}

int board_mmc_getcd(struct mmc *mmc)
{
/*  	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; 
		break;
	}  */
	return 1;
}

int board_mmc_init(bd_t *bis)
{
#ifdef CONFIG_SPL_BUILD
    int nRet = 0;
	imx_iomux_v3_setup_multiple_pads(usdhc3_emmc1_pads, ARRAY_SIZE(usdhc3_emmc1_pads));
	imx_iomux_v3_setup_multiple_pads(usdhc4_emmc2_pads, ARRAY_SIZE(usdhc4_emmc2_pads));

	gpio_direction_output(USDHC3_PWR_GPIO, 0);
	udelay(500);
	gpio_direction_output(USDHC3_PWR_GPIO, 1);
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	nRet = fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
	if (nRet != 0) {
		printf(" --EE: board_mmc_init() EMMC#0 Failed to init");
		return nRet;
	}

	gpio_direction_output(USDHC4_PWR_GPIO, 0);
	udelay(500);
	gpio_direction_output(USDHC4_PWR_GPIO, 1);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
	nRet = fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
	if (nRet != 0) {
		printf(" --EE: board_mmc_init() EMMC#1 Failed to init");
		return nRet;
	}
	return nRet;
#else
	int i, ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_emmc1_pads, ARRAY_SIZE(usdhc3_emmc1_pads));

			gpio_direction_output(USDHC3_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC3_PWR_GPIO, 1);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_emmc2_pads, ARRAY_SIZE(usdhc4_emmc2_pads));

			gpio_direction_output(USDHC4_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC4_PWR_GPIO, 1);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers (%d) than supported by the board\n", i + 1);
			return -EINVAL;
			}

			ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
			if (ret) {
				printf("Warning: failed to initialize mmc dev %d\n", i);
			}
	}
#endif
	return 0;
}
#endif

#ifdef CONFIG_MXC_SPI
static iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_CSI0_DAT4__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT7__GPIO5_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define ECSPI1_CS		IMX_GPIO_NR(5, 25)

static void setup_spi(void)
{
	int i;
	printf("setup_spi\n");
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));

	for (i = 0; i < 3; i++) {
		enable_spi_clk(1, i);
	}
	gpio_direction_output(ECSPI1_CS, 1);
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 1 && cs == 0) ? (ECSPI1_CS) : -1;
}
#endif

// ALTANEOS VERIFY
static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD0__ENET_TX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD1__ENET_TX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET_RXD0__ENET_RX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD1__ENET_RX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__ENET_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const fec_rst_pads[] = {
	MX6_PAD_NANDF_CS3__GPIO6_IO16 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

#define ENET_nRST IMX_GPIO_NR(6, 16)

static void setup_iomux_fec_rst(void)
{
	imx_iomux_v3_setup_multiple_pads(fec_rst_pads,
						ARRAY_SIZE(fec_rst_pads));
}

/* static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_pads,
						ARRAY_SIZE(fec1_pads));
} */

/* int board_eth_init(bd_t *bis)
{
	setup_iomux_fec();

	return fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
				       CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
} */

static void fec_reset(void)
{
	gpio_direction_output(ENET_nRST, 0);
	udelay(500);
	gpio_direction_output(ENET_nRST, 1);
}

// ALTANEOS : KEEP BECAUSE VERY IMPORTANT
/* static void setup_fec(void)
{
	if (is_mx6dqp()) {
		int ret;

		// select ENET MAC0 TX clock from PLL  //
		imx_iomux_set_gpr_register(5, 9, 1, 1);
		ret = enable_fec_anatop_clock(0, ENET_125MHZ);
		if (ret)
		    printf("Error fec anatop clock settings!\n");
	}
} */
static int setup_fec(void)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	if (check_module_fused(MX6_MODULE_ENET1))
		return -1;

		/*
		 * Use 50M anatop loopback REF_CLK1 for ENET1,
		 * clear gpr1[13], set gpr1[17].
		 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
				IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);


	ret = enable_fec_anatop_clock(0, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);

	return 0;
}

//ALTANEOS ADD : NO PHY
int board_phy_config(struct phy_device *phydev)
{
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8190);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(LCD_PAD_CTRL),	// enable
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(LCD_PAD_CTRL),	// hsync
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(LCD_PAD_CTRL),	//vsync
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	/* LCD_PWR */
	MX6_PAD_GPIO_3__GPIO1_IO03 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_DOWN),

	/* LCD_PWM */
	MX6_PAD_SD1_DAT3__PWM1_OUT | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define DISP0_PWR_EN IMX_GPIO_NR(4, 17)
void do_enable_parallel_lcd(struct display_info_t const *dev)
{
	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));
	
	gpio_direction_output(DISP0_PWR_EN, 1);
}

struct display_info_t const displays[] = {
	{
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= 18,
	.detect	= NULL,
	.enable	= do_enable_parallel_lcd,
	.mode	= {
		.name			= "HX8258A",
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 30072,
		.left_margin    = 62,
		.right_margin   = 62,
		.upper_margin   = 24,
		.lower_margin   = 24,
		.hsync_len      = 80,
		.vsync_len      = 48,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
		}
	} 
};
size_t display_count = ARRAY_SIZE(displays);

#endif

static iomux_v3_cfg_t const touch_pads[] = {
	/* TS_IRQ || ALTANEOS nInt ? */
	MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(FASTGPIO_PAD_CTRL),
	/* TS_nRST */
	MX6_PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(FASTGPIO_PAD_CTRL),
};

#define TOUCH_IRQ_GPIO		IMX_GPIO_NR(1, 2)
#define TOUCH_nRST_GPIO		IMX_GPIO_NR(7, 13)

int setup_touch(void)
{
	printf("%s\n", __FUNCTION__);

	imx_iomux_v3_setup_multiple_pads(
		touch_pads, ARRAY_SIZE(touch_pads));

	/* HW reset */
	gpio_direction_output(TOUCH_nRST_GPIO, 1);
	mdelay(1);
	gpio_direction_output(TOUCH_nRST_GPIO, 0);
	mdelay(100);
	gpio_direction_output(TOUCH_nRST_GPIO, 1);
	mdelay(200);

	return 0;
}

static iomux_v3_cfg_t const leds_pads[] = {
	/* led_pwm1 */
	MX6_PAD_SD1_DAT2__PWM2_OUT | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* led_pwm2 */
	MX6_PAD_SD1_DAT1__PWM3_OUT | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* led_pwm3 */
	MX6_PAD_SD1_CMD__PWM4_OUT | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* buzzer */
	MX6_PAD_GPIO_0__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* debug */
	MX6_PAD_SD2_DAT0__GPIO1_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define LED_PWM1_GPIO			IMX_GPIO_NR(1, 19)
#define LED_PWM2_GPIO			IMX_GPIO_NR(1, 17)
#define LED_PWM3_GPIO			IMX_GPIO_NR(1, 18)
#define LED_BUZZER_GPIO		IMX_GPIO_NR(1, 0)
#define LED_DEBUG_GPIO		IMX_GPIO_NR(1, 15)

int setup_led(void)
{
	printf("%s\n", __FUNCTION__);

	imx_iomux_v3_setup_multiple_pads(
		leds_pads, ARRAY_SIZE(leds_pads));

	gpio_direction_output(LED_PWM1_GPIO, 1);
	gpio_direction_output(LED_PWM2_GPIO, 1);
	gpio_direction_output(LED_PWM3_GPIO, 1);
	gpio_direction_output(LED_DEBUG_GPIO, 1);
	gpio_direction_output(LED_BUZZER_GPIO, 1);
	mdelay(500);
	gpio_direction_output(LED_BUZZER_GPIO, 0);
	mdelay(1500);
	gpio_direction_output(LED_PWM1_GPIO, 0);
	gpio_direction_output(LED_PWM2_GPIO, 0);
	gpio_direction_output(LED_PWM3_GPIO, 0);
	gpio_direction_output(LED_DEBUG_GPIO, 0);
	/* Testing ... */
	mdelay(500);
	gpio_set_value(LED_DEBUG_GPIO, 0);
	mdelay(500);
	gpio_set_value(LED_DEBUG_GPIO, 1);

	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

static iomux_v3_cfg_t const ninja_pads[] = {
	/* boot */
	MX6_PAD_SD1_DAT0__GPIO1_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* nreset */
	MX6_PAD_GPIO_1__GPIO1_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define NINJA_BOOT_GPIO		IMX_GPIO_NR(1, 16)
#define NINJA_nRST_GPIO		IMX_GPIO_NR(1, 1)

int setup_ninja(void)
{
	printf("%s\n", __FUNCTION__);

	imx_iomux_v3_setup_multiple_pads(
		ninja_pads, ARRAY_SIZE(ninja_pads));

	gpio_direction_output(NINJA_BOOT_GPIO, 0);
	/* Reset the ninja module */
	gpio_direction_output(NINJA_nRST_GPIO, 1);
	mdelay(1);
	gpio_direction_output(NINJA_nRST_GPIO, 0);
	mdelay(100);
	gpio_direction_output(NINJA_nRST_GPIO, 1);

	return 0;
}


static iomux_v3_cfg_t const dio_pads[] = {
	/* din0 */
	MX6_PAD_NANDF_CS0__GPIO6_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* din1 */
	MX6_PAD_NANDF_CS1__GPIO6_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* relay0 */
	MX6_PAD_NANDF_RB0__GPIO6_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* relay1 */
	MX6_PAD_NANDF_WP_B__GPIO6_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define DIN0_GPIO	IMX_GPIO_NR(6,11)
#define DIN1_GPIO	IMX_GPIO_NR(6,14)
#define RELAY0_GPIO	IMX_GPIO_NR(6,10)
#define RELAY1_GPIO	IMX_GPIO_NR(6,9)


int setup_dio(void)
{
	printf("%s\n", __FUNCTION__);

	imx_iomux_v3_setup_multiple_pads(
		dio_pads, ARRAY_SIZE(dio_pads));

	gpio_direction_input(DIN0_GPIO);
	gpio_direction_input(DIN1_GPIO);
	gpio_direction_output(RELAY0_GPIO, 0);
	gpio_direction_output(RELAY1_GPIO, 0);

	return 0;
}

int setup_splash(void)
{
	int ret = 0;
	char *strldaddr = NULL;
	ulong ldaddr = 0;

	char cmdarg[64];

	printf("%s\n", __FUNCTION__);

	memset(strldaddr, 0, sizeof(strldaddr));
	if (!getenv("splashaddr"))
		return 0;
	strldaddr = getenv("splashaddr");
	ldaddr = getenv_hex("splashaddr", 0);
	printf("ldaddr = %s (0x%X)\n", strldaddr, (unsigned int) ldaddr);
	if (strldaddr == NULL)
		return 0;

	snprintf(cmdarg, sizeof(cmdarg), "load mmc 1:5 %s /data/default/splash-normal.bmp", strldaddr);
	run_command(cmdarg, 0);
	ret = bmp_display(ldaddr, 0, 0);

	return ret;
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_EIM_D22__USB_OTG_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usb_hc1_pads[] = {
	MX6_PAD_EIM_D21__USB_OTG_OC | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));

	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 0);

	imx_iomux_v3_setup_multiple_pads(usb_hc1_pads,
					 ARRAY_SIZE(usb_hc1_pads));
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		if (on)
			gpio_direction_output(IMX_GPIO_NR(1, 29), 1);
		else
			gpio_direction_output(IMX_GPIO_NR(1, 29), 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

#define EEPROM_DATA_DEVADDR    0x50    /* User data sector of the AT24MAC */
#define EEPROM_DATA_DEVADDR1   0x53    /* User data sector of 2nd AT24MAC */
#define EEPROM_MAC_DEVADDR     0x58    /* RO serial sector of the AT24MAC */
#define EEPROM_MAC_DEVADDR1    0x5B    /* RO serial sector of 2nd AT24MAC */
#define EEPROM_MAC_REG_MAC     0x9A
#define EEPROM_MAC_REG_GUID    0x80

int misc_init_r(void)
{
	uchar buf[6];
	char str[18];

	/* Read ethaddr from EEPROM */
	if (i2c_read(EEPROM_MAC_DEVADDR, EEPROM_MAC_REG_MAC, 1, buf, 6) == 0) {
		sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		/* Check if MAC address is valid - Must match the at24mac prefix*/
		if (strstr(str, "FC:C2:3D") != str)
			printf("Error - Bad MAC address (%s)\n", str);
		else {
			printf("Use MAC address from EEPROM (%s)\n", str);
			setenv("ethaddr", str);
		}
	} else {
		printf("Warning - Unable to read MAC from I2C device %02X @%04X\n",
			EEPROM_MAC_DEVADDR,
			EEPROM_MAC_REG_MAC);
	}

	if (!getenv("ethaddr"))
		printf("MAC address not set\n");
	
	/* Inview X has a 2nd EEPROM */
	if (i2c_read(EEPROM_MAC_DEVADDR1, EEPROM_MAC_REG_MAC, 1, buf, 6) == 0) {
		sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		// Check if MAC address is valid - Must match the at24mac prefix
		if (strstr(str, "FC:C2:3D") != str)
			printf("Error - Bad 2nd MAC address (%s)\n", str);
		else {
			printf("Use 2nd MAC address from EEPROM (%s)\n", str);
			setenv("eth1addr", str);
		}
	} else {
		printf("Warning - Unable to read MAC from I2C device %02X @%04X\n",
			EEPROM_MAC_DEVADDR1,
			EEPROM_MAC_REG_MAC);
	}

	if (!getenv("eth1addr"))
		printf("2nd MAC address not set\n");

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_iomux_fec_rst();
	fec_reset();
	setup_dio();
	
#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x08, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_touch();
#endif

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

#ifdef	CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#if defined(CONFIG_MXC_SPI) || defined(CONFIG_FSL_QSPI)
	setup_spi();
#endif

	setup_led();
	setup_ninja();
	
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	{"emmc", MAKE_CFGVAL(0x62, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
	printf("board_late_init\n");
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	setenv("board_name", "IMX6QPFATMANINVIEWX");

	if (is_mx6dqp())
		setenv("board_rev", "MX6QP");
	else if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D))
		setenv("board_rev", "MX6Q");
	else if (is_cpu_type(MXC_CPU_MX6DL) || is_cpu_type(MXC_CPU_MX6SOLO))
		setenv("board_rev", "MX6DL");
	printf("\n\n\n\n\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n\n\n\n");
	setenv("test", "test_in_c_file");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
	
	setup_splash();

	return 0;
}

int checkboard(void)
{
	puts("Board: IMX6QP FATMAN INVIEWX\n");
	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT

void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "sata");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota sata");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc0");
	    break;
	case SD3_BOOT:
	case MMC3_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc1");
	    break;
	case MMC4_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc2");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc2");
	    break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}

}

#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int check_recovery_cmd_file(void)
{
    int button_pressed = 0;
    int recovery_mode = 0;

    recovery_mode = recovery_check_and_clean_flag();

    /* Check Recovery Combo Button press or not. */
	imx_iomux_v3_setup_multiple_pads(recovery_key_pads,
			ARRAY_SIZE(recovery_key_pads));

    gpio_direction_input(GPIO_VOL_DN_KEY);

    if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
    }

    return recovery_mode || button_pressed;
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota sata recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota mmc1 recovery");
		break;
	case MMC4_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota mmc2 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	setenv("bootcmd", "run bootcmd_android_recovery");
}

#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FSL_FASTBOOT*/


#ifdef CONFIG_SPL_BUILD
#include <spl.h>
#include <libfdt.h>

const struct mx6dq_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_sdclk_0 =  0x00020030,
	.dram_sdclk_1 =  0x00020030,
	.dram_cas =  0x00020030,
	.dram_ras =  0x00020030,
	.dram_reset =  0x00020030,
	.dram_sdcke0 =  0x00003000,
	.dram_sdcke1 =  0x00003000,
	.dram_sdba2 =  0x00000000,
	.dram_sdodt0 =  0x00003030,
	.dram_sdodt1 =  0x00003030,
	.dram_sdqs0 =  0x00000030,
	.dram_sdqs1 =  0x00000030,
	.dram_sdqs2 =  0x00000030,
	.dram_sdqs3 =  0x00000030,
	.dram_sdqs4 =  0x00000030,
	.dram_sdqs5 =  0x00000030,
	.dram_sdqs6 =  0x00000030,
	.dram_sdqs7 =  0x00000030,
	.dram_dqm0 =  0x00020030,
	.dram_dqm1 =  0x00020030,
	.dram_dqm2 =  0x00020030,
	.dram_dqm3 =  0x00020030,
	.dram_dqm4 =  0x00020030,
	.dram_dqm5 =  0x00020030,
	.dram_dqm6 =  0x00020030,
	.dram_dqm7 =  0x00020030,
};

const struct mx6dq_iomux_ddr_regs mx6dqp_ddr_ioregs = {
	.dram_sdclk_0 =  0x00000030,
	.dram_sdclk_1 =  0x00000030,
	.dram_cas =  0x00000030,
	.dram_ras =  0x00000030,
	.dram_reset =  0x00000030,
	.dram_sdcke0 =  0x00003000,
	.dram_sdcke1 =  0x00003000,
	.dram_sdba2 =  0x00000000,
	.dram_sdodt0 =  0x00003030,
	.dram_sdodt1 =  0x00003030,
	.dram_sdqs0 =  0x00000030,
	.dram_sdqs1 =  0x00000030,
	.dram_sdqs2 =  0x00000030,
	.dram_sdqs3 =  0x00000030,
	.dram_sdqs4 =  0x00000030,
	.dram_sdqs5 =  0x00000030,
	.dram_sdqs6 =  0x00000030,
	.dram_sdqs7 =  0x00000030,
	.dram_dqm0 =  0x00000030,
	.dram_dqm1 =  0x00000030,
	.dram_dqm2 =  0x00000030,
	.dram_dqm3 =  0x00000030,
	.dram_dqm4 =  0x00000030,
	.dram_dqm5 =  0x00000030,
	.dram_dqm6 =  0x00000030,
	.dram_dqm7 =  0x00000030,
};

const struct mx6dq_iomux_grp_regs mx6_grp_ioregs = {
	.grp_ddr_type =  0x000C0000,
	.grp_ddrmode_ctl =  0x00020000,
	.grp_ddrpke =  0x00000000,
	.grp_addds =  0x00000030,
	.grp_ctlds =  0x00000030,
	.grp_ddrmode =  0x00020000,
	.grp_b0ds =  0x00000030,
	.grp_b1ds =  0x00000030,
	.grp_b2ds =  0x00000030,
	.grp_b3ds =  0x00000030,
	.grp_b4ds =  0x00000030,
	.grp_b5ds =  0x00000030,
	.grp_b6ds =  0x00000030,
	.grp_b7ds =  0x00000030,
};

const struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 =  0x001F001F,
	.p0_mpwldectrl1 =  0x001F001F,
	.p1_mpwldectrl0 =  0x00440044,
	.p1_mpwldectrl1 =  0x00440044,
	.p0_mpdgctrl0 =  0x434B0350,
	.p0_mpdgctrl1 =  0x034C0359,
	.p1_mpdgctrl0 =  0x434B0350,
	.p1_mpdgctrl1 =  0x03650348,
	.p0_mprddlctl =  0x4436383B,
	.p1_mprddlctl =  0x39393341,
	.p0_mpwrdlctl =  0x35373933,
	.p1_mpwrdlctl =  0x48254A36,
};

const struct mx6_mmdc_calibration mx6dqp_mmcd_calib = {
	.p0_mpwldectrl0 =  0x001B001E,
	.p0_mpwldectrl1 =  0x002E0029,
	.p1_mpwldectrl0 =  0x001B002A,
	.p1_mpwldectrl1 =  0x0019002C,
	.p0_mpdgctrl0 =  0x43240334,
	.p0_mpdgctrl1 =  0x0324031A,
	.p1_mpdgctrl0 =  0x43340344,
	.p1_mpdgctrl1 =  0x03280276,
	.p0_mprddlctl =  0x44383A3E,
	.p1_mprddlctl =  0x3C3C3846,
	.p0_mpwrdlctl =  0x2E303230,
	.p1_mpwrdlctl =  0x38283E34,
};

/* MT41K128M16JT-125 */
static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 1600,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void gpr_init(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* enable AXI cache for VDOA/VPU/IPU */
	writel(0xF00000CF, &iomux->gpr[4]);
	if (is_mx6dqp()) {
		/* set IPU AXI-id1 Qos=0x1 AXI-id0/2/3 Qos=0x7 */
		writel(0x007F007F, &iomux->gpr[6]);
		writel(0x007F007F, &iomux->gpr[7]);
	} else {
		/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
		writel(0x007F007F, &iomux->gpr[6]);
		writel(0x007F007F, &iomux->gpr[7]);
	}
}

/*
 * This section requires the differentiation between iMX6 Sabre boards, but
 * for now, it will configure only for the mx6q variant.
 */
static void spl_dram_init(void)
{
	struct mx6_ddr_sysinfo sysinfo = {
		/* width of data bus:0=16,1=32,2=64 */
		.dsize = 2,
		/* config for full 4GB range so that get_mem_size() works */
		.cs_density = 32, /* 32Gb per CS */
		/* single chip select */
		.ncs = 1,
		.cs1_mirror = 0,
		.rtt_wr = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
		.rtt_nom = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
		.walat = 1,	/* Write additional latency */
		.ralat = 5,	/* Read additional latency */
		.mif3_mode = 3,	/* Command prediction working mode */
		.bi_on = 1,	/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
		.ddr_type = DDR_TYPE_DDR3,
	};

	if (is_mx6dqp()) {
		mx6dq_dram_iocfg(64, &mx6dqp_ddr_ioregs, &mx6_grp_ioregs);
		mx6_dram_cfg(&sysinfo, &mx6dqp_mmcd_calib, &mem_ddr);
	} else {
		mx6dq_dram_iocfg(64, &mx6_ddr_ioregs, &mx6_grp_ioregs);
		mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib, &mem_ddr);
	}
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif