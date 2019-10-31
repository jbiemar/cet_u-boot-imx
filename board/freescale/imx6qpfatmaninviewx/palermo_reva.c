/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <spi.h>
#include <miiphy.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <mxsfb.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "../common/pfuze.h"
#include <usb.h>
#include <usb/ehci-fsl.h>
#include <asm/imx-common/video.h>

#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

#include <command.h>
#include <fs.h>
#include <splash.h>
#include <lcd.h>

DECLARE_GLOBAL_DATA_PTR;

/*
#ifdef CONFIG_SPLASH_SCREEN
static struct splash_location palermo_splash_locations[] = {
	{
		.name = "mmc_fs",
		.storage = SPLASH_STORAGE_MMC,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "1:2",
	},
};

int splash_screen_prepare(void)
{
	return splash_source_load(palermo_splash_locations,
		ARRAY_SIZE(palermo_splash_locations));
}
#endif
*/

//ALTANEOS ADD

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_DAT3_CD_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |	\
	PAD_CTL_PUS_100K_DOWN  | PAD_CTL_SPEED_LOW |		\
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

#define ENET_RX_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

#define WEIM_NOR_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | \
		PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
		PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)


#define FASTGPIO_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define IOX_SDI IMX_GPIO_NR(5, 10)
#define IOX_STCP IMX_GPIO_NR(5, 7)
#define IOX_SHCP IMX_GPIO_NR(5, 11)
#define IOX_OE IMX_GPIO_NR(5, 8)

static iomux_v3_cfg_t const iox_pads[] = {
	/* IOX_SDI */
	MX6_PAD_BOOT_MODE0__GPIO5_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* IOX_SHCP */
	MX6_PAD_BOOT_MODE1__GPIO5_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* IOX_STCP */
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* IOX_nOE */
	MX6_PAD_SNVS_TAMPER8__GPIO5_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/*
 * HDMI_nRST --> Q0
 * ENET1_nRST --> Q1
 * ENET2_nRST --> Q2
 * CAN1_2_STBY --> Q3
 * BT_nPWD --> Q4
 * CSI_RST --> Q5
 * CSI_PWDN --> Q6
 * LCD_nPWREN --> Q7
 */
enum qn {
	HDMI_NRST,
	ENET1_NRST,
	ENET2_NRST,
	CAN1_2_STBY,
	BT_NPWD,
	CSI_RST,
	CSI_PWDN,
	LCD_NPWREN,
};

enum qn_func {
	qn_reset,
	qn_enable,
	qn_disable,
};

enum qn_level {
	qn_low = 0,
	qn_high = 1,
};

/*static enum qn_level seq[3][2] = {
	{0, 1}, {1, 1}, {0, 0}
};

static enum qn_func qn_output[8] = {
	qn_reset, qn_reset, qn_reset, qn_enable, qn_disable, qn_reset,
	qn_disable, qn_disable
};
*/

//ALTANEOS ADD
#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC and EEPROM */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode =  MX6_PAD_UART4_TX_DATA__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_UART4_TX_DATA__GPIO1_IO28 | PC,
		.gp = IMX_GPIO_NR(1, 28),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART4_RX_DATA__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_UART4_RX_DATA__GPIO1_IO29 | PC,
		.gp = IMX_GPIO_NR(1, 29),
	},
};

static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode =  MX6_PAD_UART5_TX_DATA__I2C2_SCL | MUX_PAD_CTRL(I2C2_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART5_TX_DATA__GPIO1_IO30 | MUX_PAD_CTRL(I2C2_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 30),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART5_RX_DATA__I2C2_SDA | MUX_PAD_CTRL(I2C2_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART5_RX_DATA__GPIO1_IO31 | MUX_PAD_CTRL(I2C2_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 31),
	},
};

#ifdef CONFIG_POWER
#define I2C_PMIC       0
int power_init_board(void)
{
	if (0) {
		struct pmic *pfuze;
		int ret;
		unsigned int reg, rev_id;

		ret = power_pfuze3000_init(I2C_PMIC);
		if (ret)
			return ret;

		pfuze = pmic_get("PFUZE3000");
		ret = pmic_probe(pfuze);
		if (ret)
			return ret;

		pmic_reg_read(pfuze, PFUZE3000_DEVICEID, &reg);
		pmic_reg_read(pfuze, PFUZE3000_REVID, &rev_id);
		printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n",
		       reg, rev_id);

		/* disable Low Power Mode during standby mode */
		pmic_reg_read(pfuze, PFUZE3000_LDOGCTL, &reg);
		reg |= 0x1;
		pmic_reg_write(pfuze, PFUZE3000_LDOGCTL, reg);

		/* SW1B step ramp up time from 2us to 4us/25mV */
		reg = 0x40;
		pmic_reg_write(pfuze, PFUZE3000_SW1BCONF, reg);

		/* SW1B mode to APS/PFM */
		reg = 0xc;
		pmic_reg_write(pfuze, PFUZE3000_SW1BMODE, reg);

		/* SW1B standby voltage set to 0.975V */
		reg = 0xb;
		pmic_reg_write(pfuze, PFUZE3000_SW1BSTBY, reg);
	}

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	u32 vddarm;

	struct pmic *p = pmic_get("PFUZE3000");

	if (!p) {
		printf("No PMIC found!\n");
		return;
	}

	/* switch to ldo_bypass mode */
	if (ldo_bypass) {
		prep_anatop_bypass();
		/* decrease VDDARM to 1.275V */
		pmic_reg_read(p, PFUZE3000_SW1BVOLT, &value);
		value &= ~0x1f;
		value |= PFUZE3000_SW1AB_SETP(1275);
		pmic_reg_write(p, PFUZE3000_SW1BVOLT, value);

		set_anatop_bypass(1);
		vddarm = PFUZE3000_SW1AB_SETP(1175);

		pmic_reg_read(p, PFUZE3000_SW1BVOLT, &value);
		value &= ~0x1f;
		value |= vddarm;
		pmic_reg_write(p, PFUZE3000_SW1BVOLT, value);

		finish_anatop_bypass();

		printf("switch to ldo_bypass mode!\n");
	}
}
#endif
#endif
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

//ALTANEOS ADD
static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_UART2_TX_DATA__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART2_RX_DATA__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART3_TX_DATA__UART2_DCE_CTS | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART3_RX_DATA__UART2_DCE_RTS | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart6_pads[] = {
	MX6_PAD_CSI_MCLK__UART6_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI_PIXCLK__UART6_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

//ALTANEOS ADD
static void setup_iomux_uart(void)
{
	printf("setup uart1 iomux\n");
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	printf("setup uart2 iomux\n");
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	printf("setup uart6 iomux\n");
	imx_iomux_v3_setup_multiple_pads(uart6_pads, ARRAY_SIZE(uart6_pads));
}

/*static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	// VSELECT
	MX6_PAD_GPIO1_IO05__USDHC1_VSELECT | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	// CD
	MX6_PAD_UART1_RTS_B__GPIO1_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),
	// RST_B
	MX6_PAD_NAND_WP_B__GPIO4_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
*/

//ALTANEOS ADD
static iomux_v3_cfg_t const usdhc1_emmc_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_READY_B__USDHC1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CE0_B__USDHC1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CE1_B__USDHC1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CLE__USDHC1_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* RST_B */
	MX6_PAD_NAND_WP_B__GPIO4_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
};


static iomux_v3_cfg_t const usdhc2_emmc_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA04__USDHC2_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA05__USDHC2_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA06__USDHC2_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA07__USDHC2_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/*
	 * RST_B
	 */
	MX6_PAD_NAND_ALE__GPIO4_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

//ALTANEOS ADD
#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
    {USDHC1_BASE_ADDR, 0, 8},
	{USDHC2_BASE_ADDR, 0, 8},
};

// ALTANEOS USE PWR GPIO ?
#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 19)
#define USDHC1_PWR_GPIO	IMX_GPIO_NR(4, 11)
#define USDHC2_CD_GPIO	IMX_GPIO_NR(4, 5)
#define USDHC2_PWR_GPIO	IMX_GPIO_NR(4, 10)

int board_mmc_get_env_dev(int devno)
{
	if (devno == 1 && mx6_esdhc_fused(USDHC1_BASE_ADDR))
		devno = 0;

	return devno;
}

int mmc_map_to_kernel_blk(int devno)
{
	if (devno == 0 && mx6_esdhc_fused(USDHC1_BASE_ADDR))
		devno = 1;

	return devno;
}

int board_mmc_getcd(struct mmc *mmc)
{
    struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = 1;
		break;

	case USDHC2_BASE_ADDR:
		ret = 1;
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifdef CONFIG_SPL_BUILD
    int nRet = 0;
	imx_iomux_v3_setup_multiple_pads(usdhc1_emmc_pads, ARRAY_SIZE(usdhc1_emmc_pads));
	imx_iomux_v3_setup_multiple_pads(usdhc2_emmc_pads, ARRAY_SIZE(usdhc2_emmc_pads));

	gpio_direction_output(USDHC1_PWR_GPIO, 0);
	udelay(500);
	gpio_direction_output(USDHC1_PWR_GPIO, 1);
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	nRet = fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
	if (nRet != 0) {
		printf(" --EE: board_mmc_init() EMMC#0 Failed to init");
		return nRet;
	}

	gpio_direction_output(USDHC2_PWR_GPIO, 0);
	udelay(500);
	gpio_direction_output(USDHC2_PWR_GPIO, 1);
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
				usdhc1_emmc_pads, ARRAY_SIZE(usdhc1_emmc_pads));

			gpio_direction_output(USDHC1_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC1_PWR_GPIO, 1);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_emmc_pads, ARRAY_SIZE(usdhc2_emmc_pads));

			gpio_direction_output(USDHC2_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC2_PWR_GPIO, 1);
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

//ALTANEOS ADD ???
#if defined(CONFIG_MXC_SPI) || defined(CONFIG_FSL_QSPI)
static iomux_v3_cfg_t const ecspi2_pads[] = {
	MX6_PAD_CSI_DATA00__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI_DATA03__ECSPI2_MISO   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI_DATA02__ECSPI2_MOSI    | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI_DATA01__GPIO4_IO22     | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define ECSPI2_CS		IMX_GPIO_NR(4, 22)

static void setup_spi(void)
{
	int i;
	printf("setup_spi\n");
	imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));

	for (i = 0; i < 3; i++) {
		enable_spi_clk(1, i);
	}
	gpio_direction_output(ECSPI2_CS, 1);
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 1 && cs == 0) ? (ECSPI2_CS) : -1;
}
#endif

#ifdef CONFIG_FSL_QSPI

#define QSPI_PAD_CTRL1	\
	(PAD_CTL_SRE_FAST | PAD_CTL_SPEED_MED | \
	 PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_47K_UP | PAD_CTL_DSE_120ohm)

static iomux_v3_cfg_t const quadspi_pads[] = {
	MX6_PAD_NAND_WP_B__QSPI_A_SCLK | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_READY_B__QSPI_A_DATA00 | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE0_B__QSPI_A_DATA01 | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE1_B__QSPI_A_DATA02 | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CLE__QSPI_A_DATA03 | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DQS__QSPI_A_SS0_B | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
};

static int board_qspi_init(void)
{
	/* Set the iomux */
	imx_iomux_v3_setup_multiple_pads(quadspi_pads,
					 ARRAY_SIZE(quadspi_pads));
	/* Set the clock */
	enable_qspi_clk(0);

	return 0;
}
#endif



#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

/* At default the 3v3 enables the MIC2026 for VBUS power */
static void setup_usb(void)
{

}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}
#endif

//ALTANEOS ADD CHECK LE UNDEF
#undef CONFIG_FEC_MXC
#ifdef CONFIG_FEC_MXC
/*
 * pin conflicts for fec1 and fec2, GPIO1_IO06 and GPIO1_IO07 can only
 * be used for ENET1 or ENET2, cannot be used for both.
 */
static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const fec2_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET2_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET2_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA0__ENET2_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ENET2_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET2_TX_EN__ENET2_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA0__ENET2_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA1__ENET2_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_EN__ENET2_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_ER__ENET2_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
};


static iomux_v3_cfg_t const fec_rst_pads[] = {
	MX6_PAD_LCD_DATA21__GPIO3_IO26 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_LCD_DATA22__GPIO3_IO27 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

#define ENET1_nRST IMX_GPIO_NR(3, 27)
#define ENET2_nRST IMX_GPIO_NR(3, 26)

static void setup_iomux_fec_rst(void)
{
	imx_iomux_v3_setup_multiple_pads(fec_rst_pads,
						ARRAY_SIZE(fec_rst_pads));
}

static void setup_iomux_fec(int fec_id)
{

	if (fec_id == 0)
		imx_iomux_v3_setup_multiple_pads(fec1_pads,
						ARRAY_SIZE(fec1_pads));
	else
		imx_iomux_v3_setup_multiple_pads(fec2_pads,
						ARRAY_SIZE(fec2_pads));
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_fec(CONFIG_FEC_ENET_DEV);

	return fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
				       CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
}

static void fec_reset(void)
{
	gpio_direction_output(ENET1_nRST, 0);
	gpio_direction_output(ENET2_nRST, 0);
	udelay(500);
	gpio_direction_output(ENET1_nRST, 1);
	gpio_direction_output(ENET2_nRST, 1);
}

//ALTANEOS ADD
static int setup_fec(int fec_id)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	if (fec_id == 0) {
		if (check_module_fused(MX6_MODULE_ENET1))
			return -1;

		/*
		 * Use 50M anatop loopback REF_CLK1 for ENET1,
		 * clear gpr1[13], set gpr1[17].
		 */
		clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
				IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
	} else {
		if (check_module_fused(MX6_MODULE_ENET2))
			return -1;

		/*
		 * Use 50M anatop loopback REF_CLK2 for ENET2,
		 * clear gpr1[14], set gpr1[18].
		 */
		clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
				IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);
	}

	ret = enable_fec_anatop_clock(fec_id, ENET_50MHZ);
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
#endif

//ALTANEOS ADD
#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_LCD_CLK__LCDIF_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_ENABLE__LCDIF_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_HSYNC__LCDIF_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_VSYNC__LCDIF_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA00__LCDIF_DATA00 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA01__LCDIF_DATA01 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA02__LCDIF_DATA02 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA03__LCDIF_DATA03 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA04__LCDIF_DATA04 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA05__LCDIF_DATA05 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA06__LCDIF_DATA06 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA07__LCDIF_DATA07 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA08__LCDIF_DATA08 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA09__LCDIF_DATA09 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA10__LCDIF_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA11__LCDIF_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA12__LCDIF_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA13__LCDIF_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA14__LCDIF_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA15__LCDIF_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA16__LCDIF_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA17__LCDIF_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA18__LCDIF_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA19__LCDIF_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA20__LCDIF_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA21__LCDIF_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA22__LCDIF_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA23__LCDIF_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	/* LCD_PWR */
	MX6_PAD_CSI_DATA07__GPIO4_IO28 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_DOWN),

	/* LCD_PWM */
	MX6_PAD_GPIO1_IO08__GPIO1_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};


void do_enable_parallel_lcd(struct display_info_t const *dev)
{
	enable_lcdif_clock(dev->bus);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

}

//ALTANEOS ADD
struct display_info_t const displays[] = {
	{
	.bus = MX6UL_LCDIF1_BASE_ADDR,
	.addr = 0,
	.pixfmt = 18,
	.detect = NULL,
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

//ALTANEOS ADD
static iomux_v3_cfg_t const touch_pads[] = {
	/* TS_IRQ*/
	MX6_PAD_CSI_DATA05__GPIO4_IO26 | MUX_PAD_CTRL(FASTGPIO_PAD_CTRL),
	/* TS_nRST */
	MX6_PAD_SNVS_TAMPER3__GPIO5_IO03 | MUX_PAD_CTRL(FASTGPIO_PAD_CTRL),
};

#define TOUCH_IRQ_GPIO		IMX_GPIO_NR(4, 26)
#define TOUCH_nRST_GPIO		IMX_GPIO_NR(5, 3)

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

//ALTANEOS ADD
static iomux_v3_cfg_t const leds_pads[] = {
	/* red */
	MX6_PAD_GPIO1_IO09__GPIO1_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* yellow */
	MX6_PAD_GPIO1_IO05__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* green */
	MX6_PAD_CSI_VSYNC__GPIO4_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* buzzer */
	MX6_PAD_CSI_HSYNC__GPIO4_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* debug */
	MX6_PAD_ENET2_TX_CLK__GPIO2_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define LED_RED_GPIO		IMX_GPIO_NR(1, 9)
#define LED_YELLOW_GPIO		IMX_GPIO_NR(1, 5)
#define LED_GREEN_GPIO		IMX_GPIO_NR(4, 19)
#define LED_BUZZER_GPIO		IMX_GPIO_NR(4, 20)
#define LED_DEBUG_GPIO		IMX_GPIO_NR(2, 14)

//ALTANEOS ADD
int setup_led(void)
{
	printf("%s\n", __FUNCTION__);

	imx_iomux_v3_setup_multiple_pads(
		leds_pads, ARRAY_SIZE(leds_pads));

	gpio_direction_output(LED_GREEN_GPIO, 1);
	gpio_direction_output(LED_YELLOW_GPIO, 1);
	gpio_direction_output(LED_RED_GPIO, 1);
	gpio_direction_output(LED_DEBUG_GPIO, 1);
	gpio_direction_output(LED_BUZZER_GPIO, 1);
	mdelay(500);
	gpio_direction_output(LED_BUZZER_GPIO, 0);
	mdelay(1500);
	gpio_direction_output(LED_GREEN_GPIO, 0);
	gpio_direction_output(LED_YELLOW_GPIO, 0);
	gpio_direction_output(LED_RED_GPIO, 0);
	gpio_direction_output(LED_DEBUG_GPIO, 0);
	/* Testing ... */
	mdelay(500);
	gpio_set_value(LED_DEBUG_GPIO, 0);
	mdelay(500);
	gpio_set_value(LED_DEBUG_GPIO, 1);

	return 0;
}

//ALTANEOS ADD
static iomux_v3_cfg_t const ninja_pads[] = {
	/* boot */
	MX6_PAD_GPIO1_IO00__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* nreset */
	MX6_PAD_CSI_DATA04__GPIO4_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define NINJA_BOOT_GPIO		IMX_GPIO_NR(1, 0)
#define NINJA_nRST_GPIO		IMX_GPIO_NR(4, 25)

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
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* din1 */
	MX6_PAD_SNVS_TAMPER8__GPIO5_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* relay0 */
	MX6_PAD_SNVS_TAMPER1__GPIO5_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* relay1 */
	MX6_PAD_SNVS_TAMPER4__GPIO5_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define DIN0_GPIO	IMX_GPIO_NR(5,7)
#define DIN1_GPIO	IMX_GPIO_NR(5,8)
#define RELAY0_GPIO	IMX_GPIO_NR(5,1)
#define RELAY1_GPIO	IMX_GPIO_NR(5,4)


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

// ALTANEOS ADD
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

	if (strcmp(getenv("touch"), "normal") == 0) {
		snprintf(cmdarg, sizeof(cmdarg), "load mmc 1:5 %s /data/default/splash-normal.bmp", strldaddr);
		run_command(cmdarg, 0);
		ret = bmp_display(ldaddr, 0, 0);
	} else if (strcmp(getenv("touch"), "tiny") == 0) {
		snprintf(cmdarg, sizeof(cmdarg), "load mmc 1:5 %s /data/default/splash-tiny.bmp", strldaddr);
		run_command(cmdarg, 0);
		ret = bmp_display(ldaddr, 0, 0);
	} else if (strcmp(getenv("touch"), "big") == 0) {
		snprintf(cmdarg, sizeof(cmdarg), "load mmc 1:5 %s /data/default/splash-big.bmp", strldaddr);
		run_command(cmdarg, 0);
		ret = bmp_display(ldaddr, 240, 120);
	}
	return ret;
}


int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

//ALTANEOS ADD
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
	if (strcmp(getenv("touch"), "big") == 0) {
		if (i2c_read(EEPROM_MAC_DEVADDR1, EEPROM_MAC_REG_MAC, 1, buf, 6) == 0) {
			sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
				buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
			/* Check if MAC address is valid - Must match the at24mac prefix*/
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
	}

	if (!getenv("eth1addr"))
		printf("2nd MAC address not set\n");

	return 0;
}


int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	imx_iomux_v3_setup_multiple_pads(iox_pads, ARRAY_SIZE(iox_pads));

	/*setup_iomux_fec_rst();
	fec_reset();*/
	setup_dio();

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_touch();
#endif

#ifdef	CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

#if defined(CONFIG_MXC_SPI) || defined(CONFIG_FSL_QSPI)
	setup_spi();
#endif

#ifdef CONFIG_FSL_QSPI
	printf("Init QSPI\n");
	board_qspi_init();
#endif

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand();
#endif

#if defined(CONFIG_MXC_SPI) || defined(CONFIG_FSL_QSPI)
//	setup_ili9341_rgbmode();
#endif

	setup_led();
	setup_ninja();

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x42, 0x20, 0x00, 0x00)},
	{"sd2", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	#ifdef CONFIG_FSL_QSPI
	{"qspi1", MAKE_CFGVAL(0x10, 0x00, 0x00, 0x00)},
	#endif
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
	setenv("board_name", "PALERMO");
	setenv("board_rev", "REVA");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	setup_splash();

	return 0;
}

int checkboard(void)
{
	puts("Board: MX6UL PALERMO REVA\n");

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc0");
		break;
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc1");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
#if defined(CONFIG_FASTBOOT_STORAGE_NAND)
	case NAND_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "nand");
		if (!getenv("fbparts"))
			setenv("fbparts", ANDROID_FASTBOOT_NAND_PARTS);
		if (!getenv("bootcmd"))
			setenv("bootcmd",
				"nand read ${loadaddr} ${boot_nand_offset} "
				"${boot_nand_size};boota ${loadaddr}");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_NAND*/

	default:
		printf("unsupported boot devices\n");
		break;
	}
}

#ifdef CONFIG_ANDROID_RECOVERY
int check_recovery_cmd_file(void)
{
	int recovery_mode = 0;

	recovery_mode = recovery_check_and_clean_flag();

	return recovery_mode;
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc0 recovery");
		break;
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc1 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
#if defined(CONFIG_FASTBOOT_STORAGE_NAND)
	case NAND_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"nand read ${loadaddr} ${recovery_nand_offset} "
				"${recovery_nand_size};boota ${loadaddr}");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_NAND*/

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
#include <libfdt.h>
#include <spl.h>
#include <asm/arch/mx6-ddr.h>


static struct mx6ul_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000030,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_ctlds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x000c0000,
};

static struct mx6ul_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_ras = 0x00000030,
	.dram_cas = 0x00000030,
	.dram_odt0 = 0x00000030,
	.dram_odt1 = 0x00000030,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000008,
	.dram_sdqs0 = 0x00000038,
	.dram_sdqs1 = 0x00000030,
	.dram_reset = 0x00000030,
};

static struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 = 0x00070007,
	.p0_mpdgctrl0 = 0x41490145,
	.p0_mprddlctl = 0x40404546,
	.p0_mpwrdlctl = 0x4040524D,
};

struct mx6_ddr_sysinfo ddr_sysinfo = {
	.dsize = 0,
	.cs_density = 20,
	.ncs = 1,
	.cs1_mirror = 0,
	.rtt_wr = 2,
	.rtt_nom = 1,		/* RTT_Nom = RZQ/2 */
	.walat = 1,		/* Write additional latency */
	.ralat = 5,		/* Read additional latency */
	.mif3_mode = 3,		/* Command prediction working mode */
	.bi_on = 1,		/* Bank interleaving enabled */
	.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.ddr_type = DDR_TYPE_DDR3,
};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 800,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

static void spl_dram_init(void)
{
	mx6ul_dram_iocfg(mem_ddr.width, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&ddr_sysinfo, &mx6_mmcd_calib, &mem_ddr);
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

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
