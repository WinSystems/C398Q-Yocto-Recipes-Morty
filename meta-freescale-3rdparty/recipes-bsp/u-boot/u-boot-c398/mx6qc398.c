/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2016 WinSystems, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#ifdef CONFIG_SYS_I2C_MXC
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#endif
#ifdef CONFIG_PCA953X
#include <pca953x.h>
#endif
#ifdef CONFIG_FEC_MXC
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <micrel.h>
#endif
#ifdef CONFIG_CMD_SATA
#include <asm/imx-common/sata.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |   \
    PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | \
    PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |    \
    PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | \
    PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |   \
    PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
    PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |    \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |   \
    PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
    PAD_CTL_ODE | PAD_CTL_SRE_FAST)

void udc_pins_setting(void);
void board_gpio_init(void);

/* setup boot device */
enum boot_device ws_get_boot_device(void)
{
    enum boot_device boot_dev = UNKNOWN_BOOT;

    /* SRC_BASE_ADDR = 0x020D8000 */
    uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);	/* 0x020D8004 = SRC Boot Mode Register 1 (SRC_SBMR1) */
    uint bt_mem_ctl = (soc_sbmr & 0x00000060) >> 5; // BOOT_CFG1[6:5]
    uint bt_mem_type = (soc_sbmr & 0x00000010) >> 4; // BOOT_CFG1[4]

    switch (bt_mem_ctl) {
        case 0x1:
            if (bt_mem_type)
                boot_dev = SPI_NOR_BOOT;
            else
                boot_dev = SATA_BOOT;
            break;

        case 0x2:
            boot_dev = SD1_BOOT;
            break;

        case 0x3:
            boot_dev = MMC1_BOOT;
            break;

        default:
            boot_dev = UNKNOWN_BOOT;
            break;
    }

    return boot_dev;
}

int checkboard(void)
{
    printf("Board: SBC35-C398Q\n");

    printf("Boot Device: ");

    switch (ws_get_boot_device()) {
        case SPI_NOR_BOOT:
            printf("SPI-NOR\n");
            break;

        case SD1_BOOT:
            printf("SD\n");
            break;

        case MMC1_BOOT:
        printf("MMC\n");
            break;

        case UNKNOWN_BOOT:
        default:
            printf("UNKNOWN\n");
            break;
    }

    return 0;
}

/* setup memory */
int dram_init(void)
{
    gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

    return 0;
}

/* setup uart 1-5 */
iomux_v3_cfg_t const uart1_pads[] = {
    MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D19__UART1_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D20__UART1_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D23__UART1_DCD_B | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_EB3__UART1_RI_B | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D24__UART1_DTR_B | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D25__UART1_DSR_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart2_pads[] = {
    MX6_PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D28__UART2_DTE_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D29__UART2_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart3_pads[] = {
    MX6_PAD_SD4_CMD__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_SD4_CLK__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D30__UART3_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D31__UART3_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart4_pads[] = {
    MX6_PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_CSI0_DAT17__UART4_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_CSI0_DAT16__UART4_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart5_pads[] = {
    MX6_PAD_KEY_COL1__UART5_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_KEY_ROW1__UART5_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_KEY_ROW4__UART5_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_KEY_COL4__UART5_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_uart(void)
{
    imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
    imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
    imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
    imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
    imx_iomux_v3_setup_multiple_pads(uart5_pads, ARRAY_SIZE(uart5_pads));
}

/* setup i2c */
#ifdef CONFIG_SYS_I2C_MXC

iomux_v3_cfg_t const i2c1_pads[] = {
    MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
    MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
};

iomux_v3_cfg_t const i2c2_pads[] = {
    MX6_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
    MX6_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
};

iomux_v3_cfg_t const i2c3_pads[] = {
    MX6_PAD_GPIO_6__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
    MX6_PAD_GPIO_3__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
};

static void ws_setup_i2c(unsigned int module_base)
{
    unsigned int reg;

    switch (module_base) {
        case I2C1_BASE_ADDR:
            /* i2c1 #1 */
            imx_iomux_v3_setup_multiple_pads(i2c1_pads, ARRAY_SIZE(i2c1_pads));

            /* Enable i2c clock */
            reg = readl(CCM_CCGR2);
            reg |= 0xC0;
            writel(reg, CCM_CCGR2);

            break;
    
        case I2C2_BASE_ADDR:
            /* i2c1 #2 */
            imx_iomux_v3_setup_multiple_pads(i2c2_pads, ARRAY_SIZE(i2c2_pads));

            /* Enable i2c clock */
            reg = readl(CCM_CCGR2);
            reg |= 0x300;
            writel(reg, CCM_CCGR2);

            break;

        case I2C3_BASE_ADDR:
            /* i2c1 #3 */
            imx_iomux_v3_setup_multiple_pads(i2c3_pads, ARRAY_SIZE(i2c3_pads));

            /* Enable i2c clock */
            reg = readl(CCM_CCGR2);
            reg |= 0xC00;
            writel(reg, CCM_CCGR2);

            break;

        default:
            printf("Invalid I2C base: 0x%x\n", module_base);
            break;
    }
}

#ifdef CONFIG_PCA953X

/* Define for building port exp gpio, pin starts from 0 */
#define PORTEXP_IO_NR(chip, pin) \
	((chip << 5) + pin)

/* Get the chip addr from ioexp gpio */
#define PORTEXP_IO_TO_CHIP(gpio_nr) \
	(gpio_nr >> 5)

/* Get the pin number from ioexp gpio */
#define PORTEXP_IO_TO_PIN(gpio_nr) \
	(gpio_nr & 0x1f)

static int port_exp_direction_output(unsigned gpio, int value)
{
    int ret;

    i2c_set_bus_num(0);
    ret = i2c_probe(PORTEXP_IO_TO_CHIP(gpio));
    if (ret)
        return ret;

    ret = pca953x_set_dir(PORTEXP_IO_TO_CHIP(gpio),
        (1 << PORTEXP_IO_TO_PIN(gpio)),
        (PCA953X_DIR_OUT << PORTEXP_IO_TO_PIN(gpio)));

    if (ret)
        return ret;

    ret = pca953x_set_val(PORTEXP_IO_TO_CHIP(gpio),
        (1 << PORTEXP_IO_TO_PIN(gpio)),
        (value << PORTEXP_IO_TO_PIN(gpio)));

    if (ret)
        return ret;

    return 0;
}

static void setup_max7310(void)
{
    /* max7310 address 0x30 */
    port_exp_direction_output(UART1_SLEW, 0);    /* 248 */
    port_exp_direction_output(UART1_TERM, 0);    /* 249 */
    port_exp_direction_output(UART1_M1, 0);      /* 250 */
    port_exp_direction_output(UART1_M0, 1);      /* 251 */
    port_exp_direction_output(UART2_SLEW, 0);    /* 252 */
    port_exp_direction_output(UART2_TERM, 0);    /* 253 */
    port_exp_direction_output(UART2_M1, 0);      /* 254 */
    port_exp_direction_output(UART2_M0, 1);      /* 255 */

    /* max7310 address 0x32 */
    port_exp_direction_output(RX485EN3_EN, 0);    /* 240 */
    port_exp_direction_output(HD3_TE, 0);         /* 241 */
    port_exp_direction_output(RX422EN3_EN, 1);    /* 242 */
    port_exp_direction_output(FD3_TE, 0);         /* 243 */
    port_exp_direction_output(EXP_RST_N, 1);      /* 244 */
    port_exp_direction_output(PCIE_RST_N, 1);     /* 245 */
    port_exp_direction_output(PCIE_DIS_N, 1);     /* 246 */
    port_exp_direction_output(CAM_MIPI_RST_N, 1); /* 247 */

    /* max7310 address 0x34 */
    port_exp_direction_output(HD4_TE, 0);         /* 232 */
    port_exp_direction_output(RX485EN4_EN, 0);    /* 233 */
    port_exp_direction_output(RX485EN5_EN, 0);    /* 234 */
    port_exp_direction_output(HD5_TE, 0);         /* 235 */
    port_exp_direction_output(FD5_TE, 0);         /* 236 */
    port_exp_direction_output(RX422EN5_EN, 1);    /* 237 */
    port_exp_direction_output(RX422EN4_EN, 1);    /* 238 */
    port_exp_direction_output(FD4_TE, 0);         /* 239 */
}

#endif /* CONFIG_PCA953X */

#ifdef CONFIG_LDO_BYPASS_CHECK

void ldo_mode_set(int ldo_bypass)
{
    unsigned char value = 0x2b;

    /* pmic regs 20h and 2eh are being changed during kernel reboot
       causing sytem lock, these writes reset to proper values */
    if (i2c_write(0x8, 0x20, 1, &value, 1)) {
        printf("Set SW1AB error!\n");
        return;
    }

    if (i2c_write(0x8, 0x2e, 1, &value, 1)) {
        printf("Set SW1C error!\n");
        return;
    }
}

#endif /* CONFIG_LDO_BYPASS_CHECK */

#endif /* CONFIG_SYS_I2C_MXC */

/* setup spinor */
#ifdef CONFIG_MXC_SPI

iomux_v3_cfg_t const ecspi1_pads[] = {
    MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_EB2__GPIO2_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL),     /* ss0 */
};

static void setup_spinor(void)
{
    imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
    /* gpio_direction_output(IMX_GPIO_NR(2, 30), 0); */
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(2,30)) : -1;
}

#endif /* CONFIG_MXC_SPI */

/* setup sd */
#ifdef CONFIG_CMD_MMC

#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 1)

struct fsl_esdhc_cfg usdhc_cfg[2] = {
    {USDHC1_BASE_ADDR},
    {USDHC2_BASE_ADDR}
};

iomux_v3_cfg_t const usdhc1_pads[] = {
    MX6_PAD_SD1_CLK__SD1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_CMD__SD1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_DAT0__SD1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_DAT1__SD1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_DAT2__SD1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD1_DAT3__SD1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_GPIO_1__SD1_CD_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc2_pads[] = {
    MX6_PAD_SD2_CLK__SD2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_CMD__SD2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};


//int mmc_get_env_devno(void)
int board_mmc_get_env_dev(void)
{
    u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);

    /* BOOT_CFG2[3] - 0 = uSD, 1 = SD */
    return (soc_sbmr & 0x00000800) >> 11;
}

int board_mmc_getcd(struct mmc *mmc)
{
    struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
    int ret = 0;

    switch (cfg->esdhc_base) {
        case USDHC1_BASE_ADDR:
            ret = !gpio_get_value(USDHC1_CD_GPIO);
          break;

        case USDHC2_BASE_ADDR:
           ret = 1;
           break;
    }

    return ret;
}

int board_mmc_init(bd_t *bis)
{
    s32 status = 0;
    u32 index = 0;

    /* According to the board_mmc_init() the following map is done: */
    /* (U-boot device node)    (Physical Port) */
    /*    mmc0                    SD0 */
    /*    mmc1                    SD1 */
    for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; index++)
    {
        switch (index) {
            case 0:
                imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
                gpio_direction_input(USDHC1_CD_GPIO);
                usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
                break;

            case 1:
                imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
                usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
                break;

            default:
                printf("Warning: you configured more USDHC controllers"
                    "(%d) then supported by the board (%d)\n",
                    index + 1, CONFIG_SYS_FSL_USDHC_NUM);
                return status;
        }

        status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
    }

    return status;
}

/*
int check_mmc_autodetect(void)
{
    char *autodetect_str = getenv("mmcautodetect");

    if ((autodetect_str != NULL) &&
        (strcmp(autodetect_str, "yes") == 0)) {
        return 1;
    }

    return 0;
}
*/

#endif /* CONFIG_CMD_MMC */

/* setup ethernet */
#ifdef CONFIG_FEC_MXC

int mx6_rgmii_rework(struct phy_device *phydev)
{
    unsigned short value;

    phy_write(phydev, MDIO_DEVAD_NONE, MII_CTRL1000, 0x1c00);

    /* ctrl pad skew - reg idx 0x2 addr 0x4 */
    ksz9031_phy_extended_write(phydev, 0x02,
        MII_KSZ9031_EXT_RGMII_CTRL_SIG_SKEW, 
        MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
    
    /* rx pad skew - reg idx 0x2 addr 0x5 */
    ksz9031_phy_extended_write(phydev, 0x02,
        MII_KSZ9031_EXT_RGMII_RX_DATA_SKEW,
        MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
    
    /* tx pad skew - reg idx 0x2 addr 0x6 */
    ksz9031_phy_extended_write(phydev, 0x02,
        MII_KSZ9031_EXT_RGMII_TX_DATA_SKEW,
        MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0000);
    
    /* clock pad skew - reg idx 0x2 addr 0x8 */
    ksz9031_phy_extended_write(phydev, 0x02,
        MII_KSZ9031_EXT_RGMII_CLOCK_SKEW,
        MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x03FF);

    /* enable reg 0 for auto-negotiation */
    value = phy_read(phydev, MDIO_DEVAD_NONE, 0x0);

    /* 100 Mbps	 */
    value |= 0x3200;
    value &= 0xFFBF;
    phy_write(phydev, MDIO_DEVAD_NONE, MII_BMCR, value);

    return 0;
}

int board_phy_config(struct phy_device *phydev)
{
    mx6_rgmii_rework(phydev);

    if (phydev->drv->config)
        phydev->drv->config(phydev);

    return 0;
}

iomux_v3_cfg_t const enet_pads[] = {
    MX6_PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TXC__RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD0__RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD1__RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD2__RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD3__RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RXC__RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD0__RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD1__RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD2__RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD3__RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_TX_EN__GPIO1_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),  /* Micrel RGMII Phy Interrupt */
    MX6_PAD_EIM_BCLK__GPIO6_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),  /* Micrel RGMII Reset */
};

int board_eth_init(bd_t *bis)
{
    uint32_t base = IMX_FEC_BASE;
    struct mii_dev *bus = NULL;
    struct phy_device *phydev = NULL;
    int ret;

    imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

    gpio_direction_output(IMX_GPIO_NR(6, 31), 0);  /* assert PHY rst */
    udelay(1000);
    gpio_set_value(IMX_GPIO_NR(6, 31), 1);  /* deassert PHY rst */

    /* Need 100ms delay to exit from reset. */
    udelay(1000 * 100);

    bus = fec_get_miibus(base, -1);
    if (!bus)
        return 0;

    /* scan phy 4,5,6,7 */
    phydev = phy_find_by_mask(bus, (0xf << 4), PHY_INTERFACE_MODE_RGMII);

    if (!phydev) {
        free(bus);
        return 0;
    }
    //printf("using phy at %d\n", phydev->addr);
    ret = fec_probe(bis, -1, base, bus, phydev);
    if (ret) {
        printf("FEC MXC: %s:failed\n", __func__);
        free(phydev);
        free(bus);
    }

    return 0;
}

#endif /* CONFIG_FEC_MXC */

/* setup gpio */
iomux_v3_cfg_t const gpio_pads[] = {
    MX6_PAD_GPIO_4__GPIO1_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL),          /* lvdden */
    MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),          /* lbklt_en0 */
    MX6_PAD_ENET_RXD1__GPIO1_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* user_led */
    MX6_PAD_ENET_RXD0__GPIO1_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* can2_silent */
    MX6_PAD_ENET_TXD1__GPIO1_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* can1_silent */
    MX6_PAD_DI0_DISP_CLK__GPIO4_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL),    /* dio0 */
    MX6_PAD_DI0_PIN15__GPIO4_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* dio1 */
    MX6_PAD_DI0_PIN2__GPIO4_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL),        /* dio2 */
    MX6_PAD_DI0_PIN3__GPIO4_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),        /* dio3 */
    MX6_PAD_DI0_PIN4__GPIO4_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),        /* dio4 */
    MX6_PAD_DISP0_DAT10__GPIO4_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),     /* dio5 */
    MX6_PAD_DISP0_DAT11__GPIO5_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),     /* dio6 */
    MX6_PAD_DISP0_DAT12__GPIO5_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL),     /* dio7 */
    MX6_PAD_DISP0_DAT13__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),     /* dio8 */
    MX6_PAD_DISP0_DAT14__GPIO5_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),     /* dio9 */
    MX6_PAD_CSI0_PIXCLK__GPIO5_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL),     /* dio10 */
    MX6_PAD_CSI0_MCLK__GPIO5_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* dio11 */
    MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),    /* dio12 */
    MX6_PAD_CSI0_VSYNC__GPIO5_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* dio13 */
    MX6_PAD_CSI0_DAT4__GPIO5_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* dio14 */
    MX6_PAD_CSI0_DAT5__GPIO5_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* dio15 */
    MX6_PAD_CSI0_DAT6__GPIO5_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* dio16 */
    MX6_PAD_CSI0_DAT7__GPIO5_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* dio17 */
    MX6_PAD_CSI0_DAT12__GPIO5_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* dio18 */
    MX6_PAD_CSI0_DAT13__GPIO5_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* dio19 */
    MX6_PAD_CSI0_DAT14__GPIO6_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* dio20 */
    MX6_PAD_CSI0_DAT15__GPIO6_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* dio21 */
    MX6_PAD_CSI0_DAT18__GPIO6_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* dio22 */
    MX6_PAD_CSI0_DAT19__GPIO6_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),      /* dio23 */
    MX6_PAD_NANDF_CLE__GPIO6_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* usb_otg_pwr_en */
    MX6_PAD_NANDF_ALE__GPIO6_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* user_jmpr */
    MX6_PAD_NANDF_CS0__GPIO6_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* pmic_int_n */
    MX6_PAD_NANDF_CS1__GPIO6_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* usb_otg_oc */
    MX6_PAD_NANDF_CS2__GPIO6_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),       /* usb_hub_reset_n */
    MX6_PAD_SD3_DAT5__GPIO7_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),        /* gpio7_0 */
    MX6_PAD_SD3_DAT4__GPIO7_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),        /* gpio7_1 */
    MX6_PAD_SD3_CMD__GPIO7_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),         /* gpio7_2 */
    MX6_PAD_SD3_CLK__GPIO7_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),         /* gpio7_3 */
    MX6_PAD_SD3_DAT2__GPIO7_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL),        /* gpio7_6 */
    MX6_PAD_SD3_DAT3__GPIO7_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),        /* gpio7_7 */
    MX6_PAD_SD3_RST__GPIO7_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),         /* gpio7_8 */
    MX6_PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),         /* gpio7_11 */
    MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),         /* exp_int */
    MX6_PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL),         /* lbklt_en1 */
};

void board_gpio_init(void)
{
    imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));

    /* gpio1 */
    /* output bits: 29,27,26,5,4 */
    gpio_direction_output(IMX_GPIO_NR(1, 29), 0);   /* 29 - can1_silent */
    gpio_direction_output(IMX_GPIO_NR(1, 27), 0);   /* 27 - can2_silent */
    gpio_direction_output(IMX_GPIO_NR(1, 26), 0);   /* 26 - user_led */
    gpio_direction_output(IMX_GPIO_NR(1, 5), 1);    /* 5 - lbklt_en0 */
    gpio_direction_output(IMX_GPIO_NR(1, 4), 1);    /* 4 - lvdden */

    /* gpio6 */
    /* input bits: 14,11,8 */
    /* output bits: 31,15,7 */
    gpio_direction_output(IMX_GPIO_NR(6, 31), 1);   /* 191 - rgmii_reset_n */
    gpio_direction_output(IMX_GPIO_NR(6, 15), 1);   /* 175 - usb_hub_reset_n */
    gpio_direction_input(IMX_GPIO_NR(6, 14));       /* 174 - usb_otg_oc */
    gpio_direction_input(IMX_GPIO_NR(6, 11));       /* 171 - pmic_int_n */
    gpio_direction_input(IMX_GPIO_NR(6, 8));        /* 168 - user_jmpr */
    gpio_direction_output(IMX_GPIO_NR(6, 7), 1);    /* 167 - usb_otg_pwr_en */

    /* gpio7 */
    /* output bits: 13 */
    gpio_direction_output(IMX_GPIO_NR(7, 13), 1);   /* 205 - lbklt_en1 */
}

#if defined(CONFIG_VIDEO_IPUV3)

/* setup ipu */
iomux_v3_cfg_t const ipu_pads[] = {
    MX6_PAD_EIM_A16__IPU2_CSI1_PIXCLK,
    MX6_PAD_EIM_A24__IPU2_CSI1_DATA19,
    MX6_PAD_EIM_A23__IPU2_CSI1_DATA18,
    MX6_PAD_EIM_A22__IPU2_CSI1_DATA17,
    MX6_PAD_EIM_A21__IPU2_CSI1_DATA16,
    MX6_PAD_EIM_A20__IPU2_CSI1_DATA15,
    MX6_PAD_EIM_A19__IPU2_CSI1_DATA14,
    MX6_PAD_EIM_A18__IPU2_CSI1_DATA13,
    MX6_PAD_EIM_A17__IPU2_CSI1_DATA12,
    MX6_PAD_EIM_D21__IPU2_CSI1_DATA11,
    MX6_PAD_EIM_D22__IPU2_CSI1_DATA10,
    MX6_PAD_EIM_DA0__IPU2_CSI1_DATA09,
    MX6_PAD_EIM_DA1__IPU2_CSI1_DATA08,
    MX6_PAD_EIM_DA2__IPU2_CSI1_DATA07,
    MX6_PAD_EIM_DA3__IPU2_CSI1_DATA06,
    MX6_PAD_EIM_DA4__IPU2_CSI1_DATA05,
    MX6_PAD_EIM_DA5__IPU2_CSI1_DATA04,
    MX6_PAD_EIM_DA6__IPU2_CSI1_DATA03,
    MX6_PAD_EIM_DA7__IPU2_CSI1_DATA02,
    MX6_PAD_EIM_DA8__IPU2_CSI1_DATA01,
    MX6_PAD_EIM_DA9__IPU2_CSI1_DATA00,
    MX6_PAD_EIM_DA10__IPU2_CSI1_DATA_EN,
    MX6_PAD_EIM_DA11__IPU2_CSI1_HSYNC,
    MX6_PAD_EIM_DA12__IPU2_CSI1_VSYNC,
};

iomux_v3_cfg_t const backlight_pads[] = {
    MX6_PAD_GPIO_9__PWM1_OUT,
    MX6_PAD_DISP0_DAT9__PWM2_OUT,
};

struct display_info_t {
    int	bus;
    int	addr;
    int	pixfmt;
    int(*detect)(struct display_info_t const *dev);
    void(*enable)(struct display_info_t const *dev);
    struct fb_videomode mode;
};

static void do_enable_hdmi(struct display_info_t const *dev)
{
    imx_enable_hdmi_phy();
}

static struct display_info_t const displays[] = { {
        .bus = -1,
        .addr = 0,
        .pixfmt = IPU_PIX_FMT_RGB666,
        .detect = NULL,
        .enable = NULL,
        .mode = {
            .name = "LDB-XGA",
            .refresh = 60,
            .xres = 800,
            .yres = 480,
            .pixclock = 15385,
            .left_margin = 220,
            .right_margin = 40,
            .upper_margin = 21,
            .lower_margin = 7,
            .hsync_len = 60,
            .vsync_len = 10,
            .sync = FB_SYNC_EXT,
            .vmode = FB_VMODE_NONINTERLACED
        } }, {
        .bus = -1,
        .addr = 0,
        .pixfmt = IPU_PIX_FMT_RGB666,
        .detect = NULL,
        .enable = NULL,
        .mode = {
            .name = "LDB-XGA",
            .refresh = 60,
            .xres = 800,
            .yres = 480,
            .pixclock = 15385,
            .left_margin = 220,
            .right_margin = 40,
            .upper_margin = 21,
            .lower_margin = 7,
            .hsync_len = 60,
            .vsync_len = 10,
            .sync = FB_SYNC_EXT,
            .vmode = FB_VMODE_NONINTERLACED
        } }, {
        .bus = -1,
        .addr = 0,
        .pixfmt = IPU_PIX_FMT_RGB24,
        .detect = NULL,
        .enable = do_enable_hdmi,
        .mode = {
            .name = "HDMI",
            .refresh = 60,
            .xres = 640,
            .yres = 480,
            .pixclock = 39721,
            .left_margin = 48,
            .right_margin = 16,
            .upper_margin = 33,
            .lower_margin = 10,
            .hsync_len = 96,
            .vsync_len = 2,
            .sync = 0,
            .vmode = FB_VMODE_NONINTERLACED
    } } };

int board_video_skip(void)
{
    return 0;
}

static void setup_display(void)
{
    struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
    struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
    int reg;
    int ret;
    int i;

    /* ipu2 config */
    imx_iomux_v3_setup_multiple_pads(ipu_pads, ARRAY_SIZE(ipu_pads));

    /* backlight */
    imx_iomux_v3_setup_multiple_pads(backlight_pads, ARRAY_SIZE(backlight_pads));

    enable_ipu_clock();
    imx_setup_hdmi();

    for (i = 0; i < ARRAY_SIZE(displays); i++) {
        ret = ipuv3_fb_init(&displays[i].mode, 0,
            displays[i].pixfmt);

        if (ret) {
            puts("Display cannot be configured\n");
            break;
        }
    }

    /* Turn on LDB_DI0 and LDB_DI1 clocks */
    reg = readl(&mxc_ccm->CCGR3);
    reg |= MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
    writel(reg, &mxc_ccm->CCGR3);

    /* Set LDB_DI0 and LDB_DI1 clk select to 3b'011 */
    reg = readl(&mxc_ccm->cs2cdr);
    reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK |
        MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
    reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET) |
        (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
    writel(reg, &mxc_ccm->cs2cdr);

    reg = readl(&mxc_ccm->cscmr2);
    reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
    writel(reg, &mxc_ccm->cscmr2);

    reg = readl(&mxc_ccm->chsccdr);
    reg |= (CHSCCDR_CLK_SEL_LDB_DI0 <<
        MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
    reg |= (CHSCCDR_CLK_SEL_LDB_DI0 <<
        MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
    writel(reg, &mxc_ccm->chsccdr);

    reg = IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH |
        IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_HIGH |
        IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG |
        IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT |
        IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG |
        IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
        IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI1 |
        IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
    writel(reg, &iomux->gpr[2]);

    reg = readl(&iomux->gpr[3]);
    reg &= ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK |
        IOMUXC_GPR3_HDMI_MUX_CTL_MASK);
    reg |= (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<
        IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET) |
        (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<
        IOMUXC_GPR3_HDMI_MUX_CTL_OFFSET);
    writel(reg, &iomux->gpr[3]);
}

#endif /* CONFIG_VIDEO_IPUV3 */

#ifdef CONFIG_IMX_UDC

iomux_v3_cfg_t const otg_udc_pads[] = {
    (MX6_PAD_ENET_RX_ER__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

void udc_pins_setting(void)
{
    imx_iomux_v3_setup_multiple_pads(otg_udc_pads,
        ARRAY_SIZE(otg_udc_pads));

    /* set daisy chain for otg_pin_id on 6q. for 6dl, bit is reserved */
    mxc_iomux_set_gpr_register(1, 13, 1, 0);
}

#endif /*CONFIG_IMX_UDC*/

/* Do not overwrite the console */
/* Use always serial for U-Boot console */
int overwrite_console(void)
{
    return 1;
}

/* the following functions are board specific and override existing functions within the source tree */

int board_early_init_f(void)
{
    setup_uart();

    #ifdef CONFIG_VIDEO_IPUV3
    setup_display();
    #endif

    /* gpio init */
    board_gpio_init();
	
    #ifdef CONFIG_CMD_SATA
    setup_sata();
    #endif

    return 0;
}

int board_init(void)
{
    // address of boot parameters
    gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

    #ifdef CONFIG_MXC_SPI
    setup_spinor();
    #endif

    return 0;
}

int board_late_init(void)
{
    /* watchdog init */
    imx_iomux_v3_setup_pad(MX6_PAD_DISP0_DAT8__WDOG1_B);

    #ifdef CONFIG_ENV_IS_IN_MMC
    board_late_mmc_env_init();
    #endif

    return 0;
}

/* tried to add max7310 init for uart in
    - board_early_init_f
    - board_init
    - board_late_init
   none worked so this function was added */
int c398_board_postclk_init(void)
{
    #ifdef CONFIG_SYS_I2C_MXC
    ws_setup_i2c(I2C1_BASE_ADDR);
    ws_setup_i2c(I2C2_BASE_ADDR);
    ws_setup_i2c(I2C3_BASE_ADDR);
    #endif

    #ifdef CONFIG_PCA953X
    setup_max7310();
    #endif

    return 0;
}
