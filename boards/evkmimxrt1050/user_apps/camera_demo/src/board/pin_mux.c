/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: MIMXRT1052xxxxx
package_id: MIMXRT1052DVL6A
mcu_data: i_mx_1_0
processor_version: 0.0.3
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: L14, peripheral: LPUART1, signal: RX, pin_signal: GPIO_AD_B0_13, slew_rate: Slow, software_input_on: Disable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Down_100K_Ohm, hysteresis_enable: Disable}
  - {pin_num: K14, peripheral: LPUART1, signal: TX, pin_signal: GPIO_AD_B0_12, slew_rate: Slow, software_input_on: Disable, open_drain: Disable, speed: MHZ_100, drive_strength: R0_6,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Down_100K_Ohm, hysteresis_enable: Disable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitDEBUG_UARTPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitDEBUG_UARTPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);          /* iomuxc clock (iomuxc_clk_enable): 0x03u */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 is configured as LPUART1_TX */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 is configured as LPUART1_RX */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitSDRAMPins:
- options: {coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: C2, peripheral: SEMC, signal: 'ADDR, 00', pin_signal: GPIO_EMC_09}
  - {pin_num: G1, peripheral: SEMC, signal: 'ADDR, 01', pin_signal: GPIO_EMC_10}
  - {pin_num: G3, peripheral: SEMC, signal: 'ADDR, 02', pin_signal: GPIO_EMC_11}
  - {pin_num: H1, peripheral: SEMC, signal: 'ADDR, 03', pin_signal: GPIO_EMC_12}
  - {pin_num: A6, peripheral: SEMC, signal: 'ADDR, 04', pin_signal: GPIO_EMC_13}
  - {pin_num: B6, peripheral: SEMC, signal: 'ADDR, 05', pin_signal: GPIO_EMC_14}
  - {pin_num: B1, peripheral: SEMC, signal: 'ADDR, 06', pin_signal: GPIO_EMC_15}
  - {pin_num: A5, peripheral: SEMC, signal: 'ADDR, 07', pin_signal: GPIO_EMC_16}
  - {pin_num: A4, peripheral: SEMC, signal: 'ADDR, 08', pin_signal: GPIO_EMC_17}
  - {pin_num: B2, peripheral: SEMC, signal: 'ADDR, 09', pin_signal: GPIO_EMC_18}
  - {pin_num: G2, peripheral: SEMC, signal: 'ADDR, 10', pin_signal: GPIO_EMC_23}
  - {pin_num: B4, peripheral: SEMC, signal: 'ADDR, 11', pin_signal: GPIO_EMC_19}
  - {pin_num: A3, peripheral: SEMC, signal: 'ADDR, 12', pin_signal: GPIO_EMC_20}
  - {pin_num: C1, peripheral: SEMC, signal: 'BA, 0', pin_signal: GPIO_EMC_21}
  - {pin_num: F1, peripheral: SEMC, signal: 'BA, 1', pin_signal: GPIO_EMC_22}
  - {pin_num: D3, peripheral: SEMC, signal: semc_cas, pin_signal: GPIO_EMC_24}
  - {pin_num: A2, peripheral: SEMC, signal: semc_cke, pin_signal: GPIO_EMC_27}
  - {pin_num: B3, peripheral: SEMC, signal: semc_clk, pin_signal: GPIO_EMC_26}
  - {pin_num: C7, peripheral: SEMC, signal: 'CSX, 0', pin_signal: GPIO_EMC_41}
  - {pin_num: E3, peripheral: SEMC, signal: 'DATA, 00', pin_signal: GPIO_EMC_00}
  - {pin_num: F3, peripheral: SEMC, signal: 'DATA, 01', pin_signal: GPIO_EMC_01}
  - {pin_num: F4, peripheral: SEMC, signal: 'DATA, 02', pin_signal: GPIO_EMC_02}
  - {pin_num: G4, peripheral: SEMC, signal: 'DATA, 03', pin_signal: GPIO_EMC_03}
  - {pin_num: F2, peripheral: SEMC, signal: 'DATA, 04', pin_signal: GPIO_EMC_04}
  - {pin_num: G5, peripheral: SEMC, signal: 'DATA, 05', pin_signal: GPIO_EMC_05}
  - {pin_num: H5, peripheral: SEMC, signal: 'DATA, 06', pin_signal: GPIO_EMC_06}
  - {pin_num: H4, peripheral: SEMC, signal: 'DATA, 07', pin_signal: GPIO_EMC_07}
  - {pin_num: C6, peripheral: SEMC, signal: 'DATA, 08', pin_signal: GPIO_EMC_30}
  - {pin_num: C5, peripheral: SEMC, signal: 'DATA, 09', pin_signal: GPIO_EMC_31}
  - {pin_num: D5, peripheral: SEMC, signal: 'DATA, 10', pin_signal: GPIO_EMC_32}
  - {pin_num: C4, peripheral: SEMC, signal: 'DATA, 11', pin_signal: GPIO_EMC_33}
  - {pin_num: D4, peripheral: SEMC, signal: 'DATA, 12', pin_signal: GPIO_EMC_34}
  - {pin_num: E5, peripheral: SEMC, signal: 'DATA, 13', pin_signal: GPIO_EMC_35}
  - {pin_num: C3, peripheral: SEMC, signal: 'DATA, 14', pin_signal: GPIO_EMC_36}
  - {pin_num: E4, peripheral: SEMC, signal: 'DATA, 15', pin_signal: GPIO_EMC_37}
  - {pin_num: H3, peripheral: SEMC, signal: 'DM, 0', pin_signal: GPIO_EMC_08}
  - {pin_num: D6, peripheral: SEMC, signal: 'DM, 1', pin_signal: GPIO_EMC_38}
  - {pin_num: D2, peripheral: SEMC, signal: semc_ras, pin_signal: GPIO_EMC_25}
  - {pin_num: D1, peripheral: SEMC, signal: semc_we, pin_signal: GPIO_EMC_28}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitSDRAMPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitSDRAMPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);          /* iomuxc clock (iomuxc_clk_enable): 0x03u */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_00_SEMC_DATA00,         /* GPIO_EMC_00 is configured as SEMC_DATA00 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_01_SEMC_DATA01,         /* GPIO_EMC_01 is configured as SEMC_DATA01 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_02_SEMC_DATA02,         /* GPIO_EMC_02 is configured as SEMC_DATA02 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_03_SEMC_DATA03,         /* GPIO_EMC_03 is configured as SEMC_DATA03 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_04_SEMC_DATA04,         /* GPIO_EMC_04 is configured as SEMC_DATA04 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_05_SEMC_DATA05,         /* GPIO_EMC_05 is configured as SEMC_DATA05 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_06_SEMC_DATA06,         /* GPIO_EMC_06 is configured as SEMC_DATA06 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_07_SEMC_DATA07,         /* GPIO_EMC_07 is configured as SEMC_DATA07 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_08_SEMC_DM00,           /* GPIO_EMC_08 is configured as SEMC_DM00 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_09_SEMC_ADDR00,         /* GPIO_EMC_09 is configured as SEMC_ADDR00 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_10_SEMC_ADDR01,         /* GPIO_EMC_10 is configured as SEMC_ADDR01 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_11_SEMC_ADDR02,         /* GPIO_EMC_11 is configured as SEMC_ADDR02 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_12_SEMC_ADDR03,         /* GPIO_EMC_12 is configured as SEMC_ADDR03 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_13_SEMC_ADDR04,         /* GPIO_EMC_13 is configured as SEMC_ADDR04 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_14_SEMC_ADDR05,         /* GPIO_EMC_14 is configured as SEMC_ADDR05 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_15_SEMC_ADDR06,         /* GPIO_EMC_15 is configured as SEMC_ADDR06 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_16_SEMC_ADDR07,         /* GPIO_EMC_16 is configured as SEMC_ADDR07 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_17_SEMC_ADDR08,         /* GPIO_EMC_17 is configured as SEMC_ADDR08 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_18_SEMC_ADDR09,         /* GPIO_EMC_18 is configured as SEMC_ADDR09 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_19_SEMC_ADDR11,         /* GPIO_EMC_19 is configured as SEMC_ADDR11 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_20_SEMC_ADDR12,         /* GPIO_EMC_20 is configured as SEMC_ADDR12 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_21_SEMC_BA0,            /* GPIO_EMC_21 is configured as SEMC_BA0 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_22_SEMC_BA1,            /* GPIO_EMC_22 is configured as SEMC_BA1 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_23_SEMC_ADDR10,         /* GPIO_EMC_23 is configured as SEMC_ADDR10 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_24_SEMC_CAS,            /* GPIO_EMC_24 is configured as SEMC_CAS */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_25_SEMC_RAS,            /* GPIO_EMC_25 is configured as SEMC_RAS */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_26_SEMC_CLK,            /* GPIO_EMC_26 is configured as SEMC_CLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_27_SEMC_CKE,            /* GPIO_EMC_27 is configured as SEMC_CKE */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_28_SEMC_WE,             /* GPIO_EMC_28 is configured as SEMC_WE */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_30_SEMC_DATA08,         /* GPIO_EMC_30 is configured as SEMC_DATA08 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_31_SEMC_DATA09,         /* GPIO_EMC_31 is configured as SEMC_DATA09 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_32_SEMC_DATA10,         /* GPIO_EMC_32 is configured as SEMC_DATA10 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_33_SEMC_DATA11,         /* GPIO_EMC_33 is configured as SEMC_DATA11 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_34_SEMC_DATA12,         /* GPIO_EMC_34 is configured as SEMC_DATA12 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_35_SEMC_DATA13,         /* GPIO_EMC_35 is configured as SEMC_DATA13 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_36_SEMC_DATA14,         /* GPIO_EMC_36 is configured as SEMC_DATA14 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_37_SEMC_DATA15,         /* GPIO_EMC_37 is configured as SEMC_DATA15 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_38_SEMC_DM01,           /* GPIO_EMC_38 is configured as SEMC_DM01 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_41_SEMC_CSX00,          /* GPIO_EMC_41 is configured as SEMC_CSX00 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitCSIPins:
- options: {coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: H13, peripheral: CSI, signal: 'csi_data, 09', pin_signal: GPIO_AD_B1_08}
  - {pin_num: M13, peripheral: CSI, signal: 'csi_data, 08', pin_signal: GPIO_AD_B1_09}
  - {pin_num: L13, peripheral: CSI, signal: 'csi_data, 07', pin_signal: GPIO_AD_B1_10}
  - {pin_num: J13, peripheral: CSI, signal: 'csi_data, 06', pin_signal: GPIO_AD_B1_11}
  - {pin_num: H12, peripheral: CSI, signal: 'csi_data, 05', pin_signal: GPIO_AD_B1_12}
  - {pin_num: H11, peripheral: CSI, signal: 'csi_data, 04', pin_signal: GPIO_AD_B1_13}
  - {pin_num: G12, peripheral: CSI, signal: 'csi_data, 03', pin_signal: GPIO_AD_B1_14}
  - {pin_num: J14, peripheral: CSI, signal: 'csi_data, 02', pin_signal: GPIO_AD_B1_15}
  - {pin_num: L12, peripheral: CSI, signal: csi_pixclk, pin_signal: GPIO_AD_B1_04}
  - {pin_num: K12, peripheral: CSI, signal: csi_mclk, pin_signal: GPIO_AD_B1_05}
  - {pin_num: J12, peripheral: CSI, signal: csi_vsync, pin_signal: GPIO_AD_B1_06}
  - {pin_num: K10, peripheral: CSI, signal: csi_hsync, pin_signal: GPIO_AD_B1_07}
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Up_22K_Ohm, hysteresis_enable: Disable}
  - {pin_num: F11, peripheral: GPIO1, signal: 'gpio_io, 04', pin_signal: GPIO_AD_B0_04}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitCSIPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitCSIPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);          /* iomuxc clock (iomuxc_clk_enable): 0x03u */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B0_04_GPIO1_IO04,        /* GPIO_AD_B0_04 is configured as GPIO1_IO04 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_04_CSI_PIXCLK,        /* GPIO_AD_B1_04 is configured as CSI_PIXCLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_05_CSI_MCLK,          /* GPIO_AD_B1_05 is configured as CSI_MCLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_06_CSI_VSYNC,         /* GPIO_AD_B1_06 is configured as CSI_VSYNC */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_07_CSI_HSYNC,         /* GPIO_AD_B1_07 is configured as CSI_HSYNC */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_08_CSI_DATA09,        /* GPIO_AD_B1_08 is configured as CSI_DATA09 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_09_CSI_DATA08,        /* GPIO_AD_B1_09 is configured as CSI_DATA08 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_10_CSI_DATA07,        /* GPIO_AD_B1_10 is configured as CSI_DATA07 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_11_CSI_DATA06,        /* GPIO_AD_B1_11 is configured as CSI_DATA06 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_12_CSI_DATA05,        /* GPIO_AD_B1_12 is configured as CSI_DATA05 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_13_CSI_DATA04,        /* GPIO_AD_B1_13 is configured as CSI_DATA04 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_14_CSI_DATA03,        /* GPIO_AD_B1_14 is configured as CSI_DATA03 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_15_CSI_DATA02,        /* GPIO_AD_B1_15 is configured as CSI_DATA02 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitCSIPins:
- options: {coreID: core0, enableClock: 'true'}
- pin_list:

  - {pin_num: J11, peripheral: LPI2C1, signal: SCL, pin_signal: GPIO_AD_B1_00, slew_rate: Slow, software_input_on: Enable, open_drain: Enable, speed: MHZ_100, drive_strength: R0_6,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Up_22K_Ohm, hysteresis_enable: Disable}
  - {pin_num: K11, peripheral: LPI2C1, signal: SDA, pin_signal: GPIO_AD_B1_01, slew_rate: Slow, software_input_on: Enable, open_drain: Enable, speed: MHZ_100, drive_strength: R0_6,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Up_22K_Ohm, hysteresis_enable: Disable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitLPI2C1Pins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitLPI2C1Pins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);          /* iomuxc clock (iomuxc_clk_enable): 0x03u */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL,        /* GPIO_AD_B1_00 is configured as LPI2C1_SCL */
      1U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA,        /* GPIO_AD_B1_01 is configured as LPI2C1_SDA */
      1U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL,        /* GPIO_AD_B1_00 PAD functional properties : */
      0xD8B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Enabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 22K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA,        /* GPIO_AD_B1_01 PAD functional properties : */
      0xD8B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Enabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 22K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitLCDPins:
- options: {coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: D7, peripheral: LCDIF, signal: lcdif_clk, pin_signal: GPIO_B0_00}
  - {pin_num: E7, peripheral: LCDIF, signal: lcdif_enable, pin_signal: GPIO_B0_01}
  - {pin_num: E8, peripheral: LCDIF, signal: lcdif_hsync, pin_signal: GPIO_B0_02}
  - {pin_num: D8, peripheral: LCDIF, signal: lcdif_vsync, pin_signal: GPIO_B0_03}
  - {pin_num: C8, peripheral: LCDIF, signal: 'lcdif_data, 00', pin_signal: GPIO_B0_04}
  - {pin_num: B8, peripheral: LCDIF, signal: 'lcdif_data, 01', pin_signal: GPIO_B0_05}
  - {pin_num: A8, peripheral: LCDIF, signal: 'lcdif_data, 02', pin_signal: GPIO_B0_06}
  - {pin_num: A9, peripheral: LCDIF, signal: 'lcdif_data, 03', pin_signal: GPIO_B0_07}
  - {pin_num: B9, peripheral: LCDIF, signal: 'lcdif_data, 04', pin_signal: GPIO_B0_08}
  - {pin_num: C9, peripheral: LCDIF, signal: 'lcdif_data, 05', pin_signal: GPIO_B0_09}
  - {pin_num: D9, peripheral: LCDIF, signal: 'lcdif_data, 06', pin_signal: GPIO_B0_10}
  - {pin_num: A10, peripheral: LCDIF, signal: 'lcdif_data, 07', pin_signal: GPIO_B0_11}
  - {pin_num: C10, peripheral: LCDIF, signal: 'lcdif_data, 08', pin_signal: GPIO_B0_12}
  - {pin_num: D10, peripheral: LCDIF, signal: 'lcdif_data, 09', pin_signal: GPIO_B0_13}
  - {pin_num: E10, peripheral: LCDIF, signal: 'lcdif_data, 10', pin_signal: GPIO_B0_14}
  - {pin_num: E11, peripheral: LCDIF, signal: 'lcdif_data, 11', pin_signal: GPIO_B0_15}
  - {pin_num: A11, peripheral: LCDIF, signal: 'lcdif_data, 12', pin_signal: GPIO_B1_00}
  - {pin_num: B11, peripheral: LCDIF, signal: 'lcdif_data, 13', pin_signal: GPIO_B1_01}
  - {pin_num: C11, peripheral: LCDIF, signal: 'lcdif_data, 14', pin_signal: GPIO_B1_02}
  - {pin_num: D11, peripheral: LCDIF, signal: 'lcdif_data, 15', pin_signal: GPIO_B1_03}
  - {pin_num: M11, peripheral: GPIO1, signal: 'gpio_io, 02', pin_signal: GPIO_AD_B0_02}
  - {pin_num: B14, peripheral: GPIO2, signal: 'gpio_io, 31', pin_signal: GPIO_B1_15}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitLCDPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitLCDPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);          /* iomuxc clock (iomuxc_clk_enable): 0x03u */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B0_02_GPIO1_IO02,        /* GPIO_AD_B0_02 is configured as GPIO1_IO02 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_00_LCD_CLK,              /* GPIO_B0_00 is configured as LCD_CLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_01_LCD_ENABLE,           /* GPIO_B0_01 is configured as LCD_ENABLE */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_02_LCD_HSYNC,            /* GPIO_B0_02 is configured as LCD_HSYNC */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_03_LCD_VSYNC,            /* GPIO_B0_03 is configured as LCD_VSYNC */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_04_LCD_DATA00,           /* GPIO_B0_04 is configured as LCD_DATA00 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_05_LCD_DATA01,           /* GPIO_B0_05 is configured as LCD_DATA01 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_06_LCD_DATA02,           /* GPIO_B0_06 is configured as LCD_DATA02 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_07_LCD_DATA03,           /* GPIO_B0_07 is configured as LCD_DATA03 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_08_LCD_DATA04,           /* GPIO_B0_08 is configured as LCD_DATA04 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_09_LCD_DATA05,           /* GPIO_B0_09 is configured as LCD_DATA05 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_10_LCD_DATA06,           /* GPIO_B0_10 is configured as LCD_DATA06 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_11_LCD_DATA07,           /* GPIO_B0_11 is configured as LCD_DATA07 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_12_LCD_DATA08,           /* GPIO_B0_12 is configured as LCD_DATA08 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_13_LCD_DATA09,           /* GPIO_B0_13 is configured as LCD_DATA09 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_14_LCD_DATA10,           /* GPIO_B0_14 is configured as LCD_DATA10 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_15_LCD_DATA11,           /* GPIO_B0_15 is configured as LCD_DATA11 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_00_LCD_DATA12,           /* GPIO_B1_00 is configured as LCD_DATA12 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_01_LCD_DATA13,           /* GPIO_B1_01 is configured as LCD_DATA13 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_02_LCD_DATA14,           /* GPIO_B1_02 is configured as LCD_DATA14 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_03_LCD_DATA15,           /* GPIO_B1_03 is configured as LCD_DATA15 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_15_GPIO2_IO31,           /* GPIO_B1_15 is configured as GPIO2_IO31 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
}

void BOARD_InitSDPins(void) {
    CLOCK_EnableClock(kCLOCK_Iomuxc);          /* iomuxc clock (iomuxc_clk_enable): 0x03u */

    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD, 0);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK, 0);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0, 0);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1, 0);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2, 0);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3, 0);

    /* voltage select PIN */
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_14_USDHC1_VSELECT, 0);

     /* card detect PIN */
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_12_GPIO2_IO28, 0);
    /* power reset pin */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05, 0);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD, IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
                                                       IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                                                       IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                                                       IOMUXC_SW_PAD_CTL_PAD_DSE(1));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK, IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                                                       IOMUXC_SW_PAD_CTL_PAD_SPEED(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                                                       IOMUXC_SW_PAD_CTL_PAD_DSE(1));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0, IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1));

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1, IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1));

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2, IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1));

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3, IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1));

    /*voltage select pin*/
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_14_USDHC1_VSELECT, IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(4));

    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_12_GPIO2_IO28, IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                                                           IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
                                                           IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1));
    
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
