/*
 * Copyright 2017 NXP
 * All rights reserved.
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
 * o Neither the name of the copyright holder nor the names of its
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

#include "fsl_video_common.h"
#include "fsl_camera.h"
#include "fsl_camera_device.h"
#include "fsl_mt9m114.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct _mt9m114_reg
{
    uint16_t reg;
    uint8_t  size;
    uint32_t value;
} mt9m114_reg_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
status_t MT9M114_Init(camera_device_handle_t *handle, const camera_config_t *config);
status_t MT9M114_Deinit(camera_device_handle_t *handle);
status_t MT9M114_Start(camera_device_handle_t *handle);
status_t MT9M114_Stop(camera_device_handle_t *handle);
status_t MT9M114_Control(camera_device_handle_t *handle, camera_device_cmd_t cmd, int32_t arg);
status_t MT9M114_InitExt(camera_device_handle_t *handle, const camera_config_t *config, const void *specialConfig);

/*******************************************************************************
 * Variables
 ******************************************************************************/
const camera_device_operations_t mt9m114_ops = {
    .init = MT9M114_Init,
    .deinit = MT9M114_Deinit,
    .start = MT9M114_Start,
    .stop = MT9M114_Stop,
    .control = MT9M114_Control,
    .init_ext = MT9M114_InitExt,
};

static const mt9m114_reg_t ConfigFram720P[] = {
    {MT9M114_REG_LOGICAL_ADDRESS_ACCESS,                2u, 0x1000},
    {MT9M114_VAR_CAM_SYSCTL_PLL_ENABLE,                 1u, 0x01},      //cam_sysctl_pll_enable = 1
    {MT9M114_VAR_CAM_SYSCTL_PLL_DIVIDER_M_N,            2u, 0x0225},    //cam_sysctl_pll_divider_m_n = 549
    {MT9M114_VAR_CAM_SYSCTL_PLL_DIVIDER_P,              2u, 0x0700},    //cam_sysctl_pll_divider_p = 1792
    {MT9M114_VAR_CAM_PORT_OUTPUT_CONTROL,               2u, 0x8008},    //cam_port_output_control = 32776
    {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_START,           2u, 0x007C},    //cam_sensor_cfg_y_addr_start = 124
    {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_START,           2u, 0x0004},    //cam_sensor_cfg_x_addr_start = 4
    {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_END,             2u, 0x0353},    //cam_sensor_cfg_y_addr_end = 851
    {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_END,             2u, 0x050B},    //cam_sensor_cfg_x_addr_end = 1291
    {MT9M114_VAR_CAM_SENSOR_CFG_PIXCLK,                 4u, 0x2349340}, //cam_sensor_cfg_pixclk = 37000000
    {MT9M114_VAR_CAM_SENSOR_CFG_ROW_SPEED,              2u, 0x0001},    //cam_sensor_cfg_row_speed = 1
    {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    2u, 0x00DB},    //cam_sensor_cfg_fine_integ_time_min = 219
    {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    2u, 0x05C8},    //cam_sensor_cfg_fine_integ_time_max = 1480
    {MT9M114_VAR_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     2u, 0x02FE},    //cam_sensor_cfg_frame_length_lines = 766
    {MT9M114_VAR_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        2u, 0x064B},    //cam_sensor_cfg_line_length_pck = 1611
    {MT9M114_VAR_CAM_SENSOR_CFG_FINE_CORRECTION,        2u, 0x0060},    //cam_sensor_cfg_fine_correction = 96
    {MT9M114_VAR_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         2u, 0x02D3},    //cam_sensor_cfg_cpipe_last_row = 723
    {MT9M114_VAR_CAM_SENSOR_CFG_REG_0_DATA,             2u, 0x0020},    //cam_sensor_cfg_reg_0_data = 32
    {MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE,          2u, 0x0000},    //cam_sensor_control_read_mode = 0
    {MT9M114_VAR_CAM_CROP_WINDOW_XOFFSET,               2u, 0x0000},    //cam_crop_window_xoffset = 0
    {MT9M114_VAR_CAM_CROP_WINDOW_YOFFSET,               2u, 0x0000},    //cam_crop_window_yoffset = 0
    {MT9M114_VAR_CAM_CROP_WINDOW_WIDTH,                 2u, 0x0500},    //cam_crop_window_width = 1280
    {MT9M114_VAR_CAM_CROP_WINDOW_HEIGHT,                2u, 0x02D0},    //cam_crop_window_height = 720
    {MT9M114_VAR_CAM_CROP_CROPMODE,                     1u, 0x03},      //cam_crop_cropmode = 3
    {MT9M114_VAR_CAM_OUTPUT_WIDTH,                      2u, 0x0500},    //cam_output_width = 1280
    {MT9M114_VAR_CAM_OUTPUT_HEIGHT,                     2u, 0x02D0},    //cam_output_height = 720
    {MT9M114_VAR_CAM_AET_AEMODE,                        1u, 0x00},      //cam_aet_aemode = 0
    {MT9M114_VAR_CAM_AET_MAX_FRAME_RATE,                2u, 0x1DFC},    //cam_aet_max_frame_rate = 7676
    {MT9M114_VAR_CAM_AET_MIN_FRAME_RATE,                2u, 0x1DFC},    //cam_aet_min_frame_rate = 7676
    {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       2u, 0x0000},    //cam_stat_awb_clip_window_xstart = 0
    {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       2u, 0x0000},    //cam_stat_awb_clip_window_ystart = 0
    {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XEND,         2u, 0x04FF},    //cam_stat_awb_clip_window_xend = 1279
    {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YEND,         2u, 0x02CF},    //cam_stat_awb_clip_window_yend = 719
    {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     2u, 0x0000},    //cam_stat_ae_initial_window_xstart = 0
    {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     2u, 0x0000},    //cam_stat_ae_initial_window_ystart = 0
    {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XEND,       2u, 0x00FF},    //cam_stat_ae_initial_window_xend = 255
    {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YEND,       2u, 0x008F},    //cam_stat_ae_initial_window_yend = 143

    {MT9M114_VAR_CAM_OUTPUT_FORMAT,                     2u, 0x0112},    //RGB565 mode
    {MT9M114_REG_PAD_SLEW,                              2u, 0x0426},    //Pad slew rate
};


/*******************************************************************************
 * Code
 ******************************************************************************/

static status_t MT9M114_Read(TwoWireIf_i2c_t i2c, uint32_t reg, uint8_t size, uint32_t * value)
{
    status_t status;
    uint8_t data[6];
    uint8_t index = 0;

    data[index++] = (uint8_t)((reg >> 8U) & 0xFF);
    data[index++] = (uint8_t)(reg & 0xFF);

    while (1)
    {
        status = LPI2C_MasterStart(i2c, MT9M114_TWO_WIRE_I2C_ADDR, kLPI2C_Write);

        if (kStatus_Success != status)
        {
            LPI2C_MasterStop(i2c);
        }
        else
        {
            break;
        }
    }

    LPI2C_MasterSend(i2c, data, index);

    LPI2C_MasterStart(i2c, MT9M114_TWO_WIRE_I2C_ADDR, kLPI2C_Read);

    LPI2C_MasterReceive(i2c, data, size);

    status = LPI2C_MasterStop(i2c);

    if( size == 2)
    {
        *value = data[0];
        *value <<= 8U;
        *value += data[1];
    }
    else if( size == 4)
    {
        *value = data[0];
        *value <<= 8U;
        *value += data[1];
        *value <<= 8U;
        *value += data[2];
        *value <<= 8U;
        *value += data[3];
    }
    else if( size == 1)
    {
        *value = data[0];
    }

    return status;
}

static status_t MT9M114_Write(TwoWireIf_i2c_t i2c, uint16_t reg, uint8_t size, uint32_t value)
{
    status_t status;
    uint8_t data[6];
    uint8_t index = 0;

    data[index++] = (uint8_t)((reg >> 8U) & 0xFF);
    data[index++] = (uint8_t)(reg & 0xFF);

    if( size == 2)
    {
        data[index++] = (uint8_t)((value >> 8U) & 0xFF);
        data[index++] = (uint8_t)(value & 0xFF);
    }
    else if( size == 4)
    {
        data[index++] = (uint8_t)((value >> 24U) & 0xFF);
        data[index++] = (uint8_t)((value >> 26U) & 0xFF);
        data[index++] = (uint8_t)((value >> 8U) & 0xFF);
        data[index++] = (uint8_t)(value & 0xFF);
    }
    else if( size == 1)
    {
        data[index++] = value;
    }

    while (1)
    {
        status = LPI2C_MasterStart(i2c, MT9M114_TWO_WIRE_I2C_ADDR, kLPI2C_Write);

        if (kStatus_Success != status)
        {
            LPI2C_MasterStop(i2c);
        }
        else
        {
            break;
        }
    }

    LPI2C_MasterSend(i2c, data, index);

    return LPI2C_MasterStop(i2c);
}

static status_t MT9M114_MultiWrite(TwoWireIf_i2c_t i2c, const mt9m114_reg_t regs[], uint32_t num)
{
    status_t status = kStatus_Success;

    for (uint32_t i = 0; i < num; i++)
    {
        status = MT9M114_Write(i2c, regs[i].reg, regs[i].size, regs[i].value);

        if (kStatus_Success != status)
        {
            break;
        }
    }

    return status;
}

static void MT9M114_DelayMs(uint32_t ms)
{
    volatile uint64_t i;

    i = (uint64_t)SystemCoreClock * ms / 3000;
    while (i--)
    {
    }
}

static status_t MT9M114_SoftwareReset(TwoWireIf_i2c_t i2c)
{
    uint32_t value;

    MT9M114_Read(i2c, MT9M114_REG_RESET_AND_MISC_CONTROL, 2u, &value);
    MT9M114_Write(i2c, MT9M114_REG_RESET_AND_MISC_CONTROL, 2u, value | 0x01);
    MT9M114_DelayMs(10);
    MT9M114_Write(i2c, MT9M114_REG_RESET_AND_MISC_CONTROL, 2u, value & (~1));
    MT9M114_DelayMs(10);

    return kStatus_Success;
}

static status_t MT9M114_ChangeConfig(TwoWireIf_i2c_t i2c)
{
    uint32_t value;

    /* Set the desired next state (SYS_STATE_ENTER_CONFIG_CHANGE = 0x28). */
    MT9M114_Write(i2c, MT9M114_VAR_SYSMGR_NEXT_STATE, 1u, MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);

    /* Check that the FW is ready to accept a new command. */
    while(1)
    {
        MT9M114_Read(i2c, MT9M114_REG_COMMAND_REGISTER, 2u, &value);
        if(!(value & MT9M114_COMMAND_SET_STATE))
        {
            break;
        }
    }

    /* Issue the Set State command. */
    MT9M114_Write(i2c, MT9M114_REG_COMMAND_REGISTER, 2u, MT9M114_COMMAND_SET_STATE | MT9M114_COMMAND_OK);

    /* Wait for the FW to complete the command. */
    while(1)
    {
        MT9M114_DelayMs(10);
        MT9M114_Read(i2c, MT9M114_REG_COMMAND_REGISTER, 2u, &value);
        if(!(value & MT9M114_COMMAND_SET_STATE))
        {
            break;
        }
    }

    /* Check the 'OK' bit to see if the command was successful. */
    MT9M114_Read(i2c, MT9M114_REG_COMMAND_REGISTER, 2u, &value);
    if(!(value & MT9M114_COMMAND_OK))
    {
        return kStatus_Fail;
    }

    /* Check if the current state is streaming. */
    MT9M114_Read(i2c, MT9M114_VAR_SYSMGR_CURRENT_STATE, 1u, &value);
    if(value != MT9M114_SYS_STATE_STREAMING)
    {
        return kStatus_Fail;
    }

    /* Check if the set-state command is successful. */
    MT9M114_Read(i2c, MT9M114_VAR_SYSMGR_CMD_STATUS, 1u, &value);
    if(value != MT9M114_SYS_STATE_SET_RESULT_ENOERR)
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t MT9M114_Init(camera_device_handle_t *handle, const camera_config_t *config)
{
    status_t status;
    uint16_t chip_id;

    mt9m114_resource_t *resource = (mt9m114_resource_t *)(handle->resource);
    TwoWireIf_i2c_t i2c = resource->twoWireIfI2C;

    if ((kCAMERA_InterfaceNonGatedClock != config->interface) &&
        (kCAMERA_InterfaceGatedClock != config->interface) &&
        (kCAMERA_InterfaceCCIR656 != config->interface))
    {
        return kStatus_InvalidArgument;
    }

    if ((FSL_VIDEO_EXTRACT_WIDTH(config->resolution) > 1280) ||
        (FSL_VIDEO_EXTRACT_HEIGHT(config->resolution) > 720))
    {
        return kStatus_InvalidArgument;
    }

    resource->pullPowerDownPin(true);       /* Power down. */
    MT9M114_DelayMs(10);                    /* Delay 10ms. */
    resource->pullPowerDownPin(false);      /* Power up. */
    MT9M114_DelayMs(10);                    /* Delay 10ms. */

    /* Identify the device. */
    status = MT9M114_Read(i2c, MT9M114_REG_CHIP_ID, 2u, (uint32_t *)&chip_id);
    if (kStatus_Success != status)
    {
        return status;
    }
    if (MT9M114_CHIP_ID != chip_id)
    {
        return kStatus_Fail;
    }

    /* SW reset. */
    MT9M114_SoftwareReset(i2c);

    /* Configure video format. */
    status += MT9M114_MultiWrite(i2c, ConfigFram720P, ARRAY_SIZE(ConfigFram720P));

    if (status)
    {
        return kStatus_Fail;
    }

    /* Execute Change-Config command. */
    MT9M114_ChangeConfig(i2c);

    return kStatus_Success;
}

static status_t MT9M114_Deinit(camera_device_handle_t *handle)
{
    ((mt9m114_resource_t *)(handle->resource))->pullPowerDownPin(true);

    return kStatus_Success;
}

static status_t MT9M114_Start(camera_device_handle_t *handle)
{
    return kStatus_Success;
}

static status_t MT9M114_Stop(camera_device_handle_t *handle)
{
    return kStatus_Success;
}

static status_t MT9M114_Control(camera_device_handle_t *handle, camera_device_cmd_t cmd, int32_t arg)
{
    return kStatus_Success;
}

static status_t MT9M114_InitExt(camera_device_handle_t *handle, const camera_config_t *config, const void *specialConfig)
{
    return MT9M114_Init(handle, config);
}

