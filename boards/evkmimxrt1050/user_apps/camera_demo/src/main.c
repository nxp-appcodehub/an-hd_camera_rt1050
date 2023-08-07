/*
 * Copyright (c) 2017, NXP
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

#include "board.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"

#include "fsl_gpio.h"
#include "fsl_csi.h"
#include "fsl_pxp.h"
#include "fsl_elcdif.h"
#include "fsl_sd_disk.h"

#include "fsl_camera.h"
#include "fsl_camera_device.h"
#include "fsl_camera_receiver.h"
#include "fsl_csi_camera_adapter.h"
#include "fsl_mt9m114.h"
#include "fsl_ft5406.h"

#include "ff.h"
#include "diskio.h"

#include "jpeglib.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_ELCDIF          LCDIF
#define APP_PXP             PXP
#define APP_CAMERA_I2C      LPI2C1

/* Camera definition. */
#define CAMERA_WIDTH        1280
#define CAMERA_HEIGHT       720
#define CAMERA_CONTROL_FLAGS (kCAMERA_HrefActiveHigh | kCAMERA_DataLatchOnRisingEdge)

#define FRAME_BUFFER_ALIGN          64      /* Frame buffer data alignment. */
#define CAMERA_FRAME_BUFFER_COUNT   4
#define LCD_FRAME_BUFFER_COUNT      2
#define BYTE_PER_PIXEL              2       /* Pixel format RGB565. */

/* LCD definition. */
#define LCD_WIDTH   480
#define LCD_HEIGHT  272
#define LCD_HSW 41
#define LCD_HFP 4
#define LCD_HBP 8
#define LCD_VSW 10
#define LCD_VFP 4
#define LCD_VBP 2

#define LCD_POL_FLAGS (kELCDIF_DataEnableActiveHigh | \
                       kELCDIF_VsyncActiveLow | \
                       kELCDIF_HsyncActiveLow | \
                       kELCDIF_DriveDataOnRisingClkEdge)

#define LCDIF_DATA_BUS kELCDIF_DataBus16Bit

/* Display. */
#define LCD_DISP_GPIO GPIO1
#define LCD_DISP_GPIO_PIN   2

/* Back light. */
#define LCD_BL_GPIO GPIO2
#define LCD_BL_GPIO_PIN     31

/* Image definition. */
#define IMAGE_WIDTH         CAMERA_WIDTH
#define IMAGE_HEIGHT        CAMERA_HEIGHT
#define COMPRESS_LINES      1u
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void BOARD_PullCameraResetPin(bool pullUp);
static void BOARD_PullCameraPowerDownPin(bool pullUp);

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_ALIGN(static uint16_t csiFrameBuf[CAMERA_FRAME_BUFFER_COUNT][CAMERA_HEIGHT][CAMERA_WIDTH], FRAME_BUFFER_ALIGN);
AT_NONCACHEABLE_SECTION_ALIGN(static uint16_t lcdFrameBuf[LCD_FRAME_BUFFER_COUNT][LCD_HEIGHT][LCD_WIDTH], FRAME_BUFFER_ALIGN);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t JpegScanlines[COMPRESS_LINES][IMAGE_WIDTH * 3u], FRAME_BUFFER_ALIGN);

static mt9m114_resource_t mt9m114Resource = {
    .twoWireIfI2C = APP_CAMERA_I2C,
    .pullResetPin = BOARD_PullCameraResetPin,
    .pullPowerDownPin = BOARD_PullCameraPowerDownPin,
    .inputClockFreq_Hz = 24000000,
};

static camera_device_handle_t cameraDevice = {
    .resource = &mt9m114Resource,
    .ops = &mt9m114_ops,
};

static csi_resource_t csiResource = {
    .csiBase = CSI,
};

static csi_private_data_t csiPrivateData;

static camera_receiver_handle_t cameraReceiver = {
    .resource = &csiResource,
    .ops = &csi_ops,
    .privateData = &csiPrivateData,
};

extern sd_card_t g_sd; /* sd card descriptor */

/*! @brief SDMMC host detect card configuration */
static const sdmmchost_detect_card_t s_sdCardDetect = {
#ifndef BOARD_SD_DETECT_TYPE
    .cdType = kSDMMCHOST_DetectCardByGpioCD,
#else
    .cdType = BOARD_SD_DETECT_TYPE,
#endif
    .cdTimeOut_ms = (~0U),
};

static const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};
static FATFS g_fileSystem;              /* File system object */

/* Touch driver handle. */
static ft5406_handle_t touchHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/

extern void CSI_DriverIRQHandler(void);

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
}

static void BOARD_PullCameraResetPin(bool pullUp)
{
    /* Reset pin is connected to DCDC_3V3. */
    return;
}

static void BOARD_PullCameraPowerDownPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(GPIO1, 4, 1);
    }
    else
    {
        GPIO_PinWrite(GPIO1, 4, 0);
    }
}

/*!
 * @brief Initialize the LPI2C1.
 */
static void InitLPI2C1(void)
{
    /* Configure LPI2C. */
    lpi2c_master_config_t i2cMasterConfig;

    LPI2C_MasterGetDefaultConfig(&i2cMasterConfig);
    i2cMasterConfig.baudRate_Hz = 100000;
    i2cMasterConfig.debugEnable = true;
    i2cMasterConfig.ignoreAck = true;

    CLOCK_SetMux(kCLOCK_Lpi2cMux, 1);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 0);

    LPI2C_MasterInit(APP_CAMERA_I2C, &i2cMasterConfig, CLOCK_GetOscFreq());
}

/*!
 * @brief Initialize the camera interface.
 */
static void InitCamera(void)
{
    /* Configure CSI MCLK. */
    CLOCK_SetMux(kCLOCK_CsiMux, 0);
    CLOCK_SetDiv(kCLOCK_CsiDiv, 0);

    /* Set the pins for CSI reset and power down. */
    const gpio_pin_config_t pinConfig = {
        kGPIO_DigitalOutput, 1,
    };
    GPIO_PinInit(GPIO1, 4, &pinConfig);

    /* Configure camera device and receiver.*/
    const camera_config_t cameraConfig = {
        .pixelFormat = kVIDEO_PixelFormatRGB565,
        .bytesPerPixel = BYTE_PER_PIXEL,
        .resolution = FSL_VIDEO_RESOLUTION(CAMERA_WIDTH, CAMERA_HEIGHT),
        .frameBufferLinePitch_Bytes = CAMERA_WIDTH * BYTE_PER_PIXEL,
        .interface = kCAMERA_InterfaceGatedClock,
        .controlFlags = CAMERA_CONTROL_FLAGS,
        .framePerSec = 30,
    };
    CAMERA_DEVICE_Init(&cameraDevice, &cameraConfig);
    CAMERA_RECEIVER_Init(&cameraReceiver, &cameraConfig, NULL, NULL);
}

/*!
 * @brief Initialize the PXP.
 */
static void InitPxp(void)
{
    PXP_Init(APP_PXP);

    PXP_SetProcessSurfaceBackGroundColor(APP_PXP, 0U);

    /* PS configure. */
    const pxp_ps_buffer_config_t psBufferConfig = {
        .pixelFormat = kPXP_PsPixelFormatRGB565,
        .swapByte = false,
        .bufferAddr = 0U,
        .bufferAddrU = 0U,
        .bufferAddrV = 0U,
        .pitchBytes = CAMERA_WIDTH * BYTE_PER_PIXEL
    };
    PXP_SetProcessSurfaceBufferConfig(APP_PXP, &psBufferConfig);

    /* Disable AS. */
    PXP_SetAlphaSurfacePosition(APP_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);

    /* Output config. */
    const pxp_output_buffer_config_t outputBufferConfig = {
        .pixelFormat = kPXP_OutputPixelFormatRGB565,
        .interlacedMode = kPXP_OutputProgressive,
        .buffer0Addr = 0U,
        .buffer1Addr = 0U,
        .pitchBytes = LCD_WIDTH * BYTE_PER_PIXEL,
        .width = LCD_WIDTH,
        .height = LCD_HEIGHT
    };
    PXP_SetOutputBufferConfig(APP_PXP, &outputBufferConfig);

    /* Disable CSC1, it is enabled by default. */
    PXP_EnableCsc1(APP_PXP, false);

    PXP_SetProcessSurfaceScaler(APP_PXP,CAMERA_WIDTH, CAMERA_HEIGHT, LCD_WIDTH, LCD_HEIGHT);
    PXP_SetProcessSurfacePosition(APP_PXP, 0u, 0u, LCD_WIDTH - 1U, LCD_HEIGHT - 1U);
    PXP_SetRotateConfig(APP_PXP, kPXP_RotateOutputBuffer, kPXP_Rotate180, kPXP_FlipDisable);
}

/*!
 * @brief PXP Scale.
 */
static void PxpProcess(uint32_t inBuffer, uint32_t outBuffer)
{
    APP_PXP->PS_BUF = inBuffer;
    APP_PXP->OUT_BUF = outBuffer;

    PXP_Start(APP_PXP);

    /* Wait for process complete. */
    while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(APP_PXP)))
    {
    }

    PXP_ClearStatusFlags(APP_PXP, kPXP_CompleteFlag);
}

/*!
 * @brief Initialize the LCD interface
 */
static void InitLcdIf(void)
{
    volatile uint32_t i = 0x100U;

    clock_video_pll_config_t clock_config = {
        .loopDivider = 31,
        .postDivider = 8,
        .numerator = 0,
        .denominator = 0
    };

    gpio_pin_config_t pin_config = {
        kGPIO_DigitalOutput, 0,
    };

    const elcdif_rgb_mode_config_t lcdConfig = {
        .panelWidth = LCD_WIDTH,
        .panelHeight = LCD_HEIGHT,
        .hsw = LCD_HSW,
        .hfp = LCD_HFP,
        .hbp = LCD_HBP,
        .vsw = LCD_VSW,
        .vfp = LCD_VFP,
        .vbp = LCD_VBP,
        .polarityFlags = LCD_POL_FLAGS,
        .bufferAddr = 0,
        .pixelFormat = kELCDIF_PixelFormatRGB565,
        .dataBus = LCDIF_DATA_BUS,
    };

    /* Configure Clock. */
    CLOCK_InitVideoPll(&clock_config);
    CLOCK_SetMux(kCLOCK_Lcdif1PreMux, 2);
    CLOCK_SetDiv(kCLOCK_Lcdif1PreDiv, 4);
    CLOCK_SetMux(kCLOCK_Lcdif1Mux, 0);
    CLOCK_SetDiv(kCLOCK_Lcdif1Div, 1);

    /* Reset the LCD. */
    GPIO_PinInit(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, &pin_config);
    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);
    while (i--)
    {
    }
    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);

    ELCDIF_RgbModeInit(APP_ELCDIF, &lcdConfig);

    /* Backlight. */
    pin_config.outputLogic = 1;
    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &pin_config);
}

static void InitTouch(void)
{
    /* Initialize the touch handle. */
    FT5406_Init(&touchHandle, LPI2C1);
}

touch_event_t TouchPoll(void)
{
    touch_event_t touch_event;
    int touch_x;
    int touch_y;

    if (kStatus_Success != FT5406_GetSingleTouch(&touchHandle, &touch_event, &touch_x, &touch_y))
    {
        return false;
    }
    else if ((touch_event == kTouch_Down))
    {
        return true;
    }
    return false;
}

static status_t sdcardWaitCardInsert(void)
{
    /* Save host information. */
    g_sd.host.base = SD_HOST_BASEADDR;
    g_sd.host.sourceClock_Hz = SD_HOST_CLK_FREQ;
    /* card detect type */
    g_sd.usrParam.cd = &s_sdCardDetect;
    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("\r\nSD host init fail\r\n");
        return kStatus_Fail;
    }

    /* power off card */
    SD_PowerOffCard(g_sd.host.base, g_sd.usrParam.pwr);
    /* wait card insert */
    if (SD_WaitCardDetectStatus(SD_HOST_BASEADDR, &s_sdCardDetect, true) == kStatus_Success)
    {
        /* power on the card */
        SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);
    }
    else
    {
        PRINTF("\r\nCard detect fail.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

static status_t MountSDCard(void)
{
    DIR directory; /* Directory object */

    if (sdcardWaitCardInsert() != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if(f_mount(&g_fileSystem, driverNumberBuffer, 0U))
    {
        PRINTF("ERROR: Mount volume failed.\r\n");
        return kStatus_Fail;
    }

#if(_FS_RPATH >= 2U)
    if(f_chdrive((char const *)&driverNumberBuffer[0U]))
    {
        PRINTF("ERROR: Change drive failed.\r\n");
        return kStatus_Fail;
    }
#endif

    if (f_opendir(&directory, "/"))
    {
        PRINTF("ERROR: Open directory failed.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}
static void rgb565_2_rgb888(uint8_t *rgb565, uint8_t *rgb888, int pixels)
{
    /* RGB565 in little-endian: "G2G1G0B4B3B2B1B0 R4R3R2R1R0G5G4G3 */
    for(uint32_t i=0; i<pixels; i++)
    {
        rgb888[3*i] = ((rgb565[2*i] & 0x1F)<<3) | (rgb565[2*i] & 0x07);/*B4B3B2B1B0B2B1B0*/
        rgb888[3*i+1] = ((rgb565[2*i+1] & 0x07)<<5) | ((rgb565[2*i] & 0xE0)>>3) | ((rgb565[2*i] & 0x60)>>5);/*G5G4G3G2G1G0G1G0*/
        rgb888[3*i+2] = (rgb565[2*i+1] & 0xF8) | ((rgb565[2*i+1] & 0x38)>>3); /*R4R3R2R1R0R2R1R0*/
    }
}

static status_t JpegCompress(uint32_t buffer)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    uint16_t (*scanlines)[IMAGE_WIDTH] = (uint16_t (*)[IMAGE_WIDTH])buffer;
    JSAMPROW row_pointer;

    FILE fJpeg;
    char JpegFilename[20];
    static uint32_t JpegFilenameIndex = 0u;

    PRINTF("Start of JPEG Compression... ");

    if(JpegFilenameIndex > 9999u)
    {
        JpegFilenameIndex = 0u;
    }

    sprintf(JpegFilename, "/IMG_%04d.jpg", ++JpegFilenameIndex);

    /* Open the output file. */
    if(f_open(&fJpeg, _T(JpegFilename), (FA_WRITE | FA_CREATE_ALWAYS)))
    {
        PRINTF("ERROR: can't open %s\n", JpegFilename);
        return kStatus_Fail;
    }

    /* Initialize the JPEG compression object with default error handling. */
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    /* Initialize JPEG parameters.*/
    cinfo.in_color_space = JCS_RGB; /* arbitrary guess */
    jpeg_set_defaults(&cinfo);

    /* Specify the source format. */
    cinfo.in_color_space = JCS_RGB;
    cinfo.input_components = 3u;
    cinfo.data_precision = 8u;
    cinfo.image_width = (JDIMENSION)IMAGE_WIDTH;
    cinfo.image_height = (JDIMENSION)IMAGE_HEIGHT;

    /* Now that we know input colorspace, fix colorspace-dependent defaults */
    jpeg_default_colorspace(&cinfo);

    /* Specify data destination for compression */
    jpeg_stdio_dest(&cinfo, &fJpeg);

    jpeg_set_quality(&cinfo, 100u, true);

    /* Start compressor */
    jpeg_start_compress(&cinfo, TRUE);

    /* Process data */
    while (cinfo.next_scanline < cinfo.image_height)
    {
        rgb565_2_rgb888((uint8_t *)(scanlines + cinfo.next_scanline), (uint8_t *)(JpegScanlines), IMAGE_WIDTH);
        row_pointer = (JSAMPROW)JpegScanlines;
        jpeg_write_scanlines(&cinfo, &row_pointer, COMPRESS_LINES);
    }

    /* Finish compression and release memory */
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    f_close(&fJpeg);

    PRINTF("Done: %s is saved. \r\n", JpegFilename);

    return kStatus_Success;
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t csiFrameBufPtr1, csiFrameBufPtr2;
    uint32_t lcdFrameBufPtr1, lcdFrameBufPtr2;

    /*Config MPU.*/
    BOARD_ConfigMPU();

    /* Configure clock */
    BOARD_BootClockRUN();
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 0x12U);
    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);
    SEMC->SDRAMCR3 |= SEMC_SDRAMCR3_REN_MASK;

    /* Configure pin mux. */
    BOARD_InitDEBUG_UARTPins();
    BOARD_InitSDRAMPins();
    BOARD_InitLPI2C1Pins();
    BOARD_InitCSIPins();
    BOARD_InitLCDPins();
    BOARD_InitSDPins();

    /* Configure debug console. */
    BOARD_InitDebugConsole();

    MountSDCard();

    InitLPI2C1();
    InitCamera();
    InitPxp();
    InitLcdIf();
    InitTouch();

    PRINTF("Camera demo starting...\r\n");

    lcdFrameBufPtr1 = (uint32_t)lcdFrameBuf[0];
    lcdFrameBufPtr2 = (uint32_t)lcdFrameBuf[1];

    memset(csiFrameBuf, 0, sizeof(csiFrameBuf));
    memset(lcdFrameBuf, 0, sizeof(lcdFrameBuf));

    /* Submit the empty frame buffers to buffer queue. */
    for (uint32_t i = 0; i < CAMERA_FRAME_BUFFER_COUNT; i++)
    {
        CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(csiFrameBuf[i]));
    }

    CAMERA_DEVICE_Start(&cameraDevice);
    CAMERA_RECEIVER_Start(&cameraReceiver);

    /*
     * The LCDIF has active buffer and inactive buffer, so get two buffers here.
     */
    /* Wait to get the full frame buffer to show. */
    while (kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &csiFrameBufPtr1))
    {
    }
    PxpProcess(csiFrameBufPtr1, lcdFrameBufPtr1);

    /* Wait to get the full frame buffer to show. */
    while (kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &csiFrameBufPtr2))
    {
    }
    PxpProcess(csiFrameBufPtr2, lcdFrameBufPtr2);

    APP_ELCDIF->CUR_BUF = lcdFrameBufPtr1;
    APP_ELCDIF->NEXT_BUF = lcdFrameBufPtr2;
    ELCDIF_RgbModeStart(APP_ELCDIF);

    while (1)
    {
        ELCDIF_ClearInterruptStatus(APP_ELCDIF, kELCDIF_CurFrameDone);
        /* Wait the LCD current transfer done. */
        while (!(kELCDIF_CurFrameDone & ELCDIF_GetInterruptStatus(APP_ELCDIF)))
        {
        }

        CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, csiFrameBufPtr1);

        csiFrameBufPtr1 = csiFrameBufPtr2;
        lcdFrameBufPtr2 = lcdFrameBufPtr1;

        /* Wait CSI to get new full frame buffer. */
        while (kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &csiFrameBufPtr2))
        {
        }

        PxpProcess(csiFrameBufPtr2, lcdFrameBufPtr2);
        ELCDIF_SetNextBufferAddr(APP_ELCDIF, lcdFrameBufPtr2);

        if(TouchPoll())
        {
            JpegCompress(csiFrameBufPtr2);
        }
    }
}
