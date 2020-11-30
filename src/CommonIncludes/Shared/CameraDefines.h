//
// Created by U'Ren, Mason R (VEN) on 10/5/20.
//

#ifndef LIVEMETROLOGY_CAMERADEFINES_H
#define LIVEMETROLOGY_CAMERADEFINES_H

#include "OSDefines.h"

/**
 * Camera Defines
 */
#if USE_XIMEA_SDK
//    #include <xiApi.h>
    #include "../lib/Ximea/include/xiApi.h"

#else
    // Barrowed from xiApi.h
    // Image output format enumerator.
    typedef enum
    {
        XI_MONO8                     =0, // 8 bits per pixel.   [Intensity] (see Note5,Note6)
        XI_MONO16                    =1, // 16 bits per pixel. [Intensity LSB] [Intensity MSB] (see Note5,Note6)
        XI_RGB24                     =2, // RGB data format. [Blue][Green][Red] (see Note5)
        XI_RGB32                     =3, // RGBA data format.   [Blue][Green][Red][0] (see Note5)
        XI_RGB_PLANAR                =4, // RGB planar data format. [Red][Red]...[Green][Green]...[Blue][Blue]... (see Note5)
        XI_RAW8                      =5, // 8 bits per pixel raw data from sensor.      [pixel byte] raw data from transport (camera output)
        XI_RAW16                     =6, // 16 bits per pixel raw data from sensor.     [pixel byte low] [pixel byte high] 16 bits (depacked) raw data
        XI_FRM_TRANSPORT_DATA        =7, // Data from transport layer (e.g. packed). Depends on data on the transport layer (see Note7)
        XI_RGB48                     =8, // RGB data format. [Blue low byte][Blue high byte][Green low][Green high][Red low][Red high] (see Note5)
        XI_RGB64                     =9, // RGBA data format. [Blue low byte][Blue high byte][Green low][Green high][Red low][Red high][0][0] (Note5)
        XI_RGB16_PLANAR              =10, // RGB16 planar data format
        XI_RAW8X2                    =11, // 8 bits per pixel raw data from sensor(2 components in a row). [ch1 pixel byte] [ch2 pixel byte] 8 bits raw data from 2 chan$
        XI_RAW8X4                    =12, // 8 bits per pixel raw data from sensor(4 components in a row).      [ch1 pixel byte [ch2 pixel byte] [ch3 pixel byte] [ch4 p$
        XI_RAW16X2                   =13, // 16 bits per pixel raw data from sensor(2 components in a row).     [ch1 pixel byte low] [ch1 pixel byte high] [ch2 pixel by$
        XI_RAW16X4                   =14, // 16 bits per pixel raw data from sensor(4 components in a row).     [ch1 pixel byte low] [ch1 pixel byte high] [ch2 pixel by$
        XI_RAW32                     =15, // 32 bits per pixel raw data from sensor in integer format (LSB first). 4 bytes (LSB first) pixel (depacked) raw data
        XI_RAW32FLOAT                =16, // 32 bits per pixel raw data from sensor in single-precision floating point format. 4 bytes per pixel (depacked) raw data

    } XI_IMG_FORMAT;

    // Downsampling types
    typedef enum
    {
        XI_BINNING                   =0, // pixels are interpolated - better image
        XI_SKIPPING                  =1, // pixels are skipped - higher frame rate

    } XI_DOWNSAMPLING_TYPE;

    // Turn parameter On/Off
    typedef enum
    {
        XI_OFF                       =0, // Turn parameter off
        XI_ON                        =1, // Turn parameter on

    } XI_SWITCH;

    // structure containing information about parameters type
    typedef enum
    {
        xiTypeInteger                =0, // integer parameter type
        xiTypeFloat                  =1, // float parameter type
        xiTypeString                 =2, // string parameter type
        xiTypeEnum                   =3, // enumerator parameter type
        xiTypeBoolean                =4, // boolean parameter type
        xiTypeCommand                =5, // command parameter type
        xiTypeInteger64              =6, // 64bit integer parameter type

    } XI_PRM_TYPE;

    typedef enum {
        XI_OK = 0, // Function call succeeded
    } XI_RET;

#define  XI_PRM_EXPOSURE                                   "exposure"                               // Exposure time in microseconds
    #define  XI_PRM_IMAGE_DATA_FORMAT                          "imgdataformat"                          // Output data format. XI_IMG_FORMAT
    #define  XI_PRM_AUTO_WB                                    "auto_wb"                                // Automatic white balance
    #define  XI_PRM_DOWNSAMPLING_TYPE                          "downsampling_type"                      // Change image downsampling type. XI_DOWNSAMPLING_TYPE
    #define  XI_PRM_LENS_MODE                                  "lens_mode"                              // Status of lens control interface. This shall be set to XI_ON before any Lens operations.
    #define  XI_PRM_LENS_APERTURE_VALUE                        "lens_aperture_value"                    // Current lens aperture value in stops. Examples: 2.8, 4, 5.6, 8, 11
    #define  XI_PRM_DEVICE_NAME                                "device_name"                            // Return device name
    #define  XI_PRM_DEVICE_SN                                  "device_sn"                              // Return device serial number

    typedef int XI_RETURN;

    constexpr XI_RETURN xiGetDeviceInfoString(DWORD DevId, const char* prm, char* value, DWORD value_size) {
        return XI_OK;
    }

#endif // USE_XIMEA_SDK

#if USE_IMPERX_SDK
    // ImperX API
//    #include "IpxCameraApi.h"
    #include "../lib/ImperX/inc/IpxCameraApi.h"
#else

#endif // USE_IMPERX_SDK

#endif //LIVEMETROLOGY_CAMERADEFINES_H
