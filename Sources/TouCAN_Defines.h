/*  SPDX-License-Identifier: BSD-2-Clause OR GPL-3.0-or-later */
/*
 *  TouCAN - macOS User-Space Driver for Rusoku TouCAN USB Adapters
 *
 *  Copyright (c) 2020-2024 Uwe Vogt, UV Software, Berlin (info@mac-can.com)
 *  All rights reserved.
 *
 *  This file is part of MacCAN-TouCAN.
 *
 *  MacCAN-TouCAN is dual-licensed under the BSD 2-Clause "Simplified" License
 *  and under the GNU General Public License v3.0 (or any later version). You can
 *  choose between one of them if you use MacCAN-TouCAN in whole or in part.
 *
 *  BSD 2-Clause "Simplified" License:
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  MacCAN-TouCAN IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF MacCAN-TouCAN, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  GNU General Public License v3.0 or later:
 *  MacCAN-TouCAN is free software : you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  MacCAN-TouCAN is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with MacCAN-TouCAN.  If not, see <https://www.gnu.org/licenses/>.
 */
/** @addtogroup  can_api
 *  @{
 */
#ifndef CANAPI_TOUCAN_H_INCLUDED
#define CANAPI_TOUCAN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/*  -----------  includes  ------------------------------------------------
 */

#include <stdint.h>                     /* C99 header for sized integer types */
#include <stdbool.h>                    /* C99 header for boolean type */


/*  -----------  options  ------------------------------------------------
 */


/*  -----------  defines  ------------------------------------------------
 */

/** @name  CAN API Interfaces
 *  @brief TouCAN USB channel no.
 *  @{ */
#define TOUCAN_USB_CHANNEL0        0    /**< Rusoku TouCAN USB Interface, Channel 0 */
#define TOUCAN_USB_CHANNEL1        1    /**< Rusoku TouCAN USB Interface, Channel 1 */
#define TOUCAN_USB_CHANNEL2        2    /**< Rusoku TouCAN USB Interface, Channel 2 */
#define TOUCAN_USB_CHANNEL3        3    /**< Rusoku TouCAN USB Interface, Channel 3 */
#define TOUCAN_USB_CHANNEL4        4    /**< Rusoku TouCAN USB Interface, Channel 4 */
#define TOUCAN_USB_CHANNEL5        5    /**< Rusoku TouCAN USB Interface, Channel 5 */
#define TOUCAN_USB_CHANNEL6        6    /**< Rusoku TouCAN USB Interface, Channel 6 */
#define TOUCAN_USB_CHANNEL7        7    /**< Rusoku TouCAN USB Interface, Channel 7 */
#define TOUCAN_BOARDS             (8)   /**< number of Rusoku TouCAN Interface boards */
// alternative defines
#define TOUCAN_USB1                TOUCAN_USB_CHANNEL0
#define TOUCAN_USB2                TOUCAN_USB_CHANNEL1
#define TOUCAN_USB3                TOUCAN_USB_CHANNEL2
#define TOUCAN_USB4                TOUCAN_USB_CHANNEL3
#define TOUCAN_USB5                TOUCAN_USB_CHANNEL4
#define TOUCAN_USB6                TOUCAN_USB_CHANNEL5
#define TOUCAN_USB7                TOUCAN_USB_CHANNEL6
#define TOUCAN_USB8                TOUCAN_USB_CHANNEL7
/** @} */

/** @name  CAN API Error Codes
 *  @brief TouCAN specific error code
 *  @{ */
#define TOUCAN_ERR_OFFSET      (-500)   /**< offset for TouCAN-specific errors */
#define TOUCAN_ERR_UNKNOWN     (-599)   /**< unknown error */
/** @} */

/** @name  CAN API Property Value
 *  @brief TouCAN parameter to be read or written
 *  @{ */
#define TOUCAN_GET_HARDWARE_VERSION    (CANPROP_GET_VENDOR_PROP + 0x10U)  /**< hardware version as "0xggrrss00" (uint23_t) */
#define TOUCAN_GET_FIRMWARE_VERSION    (CANPROP_GET_VENDOR_PROP + 0x11U)  /**< firmware version as "0xggrrss00" (uint23_t) */
#define TOUCAN_GET_BOOTLOADER_VERSION  (CANPROP_GET_VENDOR_PROP + 0x12U)  /**< boot-loader version as "0xggrrss00" (uint23_t) */
#define TOUCAN_GET_SERIAL_NUMBER       (CANPROP_GET_VENDOR_PROP + 0x13U)  /**< serial no. in hex (uint23_t) */
#define TOUCAN_GET_VID_PID             (CANPROP_GET_VENDOR_PROP + 0x16U)  /**< VID & PID (uint23_t) */
#define TOUCAN_GET_DEVICE_ID           (CANPROP_GET_VENDOR_PROP + 0x17U)  /**< device id. (uint23_t) */
#define TOUCAN_GET_VENDOR_URL          (CANPROP_GET_VENDOR_PROP + 0x18U)  /**< URL of Rusoku's website (uint23_t) */
#define TOUCAN_MAX_BUFFER_SIZE   256U   /**< max. buffer size for CAN_GetValue/CAN_SetValue */
/** @} */

/** @name  CAN API Library ID
 *  @brief Library ID and dynamic library names
 *  @{ */
#define TOUCAN_LIB_ID            500    /**< library ID (CAN/COP API V1 compatible) */
#if defined(_WIN32) || defined (_WIN64)
 #define TOUCAN_LIB_CANLIB      "(none)"
 #define TOUCAN_LIB_WRAPPER     "u3cantou.dll"
#elif defined(__APPLE__)
 #define TOUCAN_LIB_CANLIB      "(none)"
 #define TOUCAN_LIB_WRAPPER     "libUVCANTOU.dylib"
#else
#error Platform not supported
#endif
/** @} */

/** @name  Miscellaneous
 *  @brief More or less useful stuff
 *  @{ */
#define TOUCAN_LIB_VENDOR       "Rusoku technologijos UAB, Lithuania"
#define TOUCAN_LIB_WEBSITE      "www.rusoku.com"
#define TOUCAN_LIB_HAZARD_NOTE  "If you connect your CAN device to a real CAN network when using this library,\n" \
                                "you might damage your application."
/** @} */

#ifdef __cplusplus
}
#endif
#endif /* CANAPI_TOUCAN_H_INCLUDED */
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de, Homepage: https://www.uv-software.de/
 */
