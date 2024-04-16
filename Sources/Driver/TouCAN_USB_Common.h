/*  SPDX-License-Identifier: BSD-2-Clause OR GPL-3.0-or-later */
/*
 *  TouCAN - macOS User-Space Driver for Rusoku TouCAN USB Adapters
 *
 *  Copyright (c) 2021-2024 Uwe Vogt, UV Software, Berlin (info@mac-can.com)
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
 *  MacCAN-TouCAN is free software: you can redistribute it and/or modify
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
#ifndef TOUCAN_USB_COMMON_H_INCLUDED
#define TOUCAN_USB_COMMON_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

#define TOUCAN_VENDOR_ID  (UInt16)0x16D0
#define TOUCAN_USB_ID     (UInt16)0x0EAC

#define TOUCAN_VENDOR_NAME  "Rusoku technologijos UAB, Lithuania"
#define TOUCAN_VENDOR_URL   "www.rusoku.com"

// TODO: move this into USB_Driver.h (when MacCAN enpoind module realized)
#define TOUCAN_USB_TX_DATA_PIPE_REF  1U
#define TOUCAN_USB_RX_DATA_PIPE_REF  2U
#define TOUCAN_USB_TX_DATA_PIPE_SIZE  64U
#define TOUCAN_USB_RX_DATA_PIPE_SIZE  64U
#define TOUCAN_USB_TX_DATA_FRAME_SIZE  18U
#define TOUCAN_USB_RX_DATA_FRAME_SIZE  18U
#define TOUCAN_USB_TX_DATA_FRAME_CNT  (TOUCAN_USB_TX_DATA_PIPE_SIZE / TOUCAN_USB_TX_DATA_FRAME_SIZE)
#define TOUCAN_USB_RX_DATA_FRAME_CNT  (TOUCAN_USB_RX_DATA_PIPE_SIZE / TOUCAN_USB_RX_DATA_FRAME_SIZE)
// TODO: end!

#define TOUCAN_RCV_QUEUE_SIZE  65536
#define TOUCAN_TRM_QUEUE_SIZE  256

#define TOUCAN_MAX_NAME_LENGTH  256
#define TOUCAN_MAX_STRING_LENGTH  80

#define TOUCAN_ERROR_SUCCESS  0
#define TOUCAN_ERROR_OFFSET  (-100)

/* ---  general CAN data types and defines  ---
 */
#if (OPTION_CANAPI_DRIVER != 0)
    #include "CANAPI_Types.h"
    /* typedefs from CAN API V3 */
    typedef can_message_t TouCAN_CanMessage_t;
    typedef can_timestamp_t TouCAN_Timestamp_t;
#else
    /* typedefs from <3rdParty.h> */
    // TODO: insert coin here
#endif

/* TouCAN return error codes (HAL)
 */
#define TOUCAN_RETVAL_OK       0x00
#define TOUCAN_RETVAL_ERROR    0x01
#define TOUCAN_RETVAL_BUSY     0x02
#define TOUCAN_RETVAL_TIMEOUT  0x03

/* TouCAN init string flags (32 bit)
 */
#define TOUCAN_ENABLE_SILENT_MODE            0x00000001
#define TOUCAN_ENABLE_LOOPBACK_MODE          0x00000002
#define TouCAN_DISABLE_RETRANSMITION         0x00000004
#define TOUCAN_ENABLE_AUTOMATIC_WAKEUP_MODE  0x00000008
#define TOUCAN_ENABLE_AUTOMATIC_BUS_OFF      0x00000010
#define TOUCAN_ENABLE_TTM_MODE               0x00000020
#define TOUCAN_ENABLE_RX_FIFO_LOCKED_MODE    0x00000040
#define TOUCAN_ENABLE_TX_FIFO_PRIORITY       0x00000080
#define TOUCAN_ENABLE_STATUS_MESSAGES        0x00000100
#define TOUCAN_ENABLE_TIMESTAMP_DELAY        0x00000200

/* TouCAN HAL return error codes
 */
typedef enum {
    HAL_OK =      0x00U,
    HAL_ERROR =   0x01U,
    HAL_BUSY =    0x02U,
    HAL_TIMEOUT = 0x03U
}   HAL_Status_t;

/* TouCAN CAN interface state
 */
typedef enum {
    HAL_CAN_STATE_RESET =         0x00U,
    HAL_CAN_STATE_READY =         0x01U,
    HAL_CAN_STATE_LISTENING =     0x02U,
    HAL_CAN_STATE_SLEEP_PENDING = 0x03U,
    HAL_CAN_STATE_SLEEP_ACTIVE =  0x04U,
    HAL_CAN_STATE_ERROR =         0x05U
}   HAL_CAN_State_t;

/* TouCAN CAN interface ERROR codes
 */
#define HAL_CAN_ERROR_NONE             0x00000000U
#define HAL_CAN_ERROR_EWG              0x00000001U
#define HAL_CAN_ERROR_EPV              0x00000002U
#define HAL_CAN_ERROR_BOF              0x00000004U
#define HAL_CAN_ERROR_STF              0x00000008U
#define HAL_CAN_ERROR_FOR              0x00000010U
#define HAL_CAN_ERROR_ACK              0x00000020U
#define HAL_CAN_ERROR_BR               0x00000040U
#define HAL_CAN_ERROR_BD               0x00000080U
#define HAL_CAN_ERROR_CRC              0x00000100U
#define HAL_CAN_ERROR_RX_FOV0          0x00000200U
#define HAL_CAN_ERROR_RX_FOV1          0x00000400U
#define HAL_CAN_ERROR_TX_ALST0         0x00000800U
#define HAL_CAN_ERROR_TX_TERR0         0x00001000U
#define HAL_CAN_ERROR_TX_ALST1         0x00002000U
#define HAL_CAN_ERROR_TX_TERR1         0x00004000U
#define HAL_CAN_ERROR_TX_ALST2         0x00008000U
#define HAL_CAN_ERROR_TX_TERR2         0x00010000U
#define HAL_CAN_ERROR_TIMEOUT          0x00020000U
#define HAL_CAN_ERROR_NOT_INITIALIZED  0x00040000U
#define HAL_CAN_ERROR_NOT_READY        0x00080000U
#define HAL_CAN_ERROR_NOT_STARTED      0x00100000U
#define HAL_CAN_ERROR_PARAM            0x00200000U

/* TouCAN CAN acceptance filter modes
 */
typedef enum {
    FILTER_ACCEPT_ALL = 0,
    FILTER_REJECT_ALL = 1,
    FILTER_VALUE = 2
}   TouCAN_FilterMode_t;

#endif /* TOUCAN_USB_COMMON_H_INCLUDED */
