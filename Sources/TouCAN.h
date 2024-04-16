//  SPDX-License-Identifier: BSD-2-Clause OR GPL-3.0-or-later
//
//  TouCAN - macOS User-Space Driver for Rusoku TouCAN USB Adapters
//
//  Copyright (c) 2020-2024 Uwe Vogt, UV Software, Berlin (info@mac-can.com)
//  All rights reserved.
//
//  This file is part of MacCAN-TouCAN.
//
//  MacCAN-TouCAN is dual-licensed under the BSD 2-Clause "Simplified" License
//  and under the GNU General Public License v3.0 (or any later version). You can
//  choose between one of them if you use MacCAN-TouCAN in whole or in part.
//
//  BSD 2-Clause "Simplified" License:
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//  1. Redistributions of source code must retain the above copyright notice, this
//     list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  MacCAN-TouCAN IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF MacCAN-TouCAN, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//  GNU General Public License v3.0 or later:
//  MacCAN-TouCAN is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  MacCAN-TouCAN is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with MacCAN-TouCAN.  If not, see <https://www.gnu.org/licenses/>.
//
#ifndef TOUCAN_H_INCLUDED
#define TOUCAN_H_INCLUDED

#include "TouCAN_Defines.h"
#include "TouCAN_Defaults.h"
#include "CANAPI.h"

/// \name   TouCAN
/// \brief  TouCAN dynamic library
/// \{
#define TOUCAN_LIBRARY_ID  CANLIB_RUSOKU_LT
#if (OPTION_CANAPI_TOUCAN_DYLIB != 0)
#define TOUCAN_LIBRARY_NAME  CANDLL_RUSOKU_LT
#else
#define TOUCAN_LIBRARY_NAME  "libTouCAN.dylib"
#endif
#define TOUCAN_LIBRARY_VENDOR  "UV Software, Berlin"
#define TOUCAN_LIBRARY_LICENSE  "BSD-2-Clause OR GPL-3.0-or-later"
#define TOUCAN_LIBRARY_COPYRIGHT  "Copyright (C) 2020-2024  Uwe Vogt, UV Software, Berlin"
#define TOUCAN_LIBRARY_HAZARD_NOTE  "If you connect your CAN device to a real CAN network when using this library,\n" \
                                    "you might damage your application."
/// \}

/// \name   TouCAN API
/// \brief  MacCAN driver for Rusoku TouCAN USB interfaces
/// \note   See CCanApi for a description of the overridden methods
/// \{
class CANCPP CTouCAN : public CCanApi {
private:
    CANAPI_Handle_t m_Handle;  ///< CAN interface handle
public:
    // constructor / destructor
    CTouCAN();
    ~CTouCAN();
    // CTouCAN-specific error codes (CAN API V3 extension)
    enum EErrorCodes {
        // note: range 0...-99 is reserved by CAN API V3
        GeneralError = VendorSpecific
    };
    // CCanApi overrides
    static bool GetFirstChannel(SChannelInfo &info, void *param = NULL);
    static bool GetNextChannel(SChannelInfo &info, void *param = NULL);

    static CANAPI_Return_t ProbeChannel(int32_t channel, const CANAPI_OpMode_t &opMode, const void *param, EChannelState &state);
    static CANAPI_Return_t ProbeChannel(int32_t channel, const CANAPI_OpMode_t &opMode, EChannelState &state);

    CANAPI_Return_t InitializeChannel(int32_t channel, const CANAPI_OpMode_t &opMode, const void *param = NULL);
    CANAPI_Return_t TeardownChannel();
    CANAPI_Return_t SignalChannel();

    CANAPI_Return_t StartController(CANAPI_Bitrate_t bitrate);
    CANAPI_Return_t ResetController();

    CANAPI_Return_t WriteMessage(CANAPI_Message_t message, uint16_t timeout = 0U);
    CANAPI_Return_t ReadMessage(CANAPI_Message_t &message, uint16_t timeout = CANREAD_INFINITE);

    CANAPI_Return_t GetStatus(CANAPI_Status_t &status);
    CANAPI_Return_t GetBusLoad(uint8_t &load);

    CANAPI_Return_t GetBitrate(CANAPI_Bitrate_t &bitrate);
    CANAPI_Return_t GetBusSpeed(CANAPI_BusSpeed_t &speed);

    CANAPI_Return_t GetProperty(uint16_t param, void *value, uint32_t nbyte);
    CANAPI_Return_t SetProperty(uint16_t param, const void *value, uint32_t nbyte);

    char *GetHardwareVersion();  // (for compatibility reasons)
    char *GetFirmwareVersion();  // (for compatibility reasons)
    static char *GetVersion();  // (for compatibility reasons)

    static CANAPI_Return_t MapIndex2Bitrate(int32_t index, CANAPI_Bitrate_t &bitrate);
    static CANAPI_Return_t MapString2Bitrate(const char *string, CANAPI_Bitrate_t &bitrate, bool &data, bool &sam);
    static CANAPI_Return_t MapBitrate2String(CANAPI_Bitrate_t bitrate, char *string, size_t length, bool data = false, bool sam = false);
    static CANAPI_Return_t MapBitrate2Speed(CANAPI_Bitrate_t bitrate, CANAPI_BusSpeed_t &speed);
};
/// \}

/// \name   TouCAN Property IDs
/// \brief  Properties that can be read (or written)
/// \{
#define TOUCAN_PROPERTY_CANAPI              (CANPROP_GET_SPEC)
#define TOUCAN_PROPERTY_VERSION             (CANPROP_GET_VERSION)
#define TOUCAN_PROPERTY_PATCH_NO            (CANPROP_GET_PATCH_NO)
#define TOUCAN_PROPERTY_BUILD_NO            (CANPROP_GET_BUILD_NO)
#define TOUCAN_PROPERTY_LIBRARY_ID          (CANPROP_GET_LIBRARY_ID)
#define TOUCAN_PROPERTY_LIBRARY_NAME        (CANPROP_GET_LIBRARY_DLLNAME)
#define TOUCAN_PROPERTY_LIBRARY_VENDOR      (CANPROP_GET_LIBRARY_VENDOR)
#define TOUCAN_PROPERTY_DEVICE_TYPE         (CANPROP_GET_DEVICE_TYPE)
#define TOUCAN_PROPERTY_DEVICE_NAME         (CANPROP_GET_DEVICE_NAME)
#define TOUCAN_PROPERTY_DEVICE_DRIVER       (CANPROP_GET_DEVICE_DLLNAME)
#define TOUCAN_PROPERTY_DEVICE_VENDOR       (CANPROP_GET_DEVICE_VENDOR)
#define TOUCAN_PROPERTY_OP_CAPABILITY       (CANPROP_GET_OP_CAPABILITY)
#define TOUCAN_PROPERTY_OP_MODE             (CANPROP_GET_OP_MODE)
#define TOUCAN_PROPERTY_BITRATE             (CANPROP_GET_BITRATE)
#define TOUCAN_PROPERTY_SPEED               (CANPROP_GET_SPEED)
#define TOUCAN_PROPERTY_STATUS              (CANPROP_GET_STATUS)
#define TOUCAN_PROPERTY_BUSLOAD             (CANPROP_GET_BUSLOAD)
#define TOUCAN_PROPERTY_NUM_CHANNELS        (CANPROP_GET_NUM_CHANNELS)
#define TOUCAN_PROPERTY_CAN_CHANNEL         (CANPROP_GET_CAN_CHANNEL)
#define TOUCAN_PROPERTY_CAN_CLOCK           (CANPROP_GET_CAN_CLOCK)
#define TOUCAN_PROPERTY_TX_COUNTER          (CANPROP_GET_TX_COUNTER)
#define TOUCAN_PROPERTY_RX_COUNTER          (CANPROP_GET_RX_COUNTER)
#define TOUCAN_PROPERTY_ERR_COUNTER         (CANPROP_GET_ERR_COUNTER)
#define TOUCAN_PROPERTY_RCV_QUEUE_SIZE      (CANPROP_GET_RCV_QUEUE_SIZE)
#define TOUCAN_PROPERTY_RCV_QUEUE_HIGH      (CANPROP_GET_RCV_QUEUE_HIGH)
#define TOUCAN_PROPERTY_RCV_QUEUE_OVFL      (CANPROP_GET_RCV_QUEUE_OVFL)
#define TOUCAN_PROPERTY_HARDWARE_VERSION    (TOUCAN_GET_HARDWARE_VERSION)
#define TOUCAN_PROPERTY_FIRMWARE_VERSION    (TOUCAN_GET_FIRMWARE_VERSION)
#define TOUCAN_PROPERTY_BOOTLOADER_VERSION  (TOUCAN_GET_BOOTLOADER_VERSION)
#define TOUCAN_PROPERTY_SERIAL_NUMBER       (TOUCAN_GET_SERIAL_NUMBER)
#define TOUCAN_PROPERTY_VID_PID             (TOUCAN_GET_VID_PID)
#define TOUCAN_PROPERTY_DEVICE_ID           (TOUCAN_GET_DEVICE_ID)
#define TOUCAN_PROPERTY_VENDOR_URL          (TOUCAN_GET_VENDOR_URL)
/// \}

#endif // TOUCAN_H_INCLUDED
