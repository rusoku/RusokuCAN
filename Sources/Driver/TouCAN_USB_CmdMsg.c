/*  SPDX-License-Identifier: BSD-2-Clause OR GPL-3.0-or-later */
/*
 *  TouCAN - macOS User-Space Driver for Rusoku TouCAN USB Adapters
 *
 *  Copyright (c) 2024 Uwe Vogt, UV Software, Berlin (info@mac-can.com)
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
#include "TouCAN_USB_CmdMsg.h"

#include "MacCAN_IOUsbKit.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

/* TouCAN USB commands for control transfer
 */
#define CMD_RESET                           0x00U
#define CMD_CAN_INTERFACE_INIT              0x01U
#define CMD_CAN_INTERFACE_DEINIT            0x02U
#define CMD_CAN_INTERFACE_START             0x03U
#define CMD_CAN_INTERFACE_STOP              0x04U
#define CMD_FILTER_STD_ACCEPT_ALL           0x05U
#define CMD_FILTER_STD_REJECT_ALL           0x06U
#define CMD_FILTER_EXT_ACCEPT_ALL           0x07U
#define CMD_FILTER_EXT_REJECT_ALL           0x08U
#define CMD_SET_FILTER_STD_LIST_MASK        0x09U
#define CMD_SET_FILTER_EXT_LIST_MASK        0x0AU
#define CMD_GET_FILTER_STD_LIST_MASK        0x0BU
#define CMD_GET_FILTER_EXT_LIST_MASK        0x0CU
#define CMD_GET_CAN_ERROR_STATUS            0x0DU
#define CMD_CLEAR_CAN_ERROR_STATUS          0x0DU
#define CMD_GET_STATISTICS                  0x0EU
#define CMD_CLEAR_STATISTICS                0x0FU
#define CMD_GET_HARDWARE_VERSION            0x10U
#define CMD_GET_FIRMWARE_VERSION            0x11U
#define CMD_GET_BOOTLOADER_VERSION          0x12U
#define CMD_GET_SERIAL_NUMBER               0x13U
#define CMD_SET_SERIAL_NUMBER               0x14U
#define CMD_RESET_SERIAL_NUMBER             0x15U
#define CMD_GET_VID_PID                     0x16U
#define CMD_GET_DEVICE_ID                   0x17U
#define CMD_GET_VENDOR                      0x18U
#define CMD_GET_LAST_ERROR_CODE             0x20U
#define CMD_CLEAR_LAST_ERROR_CODE           0x21U
#define CMD_GET_CAN_INTERFACE_STATE         0x22U
#define CMD_CLEAR_CAN_INTERFACE_STATE       0x23U
#define CMD_GET_CAN_INTERFACE_ERROR_CODE    0x24U
#define CMD_CLEAR_CAN_INTERFACE_ERROR_CODE  0x25U
#define CMD_SET_CAN_INTERFACE_DELAY         0x26U
#define CMD_GET_CAN_INTERFACE_DELAY         0x27U

/* TouCAN init
 */
int TouCAN_USB_CmdInitInterface(TouCAN_Handle_t handle, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw, uint32_t flags) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_HOST_TO_DEVICE | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_CAN_INTERFACE_INIT;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 9U;
    
    // tseg1
    data[0] = tseg1;
    // tseg2
    data[1] = tseg2;
    // sjw
    data[2] = sjw;
    // brp
    data[3] = (uint8_t)((brp >> 8) & 0xFFU);
    data[4] = (uint8_t)(brp & 0xFFU);
    // flags
    data[5] = (uint8_t)((flags >> 24) & 0xFFU);
    data[6] = (uint8_t)((flags >> 16) & 0xFFU);
    data[7] = (uint8_t)((flags >> 8) & 0xFFU);
    data[8] = (uint8_t)(flags & 0xFFU);

    // TouCAN_CAN_INTERFACE_INIT
    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)9, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    return (int)retVal;
}

/* TouCAN deinit
 */
int TouCAN_USB_CmdDeinitInterface(TouCAN_Handle_t handle) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;

    SetupPacket.RequestType = USBREQ_HOST_TO_DEVICE | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_CAN_INTERFACE_DEINIT;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 0U;
    
    // TouCAN_CAN_INTERFACE_DEINIT
    retVal = CANUSB_DeviceRequest(handle, SetupPacket, NULL, 0U, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    return (int)retVal;
}

/* TouCAN start
 */
int TouCAN_USB_CmdStartInterface(TouCAN_Handle_t handle) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;

    SetupPacket.RequestType = USBREQ_HOST_TO_DEVICE | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_CAN_INTERFACE_START;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 0U;
    
    // TouCAN_CAN_INTERFACE_START
    retVal = CANUSB_DeviceRequest(handle, SetupPacket, NULL, 0U, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    return (int)retVal;
}

/* TouCAN stop
 */
int TouCAN_USB_CmdStopInterface(TouCAN_Handle_t handle) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;

    SetupPacket.RequestType = USBREQ_HOST_TO_DEVICE | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_CAN_INTERFACE_STOP;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 0U;

    // TouCAN_CAN_INTERFACE_STOP
    retVal = CANUSB_DeviceRequest(handle, SetupPacket, NULL, 0U, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    return (int)retVal;
}

/* TouCAN HAL get last error
 */
int TouCAN_USB_CmdGetLastErrorCode(TouCAN_Handle_t handle, uint8_t *error) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    UInt8 LastErrorCode = 0U;
    UInt32 Transferred = 0U;
    
    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_LAST_ERROR_CODE;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 1U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)&LastErrorCode, (UInt16)1, (UInt32*)&Transferred);
    if (retVal < 0)
        return (int)retVal;
    
    if ((LastErrorCode != HAL_OK) & (Transferred != 1U))  // note: equivalent to logical OR !
        return (int)TOUCAN_ERROR_OFFSET - (int)LastErrorCode;
    
    if (error) {
        *error = (uint8_t)LastErrorCode;
    }
    return (int)retVal;
}

/* TouCAN get CAN interface ERROR:  hcan->ErrorCode;
 */
int TouCAN_USB_CmdGetInterfaceErrorCode(TouCAN_Handle_t handle, uint32_t *error) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_CAN_INTERFACE_ERROR_CODE;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 4U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, NULL);
    if (retVal < 0)
        return (int)retVal;

    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    if (error) {
        *error = (uint32_t)data[0] << 24;
        *error |= (uint32_t)data[1] << 16;
        *error |= (uint32_t)data[2] << 8;
        *error |= (uint32_t)data[3];
    }
    return (int)retVal;
}

/* TouCAN TouCAN_CLEAR_CAN_INTERFACE_ERROR_CODE:  hcan->ErrorCode;
 */
int TouCAN_USB_CmdClearInterfaceErrorCode(TouCAN_Handle_t handle)  {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;

    SetupPacket.RequestType = USBREQ_HOST_TO_DEVICE | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_CLEAR_CAN_INTERFACE_ERROR_CODE;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 0U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, NULL, 0U, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    return (int)retVal;
}

/* TouCAN TouCAN_GET_CAN_INTERFACE_STATE   hcan->State;
 */
int TouCAN_USB_CmdGetInterfaceState(TouCAN_Handle_t handle, uint8_t *state) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_CAN_INTERFACE_STATE;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 1U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)1, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    if (state) {
        *state = (uint8_t)data[0];
    }
    return (int)retVal;
}

#if (OPTION_TOUCAN_CANAL != 0)
/* TouCAN TouCAN_get_statistics VSCP
 */
int TouCAN_get_statistics(TouCAN_Handle_t handle, PCANALSTATISTICS statistics) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);
    UInt32 Transferred = 0U;
    //UCHAR LastErrorCode = 0U;

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_STATISTICS;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 28U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)28, (UInt32*)&Transferred);
    if (retVal < 0)
        return (int)retVal;

    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;

    if ((res != TOUCAN_RETVAL_OK) || (Transferred != 28U))
        return (int)TOUCAN_ERROR_OFFSET - (int)res;

    if (statistics) {
        statistics->cntReceiveFrames  = (((uint32_t)data[0] << 24) & 0xFF000000U);
        statistics->cntReceiveFrames |= (((uint32_t)data[1] << 16) & 0x00FF0000U);
        statistics->cntReceiveFrames |= (((uint32_t)data[2] <<  8) & 0x0000FF00U);
        statistics->cntReceiveFrames |=  ((uint32_t)data[3]        & 0x000000FFU);

        statistics->cntTransmitFrames  = (((uint32_t)data[4] << 24) & 0xFF000000U);
        statistics->cntTransmitFrames |= (((uint32_t)data[5] << 16) & 0x00FF0000U);
        statistics->cntTransmitFrames |= (((uint32_t)data[6] <<  8) & 0x0000FF00U);
        statistics->cntTransmitFrames |=  ((uint32_t)data[7]        & 0x000000FFU);

        statistics->cntReceiveData  = (((uint32_t)data[8] << 24) & 0xFF000000U);
        statistics->cntReceiveData |= (((uint32_t)data[9] << 16) & 0x00FF0000U);
        statistics->cntReceiveData |= (((uint32_t)data[10] << 8) & 0x0000FF00U);
        statistics->cntReceiveData |=  ((uint32_t)data[11]       & 0x000000FFU);

        statistics->cntTransmitData  = (((uint32_t)data[12] << 24) & 0xFF000000U);
        statistics->cntTransmitData |= (((uint32_t)data[13] << 16) & 0x00FF0000U);
        statistics->cntTransmitData |= (((uint32_t)data[14] << 8) & 0x0000FF00U);
        statistics->cntTransmitData |=  ((uint32_t)data[15]       & 0x000000FFU);

        statistics->cntOverruns =  (((uint32_t)data[16] << 24) & 0xFF000000U);
        statistics->cntOverruns |= (((uint32_t)data[17] << 16) & 0x00FF0000U);
        statistics->cntOverruns |= (((uint32_t)data[18] << 8)  & 0x0000FF00U);
        statistics->cntOverruns |=  ((uint32_t)data[19]        & 0x000000FFU);

        statistics->cntBusWarnings  = (((uint32_t)data[20] << 24) & 0xFF000000U);
        statistics->cntBusWarnings |= (((uint32_t)data[21] << 16) & 0x00FF0000U);
        statistics->cntBusWarnings |= (((uint32_t)data[22] <<  8) & 0x0000FF00U);
        statistics->cntBusWarnings |=  ((uint32_t)data[23]        & 0x000000FFU);

        statistics->cntBusOff =  (((uint32_t)data[24] << 24) & 0xFF000000U);
        statistics->cntBusOff |= (((uint32_t)data[25] << 16) & 0x00FF0000U);
        statistics->cntBusOff |= (((uint32_t)data[26] <<  8) & 0x0000FF00U);
        statistics->cntBusOff |=  ((uint32_t)data[27]        & 0x000000FFU);
    }
    //TouCAN_clear_statistics();
    return (int)retVal;
}

/* TouCAN_clear_statistics VSCP
 */
int TouCAN_clear_statistics(TouCAN_Handle_t handle) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;

    SetupPacket.RequestType = USBREQ_HOST_TO_DEVICE | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_CLEAR_STATISTICS;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 0U;

    retVal = CANUSB_ControlTransfer(handle, SetupPacket, NULL, 0U, NULL);
    if (retVal < 0)
        return (int)retVal;

    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;

    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;

    return (int)retVal;
}

/* TouCAN_get_status VSCP
 */
int TouCAN_USB_CmdGetChannelStatus(TouCAN_Handle_t handle, PCANALSTATUS status) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);
    UInt32 Transferred = 0U;

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_CAN_ERROR_STATUS;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 4U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, (UInt32*)&Transferred);
    if (retVal < 0)
        return (int)retVal;

    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;

    if ((res != TOUCAN_RETVAL_OK) || (Transferred != 4U))
        return (int)TOUCAN_ERROR_OFFSET - (int)res;

    if (status) {
        status->channel_status  = (((uint32_t)data[0] << 24) & 0xFF000000U);
        status->channel_status |= (((uint32_t)data[1] << 16) & 0x00FF0000U);
        status->channel_status |= (((uint32_t)data[2] << 8)  & 0x0000FF00U);
        status->channel_status |= (((uint32_t)data[3])       & 0x000000FFU);
    }
    return (int)retVal;
}
#endif

/* TouCAN_get_hardware_version
 */
int TouCAN_USB_CmdGetHardwareVersion(TouCAN_Handle_t handle, uint32_t *value) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_HARDWARE_VERSION;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 4U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    if (value) {
        *value = (uint32_t)data[0] << 24;
        *value |= (uint32_t)data[1] << 16;
        *value |= (uint32_t)data[2] << 8;
        *value |= (uint32_t)data[3];
    }
    return (int)retVal;
}

/* TouCAN_get_firmware_version
 */
int TouCAN_USB_CmdGetFirmwareVersion(TouCAN_Handle_t handle, uint32_t *value) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_FIRMWARE_VERSION;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 4U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    if (value) {
        *value = (uint32_t)data[0] << 24;
        *value |= (uint32_t)data[1] << 16;
        *value |= (uint32_t)data[2] << 8;
        *value |= (uint32_t)data[3];
    }
    return (int)retVal;
}

/* TouCAN_get_bootloader_version
 */
int TouCAN_USB_CmdGetBootloaderVersion(TouCAN_Handle_t handle, uint32_t *value) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_BOOTLOADER_VERSION;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 4U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    if (value) {
        *value = (uint32_t)data[0] << 24;
        *value |= (uint32_t)data[1] << 16;
        *value |= (uint32_t)data[2] << 8;
        *value |= (uint32_t)data[3];
    }
    return (int)retVal;
}

/* TouCAN_get_serial_number
 */
int TouCAN_USB_CmdGetSerialNumber(TouCAN_Handle_t handle, uint32_t *value) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_SERIAL_NUMBER;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 4U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    if (value) {
        *value = (uint32_t)data[0] << 24;
        *value |= (uint32_t)data[1] << 16;
        *value |= (uint32_t)data[2] << 8;
        *value |= (uint32_t)data[3];
    }
    return (int)retVal;
}

/* TouCAN_get_vid_pid
 */
int TouCAN_USB_CmdGetVidPid(TouCAN_Handle_t handle, uint32_t *value) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_VID_PID;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 4U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    if (value) {
        *value = (uint32_t)data[0] << 24;
        *value |= (uint32_t)data[1] << 16;
        *value |= (uint32_t)data[2] << 8;
        *value |= (uint32_t)data[3];
    }
    return (int)retVal;
}

/* TouCAN_get_device_id
 */
int TouCAN_USB_CmdGetDeviceId(TouCAN_Handle_t handle, uint32_t *value) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_DEVICE_ID;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 4U;

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    if (value) {
        *value = (uint32_t)data[0] << 24;
        *value |= (uint32_t)data[1] << 16;
        *value |= (uint32_t)data[2] << 8;
        *value |= (uint32_t)data[3];
    }
    return (int)retVal;
}

/* TouCAN_get_vendor
 */
int TouCAN_USB_CmdGetVendorName(TouCAN_Handle_t handle, size_t size, char *name) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_VENDOR;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 32U;  // FIXME: 4(?)

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)32, NULL);
    if (retVal < 0)
        return (int)retVal;
    
    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;
    
    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;
    
    if (name) {
        data[32] = '\0';
        strncpy(name, (const char *)data, size);
    }
    return (int)retVal;

}

int TouCAN_USB_CmdGetTransmitDelay(TouCAN_Handle_t handle, /*uint8_t channel,*/ uint32_t *delay) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);

    SetupPacket.RequestType = USBREQ_DEVICE_TO_HOST | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_GET_CAN_INTERFACE_DELAY;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 0U;  // FIXME: 4(?)

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, NULL);
    if (retVal < 0)
        return (int)retVal;

    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;

    if (res != TOUCAN_RETVAL_OK)
        return (int)TOUCAN_ERROR_OFFSET - (int)res;

    if (delay) {
        *delay = (uint32_t)data[0] << 24;
        *delay |= (uint32_t)data[1] << 16;
        *delay |= (uint32_t)data[2] << 8;
        *delay |= (uint32_t)data[3];
    }
    return (int)retVal;
}

int TouCAN_USB_CmdSetTransmitDelay(TouCAN_Handle_t handle, /*uint8_t channel,*/ uint32_t delay) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);
    UInt32 Transferred = 0U;

    SetupPacket.RequestType = USBREQ_HOST_TO_DEVICE | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    SetupPacket.Request = CMD_SET_CAN_INTERFACE_DELAY;
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 0U;  // FIXME: 4(?)

    data[0] = (uint8_t)((delay >> 24) & 0xFFU);
    data[1] = (uint8_t)((delay >> 16) & 0xFFU);
    data[2] = (uint8_t)((delay >> 8) & 0xFFU);
    data[3] = (uint8_t)(delay & 0xFFU);

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)4, (UInt32*)&Transferred);
    if (retVal < 0)
        return (int)retVal;

    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;

    if ((res != TOUCAN_RETVAL_OK) || (Transferred != 4U))
        return (int)TOUCAN_ERROR_OFFSET - (int)res;

    return (int)retVal;
}

int TouCAN_USB_CmdSetStandardFilter(TouCAN_Handle_t handle, TouCAN_FilterMode_t mode, uint32_t code, uint32_t mask) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);
    UInt32 Transferred = 0U;

    SetupPacket.RequestType = USBREQ_HOST_TO_DEVICE | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    switch (mode) {
    case FILTER_ACCEPT_ALL: SetupPacket.Request = CMD_FILTER_STD_ACCEPT_ALL; break;
    case FILTER_REJECT_ALL: SetupPacket.Request = CMD_FILTER_STD_REJECT_ALL; break;
    case FILTER_VALUE:      SetupPacket.Request = CMD_SET_FILTER_STD_LIST_MASK; break;
    default: 
        return (int)CANUSB_ERROR_ILLPARA;
    }
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 0U;  // FIXME: 8(?)

    data[0] = (uint8_t)((code >> 24) & 0xFFU);
    data[1] = (uint8_t)((code >> 16) & 0xFFU);
    data[2] = (uint8_t)((code >> 8) & 0xFFU);
    data[3] = (uint8_t) (code & 0xFFU);

    data[4] = (uint8_t)((mask >> 24) & 0xFFU);
    data[5] = (uint8_t)((mask >> 16) & 0xFFU);
    data[6] = (uint8_t)((mask >> 8) & 0xFFU);
    data[7] = (uint8_t) (mask & 0xFFU);

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)8, (UInt32*)&Transferred);
    if (retVal < 0)
        return (int)retVal;

    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;

    if ((res != TOUCAN_RETVAL_OK) || (Transferred != 8U))
        return (int)TOUCAN_ERROR_OFFSET - (int)res;

    return (int)retVal;
}

int TouCAN_USB_CmdSetExtendedFilter(TouCAN_Handle_t handle, TouCAN_FilterMode_t mode, uint32_t code, uint32_t mask) {
    CANUSB_SetupPacket_t SetupPacket;
    CANUSB_Return_t retVal;
    uint8_t res = 0U;
    uint8_t data[64];
    bzero(data, 64);
    UInt32 Transferred = 0U;

    SetupPacket.RequestType = USBREQ_HOST_TO_DEVICE | USBREQ_TYPE_CLASS | USBREQ_RECIPIENT_INTERFACE;
    switch (mode) {
    case FILTER_ACCEPT_ALL: SetupPacket.Request = CMD_FILTER_EXT_ACCEPT_ALL; break;
    case FILTER_REJECT_ALL: SetupPacket.Request = CMD_FILTER_EXT_REJECT_ALL; break;
    case FILTER_VALUE:      SetupPacket.Request = CMD_SET_FILTER_EXT_LIST_MASK; break;
    default:
        return (int)CANUSB_ERROR_ILLPARA;
    }
    SetupPacket.Value = 0U;
    SetupPacket.Index = 0U;
    SetupPacket.Length = 0U;  // FIXME: 8(?)

    data[0] = (uint8_t)((code >> 24) & 0xFFU);
    data[1] = (uint8_t)((code >> 16) & 0xFFU);
    data[2] = (uint8_t)((code >> 8) & 0xFFU);
    data[3] = (uint8_t)(code & 0xFFU);

    data[4] = (uint8_t)((mask >> 24) & 0xFFU);
    data[5] = (uint8_t)((mask >> 16) & 0xFFU);
    data[6] = (uint8_t)((mask >> 8) & 0xFFU);
    data[7] = (uint8_t)(mask & 0xFFU);

    retVal = CANUSB_DeviceRequest(handle, SetupPacket, (void*)data, (UInt16)8, (UInt32*)&Transferred);
    if (retVal < 0)
        return (int)retVal;

    retVal = TouCAN_USB_CmdGetLastErrorCode(handle, &res);
    if (retVal < 0)
        return (int)retVal;

    if ((res != TOUCAN_RETVAL_OK) || (Transferred != 8U))
        return (int)TOUCAN_ERROR_OFFSET - (int)res;

    return (int)retVal;
}
