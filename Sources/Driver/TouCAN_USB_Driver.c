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
#include "TouCAN_USB_Driver.h"
#include "TouCAN_USB_CmdMsg.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <assert.h>

#include "MacCAN_Debug.h"
#include <inttypes.h>

#define ISSUE_HAL_ERROR_CODES 1 // ICA for issue #32: Bus error states cannot be read from interface

#define TOUCAN_MSG_STD_FRAME  (UInt8)0x00
#define TOUCAN_MSG_XTD_FRAME  (UInt8)0x01
#define TOUCAN_MSG_RTR_FRAME  (UInt8)0x02
#define TOUCAN_MSG_STS_FRAME  (UInt8)0x04

#define TOUCAN_STS_OK         (UInt8)0x00
#define TOUCAN_STS_OVERRUN    (UInt8)0x01
#define TOUCAN_STS_BUSLIGHT   (UInt8)0x02
#define TOUCAN_STS_BUSHEAVY   (UInt8)0x03
#define TOUCAN_STS_BUSOFF     (UInt8)0x04
#define TOUCAN_STS_STUFF      (UInt8)0x20
#define TOUCAN_STS_FORM       (UInt8)0x21
#define TOUCAN_STS_ACK        (UInt8)0x23
#define TOUCAN_STS_BIT1       (UInt8)0x24
#define TOUCAN_STS_BIT0       (UInt8)0x25
#define TOUCAN_STS_CRC        (UInt8)0x27

static void ReceptionCallback(void *refCon, UInt8 *buffer, UInt32 length);
static int TouCAN_EncodeMessage(UInt8 *buffer, const TouCAN_CanMessage_t *message);
static int TouCAN_DecodeMessage(TouCAN_CanMessage_t *message, const UInt8 *buffer, TouCAN_MsgParam_t *param);
static int TouCAN_ResetDevice(CANUSB_Handle_t handle);

void TouCAN_USB_GetOperationCapability(TouCAN_OpMode_t *capability) {
    if (capability) {
        *capability |= TOUCAN_USB_MODE_FDOE ? CANMODE_FDOE : 0x00;
        *capability |= TOUCAN_USB_MODE_BRSE ? CANMODE_BRSE : 0x00;
        *capability |= TOUCAN_USB_MODE_NISO ? CANMODE_NISO : 0x00;
        *capability |= TOUCAN_USB_MODE_SHRD ? CANMODE_SHRD : 0x00;
        *capability |= TOUCAN_USB_MODE_NXTD ? CANMODE_NXTD : 0x00;
        *capability |= TOUCAN_USB_MODE_NRTR ? CANMODE_NRTR : 0x00;
        *capability |= TOUCAN_USB_MODE_ERR  ? CANMODE_ERR  : 0x00;
        *capability |= TOUCAN_USB_MODE_MON  ? CANMODE_MON  : 0x00;
    }
}

bool TouCAN_USB_ConfigureChannel(TouCAN_Device_t *device) {
    /* sanity check */
    if (!device)
        return false;
    if (device->configured)
        return false;
    if (device->handle == CANUSB_INVALID_HANDLE)
        return false;
    if (device->productId != TOUCAN_USB_PRODUCT_ID)
        return false;

    /* set CAN channel properties and defaults */
    strncpy(device->deviceInfo.name, TOUCAN_USB_DEVICE_NAME, TOUCAN_MAX_NAME_LENGTH);
    device->deviceInfo.type = TOUCAN_USB_DEVICE_TYPE;
    TouCAN_USB_GetOperationCapability(&device->opCapa);
    device->canClock = (TouCAN_CanClock_t)(TOUCAN_USB_CPU_FREQUENCY * 1000000U);
    device->bitRate.brp = TOUCAN_USB_BTR_BRP_250K;
    device->bitRate.tseg1 = TOUCAN_USB_BTR_TSEG1_250K;
    device->bitRate.tseg2 = TOUCAN_USB_BTR_TSEG2_250K;
    device->bitRate.sjw = TOUCAN_USB_BTR_SJW_250K;

    /* Gotcha! */
    device->configured = true;
    return device->configured;
}

CANUSB_Return_t TouCAN_USB_InitializeChannel(TouCAN_Device_t *device, TouCAN_OpMode_t mode) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;

    /* sanity check */
    if (!device)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* check requested operation mode */
    if ((mode & ~device->opCapa)) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): unsupported operation mode (%02x)\n", device->name, device->handle, (mode & ~device->opCapa));
        return CANUSB_ERROR_ILLPARA;  /* [2021-05-30]: cf. CAN API V3 function 'can_test' */
    }
    MACCAN_DEBUG_DRIVER("    Initializing %s driver...\n", device->name);
    /* reset device state and pending errors */
    retVal = TouCAN_ResetDevice(device->handle);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): device state could not be resetted (%i)\n", device->name, device->handle, retVal);
        goto end_init;
    }
    /* start the reception loop */
    retVal = TouCAN_StartReception(device, ReceptionCallback);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): reception loop could not be started (%i)\n", device->name, device->handle, retVal);
        goto end_init;
    }
    /* get device information (don't care about the result) */
    retVal = TouCAN_USB_CmdGetHardwareVersion(device->handle, &device->deviceInfo.hardware);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): hardware version could not be read (%i)\n", device->name, device->handle, retVal);
    }
    retVal = TouCAN_USB_CmdGetFirmwareVersion(device->handle, &device->deviceInfo.firmware);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): firmware version could not be read (%i)\n", device->name, device->handle, retVal);
    }
    retVal = TouCAN_USB_CmdGetBootloaderVersion(device->handle, &device->deviceInfo.bootloader);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): bootloader version could not be read (%i)\n", device->name, device->handle, retVal);
    }
    retVal = TouCAN_USB_CmdGetSerialNumber(device->handle, &device->deviceInfo.serialNo);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): serial no. could not be read (%i)\n", device->name, device->handle, retVal);
    }
    retVal = TouCAN_USB_CmdGetDeviceId(device->handle, &device->deviceInfo.deviceId);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): device id. could not be read (%i)\n", device->name, device->handle, retVal);
    }
    retVal = TouCAN_USB_CmdGetVidPid(device->handle, &device->deviceInfo.vid_pid);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): VID & PID could not be read (%i)\n", device->name, device->handle, retVal);
    }
    /* initialize with default bit-rate and mode flags */
    /* note: CAN API provides this at a later stage */
    retVal = TouCAN_USB_CmdInitInterface(device->handle, device->bitRate.brp, device->bitRate.tseg1,
                                         device->bitRate.tseg2, device->bitRate.sjw, 0x00000000U);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): device could not be initialized (%i)\n", device->name, device->handle, retVal);
        goto err_init;
    }
    /* store demanded CAN operation mode (and flags for the callback function) */
    device->recvData.msgParam.suppressXtd = (mode & CANMODE_NXTD) ? true : false;
    device->recvData.msgParam.suppressRtr = (mode & CANMODE_NRTR) ? true : false;
    device->recvData.msgParam.suppressSts = (mode & CANMODE_ERR) ? false : true;
    device->opMode = mode;
    retVal = CANUSB_SUCCESS;
end_init:
    return retVal;
err_init:
    (void)TouCAN_AbortReception(device);
    return retVal;
}

CANUSB_Return_t TouCAN_USB_TeardownChannel(TouCAN_Device_t *device) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;

    /* sanity check */
    if (!device)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    MACCAN_DEBUG_DRIVER("    Teardown %s driver...\n", device->name);
    /* enter RESET state (deinit) */
    retVal = TouCAN_ResetDevice(device->handle);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): device state could not be resetted (%i)\n", device->name, device->handle, retVal);
        //goto end_exit;
    }
//end_exit:
    /* stop the reception loop */
    retVal = TouCAN_AbortReception(device);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): reception loop could not be aborted (%i)\n", device->name, device->handle, retVal);
        return retVal;
    }
    /* now we are off :( */
    MACCAN_DEBUG_DRIVER("Statistical data:\n");
    MACCAN_DEBUG_DRIVER("%8"PRIu64" CAN frame(s) written to endpoint\n", device->sendData.msgCounter);
    MACCAN_DEBUG_DRIVER("%8"PRIu64" error(s) while writing to endpoint\n", device->sendData.errCounter);
    MACCAN_DEBUG_DRIVER("%8"PRIu64" CAN frame(s) received and enqueued\n", device->recvData.msgCounter);
    MACCAN_DEBUG_DRIVER("%8"PRIu64" status frame(s) received and enqueued\n", device->recvData.stsCounter);
    //MACCAN_DEBUG_DRIVER("%8"PRIu64" error frame(s) received  and enqueued\n", device->recvData.errCounter);
    MACCAN_DEBUG_DRIVER("%10.1f%% highest level of the receive queue\n", ((float)CANQUE_QueueHigh(device->recvData.msgQueue) * 100.0) \
                                                                       /  (float)CANQUE_QueueSize(device->recvData.msgQueue));
    MACCAN_DEBUG_DRIVER("%8"PRIu64" overrun event(s) of the receive queue\n", CANQUE_OverflowCounter(device->recvData.msgQueue));
    return retVal;
}

CANUSB_Return_t TouCAN_USB_SetBitrate(TouCAN_Device_t *device, const TouCAN_Bitrate_t *bitrate) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;
    UInt32 modeFlags = 0x00000000U;

    /* sanity check */
    if (!device || !bitrate)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* set mode flags depending on the operation mode */
    modeFlags |= (device->opMode & CANMODE_MON) ? TOUCAN_ENABLE_SILENT_MODE : 0x00000000U;
#ifndef ISSUE_HAL_ERROR_CODES
    modeFlags |= (device->opMode & CANMODE_ERR) ? TOUCAN_ENABLE_STATUS_MESSAGES : 0x00000000U;
#else
    // issue #32: Bus error states cannot be read from interface
    // >workaround: enable status messages to get the bus status
    modeFlags |= TOUCAN_ENABLE_STATUS_MESSAGES;
#endif
    /* reset device state and pending errors */
    retVal = TouCAN_ResetDevice(device->handle);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): device state could not be resetted (%i)\n", device->name, device->handle, retVal);
        goto end_set;
    }
    /* initialize with demanded bit-rate and mode-flags */
    retVal = TouCAN_USB_CmdInitInterface(device->handle, bitrate->brp, bitrate->tseg1,
                                         bitrate->tseg2, bitrate->sjw, modeFlags);
    if (retVal < 0) {
        MACCAN_DEBUG_ERROR("+++ %s (device #%u): device could not be re-initialized (%i)\n", device->name, device->handle, retVal);
        goto end_set;
    }
    /* store demanded CAN bit-rate settings */
    /* note: they cannot be read from device */
    memcpy(&device->bitRate, bitrate, sizeof(TouCAN_Bitrate_t));
end_set:
    return retVal;
}

CANUSB_Return_t TouCAN_USB_StartCan(TouCAN_Device_t *device) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;

    /* sanity check */
    if (!device)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* enter LISTENING state */
    retVal = TouCAN_USB_CmdStartInterface(device->handle);

    return retVal;
}

CANUSB_Return_t TouCAN_USB_StopCan(TouCAN_Device_t *device) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;

    /* sanity check */
    if (!device)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* enter READY state again */
    retVal = TouCAN_USB_CmdStopInterface(device->handle);

    return retVal;
}

CANUSB_Return_t TouCAN_USB_WriteMessage(TouCAN_Device_t *device, const TouCAN_CanMessage_t *message, uint16_t timeout) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;
    UInt8 buffer[TOUCAN_USB_TX_DATA_PIPE_SIZE];
    bzero(buffer, TOUCAN_USB_TX_DATA_PIPE_SIZE);

    /* sanity check */
    if (!device || !message)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* refuse certain CAN messages depending on the operation mode */
    if (message->xtd && (device->opMode & CANMODE_NXTD))
        return CANUSB_ERROR_ILLPARA;
    if (message->rtr && (device->opMode & CANMODE_NRTR))
        return CANUSB_ERROR_ILLPARA;
    if (message->sts)  /* note: error frames cannot be sent */
        return CANUSB_ERROR_ILLPARA;

    /* encode and send the message (w/o achknowledge) */
    int size = TouCAN_EncodeMessage(buffer, message);
    MACCAN_LOG_WRITE(buffer, (UInt32)size, ">");
    retVal = CANUSB_WritePipe(device->handle, TOUCAN_USB_TX_DATA_PIPE_REF, buffer, (UInt32)size, 0U);  // FIXME: time-out

    // TODO: implement a transmit queue to speed up sending
    (void) timeout;

    /* counting */
    if (retVal == CANUSB_SUCCESS)
        device->sendData.msgCounter++;
    else
        device->sendData.errCounter++;
    return retVal;
}

CANUSB_Return_t TouCAN_USB_ReadMessage(TouCAN_Device_t *device, TouCAN_CanMessage_t *message, uint16_t timeout) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;

    /* sanity check */
    if (!device || !message)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* read one CAN message from message queue, if any */
    retVal = CANQUE_Dequeue(device->recvData.msgQueue, (void*)message, timeout);

    return retVal;
}

CANUSB_Return_t TouCAN_USB_GetBusStatus(TouCAN_Device_t *device, TouCAN_Status_t *status) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;
    TouCAN_Status_t tmpStatus = 0x00U;
    UInt32 errorCode;

    /* sanity check */
    if (!device)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* get status from received status frames */
    tmpStatus = device->recvData.msgParam.statusByte;
    
    /* get (and clear) interface error code */
    retVal = TouCAN_USB_CmdGetInterfaceErrorCode(device->handle, &errorCode);
    if (retVal == CANUSB_SUCCESS) {
        // FIXME: error code is always 0 even when there are errors on the bus
#ifndef ISSUE_HAL_ERROR_CODES
        tmpStatus |= (errorCode & (HAL_CAN_ERROR_EWG |
                                   HAL_CAN_ERROR_EPV)) ? CANSTAT_EWRN : 0;
        tmpStatus |= (errorCode & (HAL_CAN_ERROR_BOF)) ? CANSTAT_BOFF : 0;
        tmpStatus |= (errorCode & (HAL_CAN_ERROR_STF |
                                   HAL_CAN_ERROR_FOR |
                                   HAL_CAN_ERROR_ACK |
                                   HAL_CAN_ERROR_BR  |
                                   HAL_CAN_ERROR_BD  |
                                   HAL_CAN_ERROR_CRC)) ? CANSTAT_BERR : 0;
        tmpStatus |= (errorCode & (HAL_CAN_ERROR_RX_FOV0 |
                                   HAL_CAN_ERROR_RX_FOV1)) ? CANSTAT_MSG_LST : 0;
        tmpStatus |= (errorCode & (HAL_CAN_ERROR_TX_ALST0 |
                                   HAL_CAN_ERROR_TX_TERR0 |
                                   HAL_CAN_ERROR_TX_ALST1 |
                                   HAL_CAN_ERROR_TX_TERR1 |
                                   HAL_CAN_ERROR_TX_ALST2 |
                                   HAL_CAN_ERROR_TX_TERR2)) ? CANSTAT_TX_BUSY :0;
#endif
    }
    (void)TouCAN_USB_CmdClearInterfaceErrorCode(device->handle);
    
    /* return updated status register */
    if (status)
        *status = tmpStatus;
    return retVal;
}

CANUSB_Return_t TouCAN_USB_SetStdFilter(TouCAN_Device_t *device, uint32_t code, uint32_t mask) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;

    /* sanity check */
    if (!device)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* set standard filter */
    retVal = TouCAN_USB_CmdSetStandardFilter(device->handle, FILTER_VALUE, code, mask);

    return retVal;
}

CANUSB_Return_t TouCAN_USB_SetXtdFilter(TouCAN_Device_t *device, uint32_t code, uint32_t mask) {
    CANUSB_Return_t retVal = CANUSB_ERROR_FATAL;

    /* sanity check */
    if (!device)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* set extended filter */
    retVal = TouCAN_USB_CmdSetExtendedFilter(device->handle, FILTER_VALUE, code, mask);

    return retVal;
}

CANUSB_Return_t TouCAN_USB_ResetFilters(TouCAN_Device_t *device) {
    CANUSB_Return_t retStd = CANUSB_ERROR_FATAL;
    CANUSB_Return_t retXtd = CANUSB_ERROR_FATAL;

    /* sanity check */
    if (!device)
        return CANUSB_ERROR_NULLPTR;
    if (!device->configured)
        return CANUSB_ERROR_NOTINIT;

    /* reset both filters to accept all identifiers */
    retStd = TouCAN_USB_CmdSetStandardFilter(device->handle, FILTER_ACCEPT_ALL, 0x0U, 0x0U);
    retXtd = TouCAN_USB_CmdSetExtendedFilter(device->handle, FILTER_ACCEPT_ALL, 0x0U, 0x0U);

    return (retStd != CANUSB_SUCCESS) ? retStd : retXtd;
}

bool TouCAN_USB_Index2Bitrate(int32_t index, TouCAN_Bitrate_t *bitrate) {
    bool retVal = true;
    
    assert(bitrate);
    
    switch (index) {
        case CANBTR_INDEX_1M:
            bitrate->brp = 5;
            bitrate->tseg1 = 7;
            bitrate->tseg2 = 2;
            bitrate->sjw = 2;
            break;
        case CANBTR_INDEX_500K:
            bitrate->brp = 5;
            bitrate->tseg1 = 14;
            bitrate->tseg2 = 5;
            bitrate->sjw = 4;
            break;
        case CANBTR_INDEX_250K:
            bitrate->brp = 10;
            bitrate->tseg1 = 14;
            bitrate->tseg2 = 5;
            bitrate->sjw = 4;
            break;
        case CANBTR_INDEX_125K:
            bitrate->brp = 20;
            bitrate->tseg1 = 14;
            bitrate->tseg2 = 5;
            bitrate->sjw = 4;
            break;
        case CANBTR_INDEX_100K:
            bitrate->brp = 25;
            bitrate->tseg1 = 14;
            bitrate->tseg2 = 5;
            bitrate->sjw = 4;
            break;
        case CANBTR_INDEX_50K:
            bitrate->brp = 50;
            bitrate->tseg1 = 14;
            bitrate->tseg2 = 5;
            bitrate->sjw = 4;
            break;
        case CANBTR_INDEX_20K:
            bitrate->brp = 125;
            bitrate->tseg1 = 14;
            bitrate->tseg2 = 5;
            bitrate->sjw = 4;
            break;
        case CANBTR_INDEX_10K:
            bitrate->brp = 250;
            bitrate->tseg1 = 14;
            bitrate->tseg2 = 5;
            bitrate->sjw = 4;
            break;
        default:
            retVal = false;
            break;
    }
    return retVal;
}

/* --- local functions ---
 */
static int TouCAN_ResetDevice(CANUSB_Handle_t handle) {
    int retVal;
    UInt8 state = 0;
    UInt32 error = 0;
    
    /* get device state */
    retVal = TouCAN_USB_CmdGetInterfaceState(handle, &state);
    if (retVal < 0)
        return retVal;
    /* state LISTENING ==> state READY */
    if (state == (UInt8)HAL_CAN_STATE_LISTENING) {
        retVal = TouCAN_USB_CmdStopInterface(handle);
        if (retVal < 0)
            return retVal;
        /* get new device state */
        retVal = TouCAN_USB_CmdGetInterfaceState(handle, &state);
        if (retVal < 0)
            return retVal;
    }
    /* state READY++ ==> state RESET */
    if (state != (UInt8)HAL_CAN_STATE_RESET) {
        retVal = TouCAN_USB_CmdDeinitInterface(handle);
        if (retVal < 0)
            return retVal;
    }
    /* get interface error code */
    retVal = TouCAN_USB_CmdGetInterfaceErrorCode(handle, &error);
    if (retVal < 0)
        return retVal;
    /* clear pending error(s) */
    if (error != (UInt32)HAL_CAN_ERROR_NONE) {
        /* Hibernation Issue
         * ~~~~~~~~~~~~~~~~~
         * When the Mac wakes up from hibernation then the interface is in state
         * RESET but the CAN controller is in an error state. We have first to
         * initialize the interface to reset the error and then to clear it.
         */
        retVal = TouCAN_USB_CmdInitInterface(handle, 10U, 14U, 5U, 4U, 0x00000000U);
        //fprintf(stderr, "!!! TouCAN_USB_CmdInitInterface returned %i\n", retVal);
        if (retVal < 0)
            return retVal;
        retVal = TouCAN_USB_CmdClearInterfaceErrorCode(handle);
        //fprintf(stderr, "!!! TouCAN_clear_interface_error_code returned %i\n", retVal);
        if (retVal < 0)
            return retVal;
        /* again get interface error code, but in any case return an error */
        retVal = TouCAN_USB_CmdGetInterfaceErrorCode(handle, &error);
        //fprintf(stderr, "!!! TouCAN_USB_CmdGetinterface_error_code returned %i\n", retVal);
        if (retVal < 0)
            return retVal;
        //fprintf(stderr, "    interface error code is 0x%x\n", error);
        if (error != (UInt32)HAL_CAN_ERROR_NONE)
            retVal = (int)TOUCAN_ERROR_OFFSET - (int)99;  // FATAL_ERROR;
        else
            retVal = (int)TOUCAN_ERROR_OFFSET - (int)TOUCAN_RETVAL_ERROR;
        /* finally tear off all the crap */
        (void)TouCAN_USB_CmdStopInterface(handle);
        (void)TouCAN_USB_CmdDeinitInterface(handle);
    }
    return retVal;
}

static void ReceptionCallback(void *refCon, UInt8 *buffer, UInt32 length) {
    TouCAN_ReceiveData_t *context = (TouCAN_ReceiveData_t *)refCon;
    TouCAN_CanMessage_t message;
    UInt32 index = 0;

    assert(refCon);
    assert(buffer);

    MACCAN_LOG_WRITE(buffer, length, "<");
    while (length >= TOUCAN_USB_RX_DATA_FRAME_SIZE) {
        bzero(&message, sizeof(TouCAN_CanMessage_t));
        (void)TouCAN_DecodeMessage(&message, &buffer[index], &context->msgParam);
        if ((message.xtd && context->msgParam.suppressXtd) ||
            (message.rtr && context->msgParam.suppressRtr) ||
            (message.sts && context->msgParam.suppressSts) ||
            (message.sts && !message.timestamp.tv_sec && !message.timestamp.tv_nsec)) {
            /* suppress certain CAN messages depending on the operation mode*/
        } else {
            if (CANQUE_Enqueue(context->msgQueue, &message) == CANUSB_SUCCESS) {
                if (!message.sts)
                    context->msgCounter++;
                else
                    context->stsCounter++;
            }
        }
        index += TOUCAN_USB_RX_DATA_FRAME_SIZE;
        length -= TOUCAN_USB_RX_DATA_FRAME_SIZE;
    }
}

static int TouCAN_EncodeMessage(UInt8 *buffer, const TouCAN_CanMessage_t *message) {
    int index = 0;
    
    assert(buffer);
    assert(message);

    /* byte 0: transmission flags */
    buffer[index++] = (UInt8)(message->xtd ? TOUCAN_MSG_XTD_FRAME : TOUCAN_MSG_STD_FRAME)
                    | (UInt8)(message->rtr ? TOUCAN_MSG_RTR_FRAME : TOUCAN_MSG_STD_FRAME);
    /* byte 1 - 4: identifier (big endian) */
    buffer[index++] = (UInt8)(message->id >> 24);
    buffer[index++] = (UInt8)(message->id >> 16);
    buffer[index++] = (UInt8)(message->id >> 8);
    buffer[index++] = (UInt8)message->id;
    /* byte 5: data length code (0..8) */
    buffer[index++] = (UInt8)message->dlc;
    /* byte 6 - 13: payload (8 bytes) */
    for (int i = 0; i < TOUCAN_USB_MAX_FRAME_LEN; i++)
        buffer[index++] = message->data[i];
    /* byte 14 - 17: timestamp? (big endian) */
    buffer[index++] = 0x00U;
    buffer[index++] = 0x00U;
    buffer[index++] = 0x00U;
    buffer[index++] = 0x00U;
    
    return index;
}

static int TouCAN_DecodeMessage(TouCAN_CanMessage_t *message, const UInt8 *buffer, TouCAN_MsgParam_t *param) {
    int index = 0;
    UInt64 hw_usec = 0U;
    UInt64 sw_usec = 0U;
    struct timespec ts;
    
    assert(buffer);
    assert(message);
    bzero(message, sizeof(TouCAN_CanMessage_t));
    
    /* get system time (in [usec]) */
    (void) clock_gettime(CLOCK_MONOTONIC, &ts);
    sw_usec = ((UInt64)ts.tv_sec * 1000000U)
            + ((UInt64)ts.tv_nsec / 1000U);
    
    /* byte 0: transmission flags */
    message->xtd = (buffer[index] & TOUCAN_MSG_XTD_FRAME) ? 1 : 0;
    message->rtr = (buffer[index] & TOUCAN_MSG_RTR_FRAME) ? 1 : 0;
    message->sts = (buffer[index++] & TOUCAN_MSG_STS_FRAME) ? 1 : 0;
    /* byte 1 - 4: identifier (big endian) */
    message->id |= (UInt32)buffer[index++] << 24;
    message->id |= (UInt32)buffer[index++] << 16;
    message->id |= (UInt32)buffer[index++] << 8;
    message->id |= (UInt32)buffer[index++];
    /* byte 5: data length code (0..8) */
    message->dlc = (UInt8)buffer[index++];
    /* byte 6 - 13: payload (8 bytes) */
    for (int i = 0; i < TOUCAN_USB_MAX_FRAME_LEN; i++)
        message->data[i] = buffer[index++];
    /* byte 14 - 17: timestamp (big endian) */
    hw_usec |= (UInt64)buffer[index++] << 24;
    hw_usec |= (UInt64)buffer[index++] << 16;
    hw_usec |= (UInt64)buffer[index++] << 8;
    hw_usec |= (UInt64)buffer[index++];
    
    /* overflow handling (2^32 - 1 = 1:11:34.962296) */
    if (param) {
        if (param->startTime == 0) {
            param->startTime = sw_usec - hw_usec;
        }
        UInt64 diff_time = sw_usec - param->startTime;
        hw_usec += ((diff_time / 0x100000000U) * 0x100000000U);  // FIXME: Don't understand!
    }
    /* timestamp as struct timespec (fraction in [nsec]) */
    message->timestamp.tv_sec = (time_t)(hw_usec / 1000000U);
    message->timestamp.tv_nsec =  (long)(hw_usec % 1000000U) * (long)1000;
    if (message->timestamp.tv_nsec >= (long)1000000000) {  // FIXME:  Condition 'message->timestamp.tv_nsec>=(long)1000000000' is always false
        message->timestamp.tv_nsec %= (long)1000000000;
        message->timestamp.tv_sec += (time_t)1;
    }
    /* status frame: data[0...] (bus status in byte 0) */
    if (message->sts && message->dlc && param) {
#if (1)
        switch (message->data[0]) {
        case TOUCAN_STS_OK:  // Normal condition.
            param->statusByte &= ~(CANSTAT_BOFF | CANSTAT_EWRN | CANSTAT_BERR);
            break;
        case TOUCAN_STS_OVERRUN:  // Overrun occured when sending data to CAN bus.
            // param->m_u8StatusByte |= ...;
            break;
        case TOUCAN_STS_BUSLIGHT:  // Error counter has reached 96.
            if (param->statusByte & CANSTAT_EWRN)
                param->statusByte &= ~CANSTAT_EWRN;
            break;
        case TOUCAN_STS_BUSHEAVY:  // Error counter has reached 128.
            param->statusByte |= CANSTAT_EWRN;
            break;
        case TOUCAN_STS_BUSOFF:  // Device is in BUSOFF. CANAL_STATUS_OK is
                                 // sent when returning to operational mode.
            param->statusByte |= CANSTAT_BOFF;
            break;
        case TOUCAN_STS_STUFF:  // Stuff Error.
        case TOUCAN_STS_FORM:   // Form Error.
        case TOUCAN_STS_ACK:    // Ack Error.
        case TOUCAN_STS_BIT1:   // Bit1 Error.
        case TOUCAN_STS_BIT0:   // Bit0 Error.
        case TOUCAN_STS_CRC:    // CRC Error.
            param->statusByte |= CANSTAT_BERR;
            break;
        }
#else
        // TODO: check above code with Windows driver!
#endif
        if (message->dlc >= 1)  // bus status from device
            param->busStatus = message->data[0];
        if (message->dlc >= 2)  // receive error counter (?)
            param->rxErrors = message->data[1];
        if (message->dlc >= 3)  // transmit error counter (?)
            param->txErrors = message->data[2];
        /* map status frame to CAN API error frame */
        message->id = 0U;
        message->xtd = 0;
        message->rtr = 0;
        message->sts = 1;
        message->dlc = 4U;
        message->data[0] = param->statusByte;
        message->data[1] = param->busStatus;
        message->data[2] = param->rxErrors;
        message->data[3] = param->txErrors;
    }
    return index;
}

