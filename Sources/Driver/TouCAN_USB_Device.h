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
#ifndef TOUCAN_USB_DEVICE_H_INCLUDED
#define TOUCAN_USB_DEVICE_H_INCLUDED

#include "TouCAN_USB_Common.h"

#include "MacCAN_IOUsbKit.h"
#include "MacCAN_MsgQueue.h"

typedef struct toucan_bitrate_t {       /* bit-rate settings: */
    uint16_t brp;                       /* - bit-rate prescaler */
    uint8_t tseg1;                      /* - TSEG1 segment */
    uint8_t tseg2;                      /* - TSEG2 segment */
    uint8_t sjw;                        /* - synchronization jump width */
} TouCAN_Bitrate_t;

typedef int32_t TouCAN_CanClock_t;      /* CAN clock (in [Hz]) */

typedef uint8_t TouCAN_OpMode_t;        /* operation mode (CAN API V1 compatible) */

typedef uint8_t TouCAN_Status_t;        /* bus status (CAN API V1 compatible) */

typedef struct receive_param_t {        /* additional reception data: */
    uint64_t startTime;                 /* - time synchronization */
    uint8_t busStatus;                  /* - bus status from devive */
    uint8_t rxErrors;                   /* - receive error counter */
    uint8_t txErrors;                   /* - transmit error counter */
    TouCAN_Status_t statusByte;         /* - status register */
    bool suppressXtd;                   /* - suppress extended CAN frames */
    bool suppressRtr;                   /* - suppress remote CAN frames */
    bool suppressSts;                   /* - suppress error frames */
} TouCAN_MsgParam_t;

typedef struct receive_data_t {         /* USB read pipe context: */
    CANQUE_MsgQueue_t msgQueue;         /* - message queue for received CAN frames */
    TouCAN_MsgParam_t msgParam;         /* - additional data on/for reception */
    uint64_t msgCounter;                /* - number of received CAN frames */
    uint64_t stsCounter;                /* - number of received status frames */
//    uint64_t errCounter;                /* - number of received error frames */
} TouCAN_ReceiveData_t;

typedef struct transmit_context_t_ {    /* USB write pipe context: */
#if (0)
    CANQUE_MsgQueue_t msgQueue;         /* - message queue for CAN frames to be sent */
    bool isBusy;                        /* - to indicate a transmission in progress */
#endif
    uint64_t msgCounter;                /* - number of written CAN frames */
    uint64_t errCounter;                /* - number of write pipe errors */
} TouCAN_TransmitData_t;

typedef struct device_info_t {          /* device information: */
    char name[TOUCAN_MAX_NAME_LENGTH+1];/* - short name (zero-terminated string) */
    int32_t type;                       /* - device type (not from Rusoku) */
    uint32_t hardware;                  /* - hardware version (0xggrrss00) */
    uint32_t firmware;                  /* - firmware version (0xggrrss00) */
    uint32_t bootloader;                /* - boot loader version (0xggrrss00) */
    uint32_t serialNo;                  /* - serial no. (32-bit) */
    uint32_t deviceId;                  /* - device id. (32-bit) */
    uint32_t vid_pid;                   /* - VID & PID (32-bit) */
} TouCAN_DeviceInfo_t;

typedef struct toucan_device_t_ {       /* TouCAN device: */
    uint16_t productId;                 /* - USB product id. */
    uint16_t releaseNo;                 /* - USB release no. */
    CANUSB_Handle_t handle;             /* - USB device hanlde */
    CANUSB_AsyncPipe_t recvPipe;        /* - USB receive pipe */
    TouCAN_ReceiveData_t recvData;      /* - CAN receive data (queue) */
    TouCAN_TransmitData_t sendData;     /* - CAN transmit data (no queue) */
    TouCAN_OpMode_t opCapa;             /* - CAN operation mode capability */
    TouCAN_OpMode_t opMode;             /* - CAN operation mode (demanded) */
    TouCAN_Bitrate_t bitRate;           /* - CAN bit-rate settings (demanded) */
    TouCAN_CanClock_t canClock;         /* - CAN clock (in [Hz]) = CPU frequency */
    TouCAN_DeviceInfo_t deviceInfo;     /* - device information (hw, sw, etc.) */
    char name[TOUCAN_MAX_NAME_LENGTH+1];     /* - device name (zero-terminated string) */
    char vendor[TOUCAN_MAX_STRING_LENGTH+1]; /* - vendor name (zero-terminated string) */
    char website[TOUCAN_MAX_STRING_LENGTH+1];/* - vendor website (zero-terminated string) */
    bool configured;                    /* - flag to indicate the structure's validity */
} TouCAN_Device_t;

#ifdef __cplusplus
extern "C" {
#endif

extern CANUSB_Return_t TouCAN_ProbeUsbDevice(CANUSB_Index_t channel, uint16_t *productId);
extern CANUSB_Return_t TouCAN_OpenUsbDevice(CANUSB_Index_t channel, TouCAN_Device_t *device);
extern CANUSB_Return_t TouCAN_CloseUsbDevice(TouCAN_Device_t *device);

extern CANUSB_Return_t TouCAN_StartReception(TouCAN_Device_t *device, CANUSB_AsyncPipeCbk_t callback);
extern CANUSB_Return_t TouCAN_AbortReception(TouCAN_Device_t *device);

#ifdef __cplusplus
}
#endif

#endif /* TOUCAN_USB_DEVICE_H_INCLUDED */
