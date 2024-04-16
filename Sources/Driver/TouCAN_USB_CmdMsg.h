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
#ifndef TOUCAN_USB_COMMANDS_H_INCLUDED
#define TOUCAN_USB_COMMANDS_H_INCLUDED

#include "TouCAN_USB_Common.h"

typedef int TouCAN_Handle_t;

#ifdef __cplusplus
extern "C" {
#endif

extern int TouCAN_USB_CmdInitInterface(TouCAN_Handle_t handle, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw, uint32_t flags);
extern int TouCAN_USB_CmdDeinitInterface(TouCAN_Handle_t handle);
extern int TouCAN_USB_CmdStartInterface(TouCAN_Handle_t handle);
extern int TouCAN_USB_CmdStopInterface(TouCAN_Handle_t handle);

extern int TouCAN_USB_CmdGetLastErrorCode(TouCAN_Handle_t handle, uint8_t *error);
extern int TouCAN_USB_CmdGetInterfaceErrorCode(TouCAN_Handle_t handle, uint32_t *error);
extern int TouCAN_USB_CmdClearInterfaceErrorCode(TouCAN_Handle_t handle);
extern int TouCAN_USB_CmdGetInterfaceState(TouCAN_Handle_t handle, uint8_t *state);

extern int TouCAN_USB_CmdGetHardwareVersion(TouCAN_Handle_t handle, uint32_t *value);
extern int TouCAN_USB_CmdGetFirmwareVersion(TouCAN_Handle_t handle, uint32_t *value);
extern int TouCAN_USB_CmdGetBootloaderVersion(TouCAN_Handle_t handle, uint32_t *value);
extern int TouCAN_USB_CmdGetSerialNumber(TouCAN_Handle_t handle, uint32_t *value);
extern int TouCAN_USB_CmdGetVidPid(TouCAN_Handle_t handle, uint32_t *value);
extern int TouCAN_USB_CmdGetDeviceId(TouCAN_Handle_t handle, uint32_t *value);
extern int TouCAN_USB_CmdGetVendorName(TouCAN_Handle_t handle, size_t size, char *name);
extern int TouCAN_USB_CmdGetTransmitDelay(TouCAN_Handle_t handle, uint32_t *delay);
extern int TouCAN_USB_CmdSetTransmitDelay(TouCAN_Handle_t handle, uint32_t delay);
extern int TouCAN_USB_CmdSetStandardFilter(TouCAN_Handle_t handle, TouCAN_FilterMode_t mode, uint32_t code, uint32_t mask);
extern int TouCAN_USB_CmdSetExtendedFilter(TouCAN_Handle_t handle, TouCAN_FilterMode_t mode, uint32_t code, uint32_t mask);

#ifdef __cplusplus
}
#endif
#endif  /* TOUCAN_USB_COMMANDS_H_INCLUDED */
