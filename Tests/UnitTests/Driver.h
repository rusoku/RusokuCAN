//  SPDX-License-Identifier: BSD-2-Clause OR GPL-3.0-or-later
//
//  TouCAN - macOS User-Space Driver for Rusoku TouCAN USB Adapters
//
//  Copyright (c) 2021-2024 Uwe Vogt, UV Software, Berlin (info@mac-can.com)
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
#ifndef DRIVER_H_INCLUDED
#define DRIVER_H_INCLUDED

//  A Template for CAN API V3 Testing
//
//  (§1) include the header file of the CAN API V3 C++ class of the CAN driver
#include "TouCAN.h"

//  (§2) define macro CDriverCAN with the class name of the CAN driver
#define CDriverCAN  CTouCAN

//  (§3) define macro CAN_DEVICE1 and CAN_DEVICE2 with a valid CAN channel no.
#define CAN_DEVICE1  TOUCAN_USB_CHANNEL0
#define CAN_DEVICE2  TOUCAN_USB_CHANNEL1

//  ($4) define macros for unsupported features (at least the mandatory macros, cf. compiler warnings)
#define FEATURE_BITRATE_800K  FEATURE_UNSUPPORTED
#define FEATURE_BITRATE_SAM   FEATURE_UNSUPPORTED
#define FEATURE_BITRATE_SJA1000  FEATURE_UNSUPPORTED
#define FEATURE_BITRATE_FD_SAM   FEATURE_UNSUPPORTED
#define FEATURE_BITRATE_FD_SJA1000  FEATURE_UNSUPPORTED
#define FEATURE_WRITE_ACKNOWLEDGED  FEATURE_UNSUPPORTED
#define FEATURE_STATUS_BIT_QUE_OVR  FEATURE_SUPPORTED

//  (§5) define macros for CAN Classic bit-rate settings (at least BITRATE_1M, BITRATE_500K, BITRATE_250K, BITRATE_125K, BITRATE_100K, BITRATE_50K, BITRATE_20K, BITRATE_10K)
#define BITRATE_1M(x)    TOUCAN_BR_1M(x)  
#define BITRATE_500K(x)  TOUCAN_BR_500K(x)
#define BITRATE_250K(x)  TOUCAN_BR_250K(x)
#define BITRATE_125K(x)  TOUCAN_BR_125K(x)
#define BITRATE_100K(x)  TOUCAN_BR_100K(x)
#define BITRATE_50K(x)   TOUCAN_BR_50K(x) 
#define BITRATE_20K(x)   TOUCAN_BR_20K(x) 
#define BITRATE_10K(x)   TOUCAN_BR_10K(x) 

//  (§6) define macros for workarounds in CAN Classic operation mode (e.g. TC01_3_ISSUE)
#define TC03_7_ISSUE_TOUCAN_BITRATE_10K  ISSUE_TOUCAN_BITRATE_10K_WORKAROUND
#define TC03_17_ISSUE_TOUCAN_BITRATE_10K  ISSUE_TOUCAN_BITRATE_10K_WORKAROUND
#define TC03_18_ISSUE_TOUCAN_BITRATE_10K  ISSUE_TOUCAN_BITRATE_10K_WORKAROUND
#define TC03_19_ISSUE_TOUCAN_BITRATE_10K  ISSUE_TOUCAN_BITRATE_10K_WORKAROUND
#define TC03_21_ISSUE_TOUCAN_BITRATE_10K  ISSUE_TOUCAN_BITRATE_10K_WORKAROUND
#define TC03_22_ISSUE_TOUCAN_BITRATE_10K  ISSUE_TOUCAN_BITRATE_10K_WORKAROUND
#define TC11_10_ISSUE_TOUCAN_BITRATE_10K  ISSUE_TOUCAN_BITRATE_10K_WORKAROUND
#define ISSUE_TOUCAN_BITRATE_10K_WORKAROUND  1  // set to none-zero value to skip the hardware bug
//#define TC0x_y_ISSUE_  WORKAROUND_ENABLED

//  (§7) define macro CAN_FD_SUPPORTED if CAN FD operation mode is supported
#define CAN_FD_SUPPORTED  FEATURE_UNSUPPORTED

//  (§8) define macros for CAN FD bit-rate settings (at least BITRATE_FD_1M8M, BITRATE_FD_500K4M, BITRATE_FD_250K2M, BITRATE_FD_125K1M, BITRATE_FD_1M, BITRATE_FD_500K, BITRATE_FD_250K, BITRATE_FD_125K)
#define BITRATE_FD_1M(x)      TOUCAN_FD_BR_1M(x)    
#define BITRATE_FD_500K(x)    TOUCAN_FD_BR_500K(x)  
#define BITRATE_FD_250K(x)    TOUCAN_FD_BR_250K(x)  
#define BITRATE_FD_125K(x)    TOUCAN_FD_BR_125K(x)  
#define BITRATE_FD_1M8M(x)    TOUCAN_FD_BR_1M8M(x)  
#define BITRATE_FD_500K4M(x)  TOUCAN_FD_BR_500K4M(x)
#define BITRATE_FD_250K2M(x)  TOUCAN_FD_BR_250K2M(x)
#define BITRATE_FD_125K1M(x)  TOUCAN_FD_BR_125K1M(x)

#if (CAN_FD_SUPPORTED == FEATURE_SUPPORTED)
//  (§9) define macros for workarounds in CAN FD operation mode (e.g. TC01_3_ISSUE)
//#define TC0x_y_ISSUE_  WORKAROUND_ENABLED

#endif // CAN_FD_SUPPORTED
#endif // DRIVER_H_INCLUDED

// $Id: Template.h 1073 2022-07-16 13:06:44Z makemake $  Copyright (c) UV Software, Berlin //
