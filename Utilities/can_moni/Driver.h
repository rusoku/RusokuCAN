//  SPDX-License-Identifier: GPL-3.0-or-later
//
//  CAN Monitor for Rusoku TouCAN USB Adapters (CAN API V3)
//
//  Copyright (c) 2007,2020-2024 Uwe Vogt, UV Software, Berlin (info@mac-can.com)
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
#ifndef DRIVER_H_INCLUDED
#define DRIVER_H_INCLUDED

#include "TouCAN.h"

#if (OPTION_CAN_2_0_ONLY != 0)
#error Compilation with legacy CAN 2.0 frame format!
#else
#define CAN_FD_SUPPORTED  0  // set to non-zero once CAN FD is supported
#endif
#define BITRATE_800K_UNSUPPORTED  1  // set to zero if 800kbps is supported

#define MONITOR_INTEFACE  "Rusoku TouCAN USB Interfaces"
#define MONITOR_COPYRIGHT "2007,2020-2024 by Uwe Vogt, UV Software, Berlin"

#define BITRATE_1M(x)    TOUCAN_BR_1M(x)
#define BITRATE_500K(x)  TOUCAN_BR_500K(x)
#define BITRATE_250K(x)  TOUCAN_BR_250K(x)
#define BITRATE_125K(x)  TOUCAN_BR_125K(x)
#define BITRATE_100K(x)  TOUCAN_BR_100K(x)
#define BITRATE_50K(x)   TOUCAN_BR_50K(x)
#define BITRATE_20K(x)   TOUCAN_BR_20K(x)
#define BITRATE_10K(x)   TOUCAN_BR_10K(x)

typedef CTouCAN  CCanDriver;

#endif // DRIVER_H_INCLUDED
