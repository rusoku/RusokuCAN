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
/** @addtogroup  can_btr
 *  @{
 */
#ifndef CANBTR_TOUCAN_H_INCLUDED
#define CANBTR_TOUCAN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/*  -----------  includes  ------------------------------------------------
 */

#include "CANBTR_Defaults.h"  /* CAN API default bit-rate defines */


/*  -----------  options  ------------------------------------------------
 */


/*  -----------  defines  ------------------------------------------------
 */

/** @name  Rusoku CAN Clocks
 *  @brief Default value for CAN 2.0 and CAN FD operation modes.
 *  @{ */
#define TOUCAN_CLOCK        50000000  /**< Rusoku CAN 2.0 clock (50 MHz) */
/** @} */

/** @name  Rusoku CAN 2.0 Bit-rate Settings
 *  @brief Default values (not compatible to CiA CANopen DS-301 spec.).
 *  @{ */
#define TOUCAN_BR_1M(x)     do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=5;   x.btr.nominal.tseg1=7;  x.btr.nominal.tseg2=2; x.btr.nominal.sjw=2; x.btr.nominal.sam=0; } while(0)
#define TOUCAN_BR_800K(x)   do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=0;   x.btr.nominal.tseg1=0;  x.btr.nominal.tseg2=0; x.btr.nominal.sjw=0; x.btr.nominal.sam=0; } while(0)
#define TOUCAN_BR_500K(x)   do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=5;   x.btr.nominal.tseg1=14; x.btr.nominal.tseg2=5; x.btr.nominal.sjw=4; x.btr.nominal.sam=0; } while(0)
#define TOUCAN_BR_250K(x)   do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=10;  x.btr.nominal.tseg1=14; x.btr.nominal.tseg2=5; x.btr.nominal.sjw=4; x.btr.nominal.sam=0; } while(0)
#define TOUCAN_BR_125K(x)   do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=20;  x.btr.nominal.tseg1=14; x.btr.nominal.tseg2=5; x.btr.nominal.sjw=4; x.btr.nominal.sam=0; } while(0)
#define TOUCAN_BR_100K(x)   do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=25;  x.btr.nominal.tseg1=14; x.btr.nominal.tseg2=5; x.btr.nominal.sjw=4; x.btr.nominal.sam=0; } while(0)
#define TOUCAN_BR_50K(x)    do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=50;  x.btr.nominal.tseg1=14; x.btr.nominal.tseg2=5; x.btr.nominal.sjw=4; x.btr.nominal.sam=0; } while(0)
#define TOUCAN_BR_20K(x)    do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=125; x.btr.nominal.tseg1=14; x.btr.nominal.tseg2=5; x.btr.nominal.sjw=4; x.btr.nominal.sam=0; } while(0)
#define TOUCAN_BR_10K(x)    do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=250; x.btr.nominal.tseg1=14; x.btr.nominal.tseg2=5; x.btr.nominal.sjw=4; x.btr.nominal.sam=0; } while(0)
#define TOUCAN_BR_5K(x)     do { x.btr.frequency=TOUCAN_CLOCK; x.btr.nominal.brp=0;   x.btr.nominal.tseg1=0;  x.btr.nominal.tseg2=0; x.btr.nominal.sjw=0; x.btr.nominal.sam=0; } while(0)
/** @} */

#if (OPTION_CAN_2_0_ONLY == 0)
/** @name  Rusoku CAN FD Bit-rate Settings w/o Bit-rate Switching
 *  @brief Default values for long frames only (0 to 64 bytes).
 *  @{ */
#define TOUCAN_FD_BR_1M(x)      do { x.btr.frequency=0;x.btr.nominal.brp=0;x.btr.nominal.tseg1=0;x.btr.nominal.tseg2=0;x.btr.nominal.sjw=0;x.btr.nominal.sam=0; } while(0)
#define TOUCAN_FD_BR_500K(x)    do { x.btr.frequency=0;x.btr.nominal.brp=0;x.btr.nominal.tseg1=0;x.btr.nominal.tseg2=0;x.btr.nominal.sjw=0;x.btr.nominal.sam=0; } while(0)
#define TOUCAN_FD_BR_250K(x)    do { x.btr.frequency=0;x.btr.nominal.brp=0;x.btr.nominal.tseg1=0;x.btr.nominal.tseg2=0;x.btr.nominal.sjw=0;x.btr.nominal.sam=0; } while(0)
#define TOUCAN_FD_BR_125K(x)    do { x.btr.frequency=0;x.btr.nominal.brp=0;x.btr.nominal.tseg1=0;x.btr.nominal.tseg2=0;x.btr.nominal.sjw=0;x.btr.nominal.sam=0; } while(0)
/** @} */

/** @name  Rusoku CAN FD Bit-rate Settings with Bit-rate Switching
 *  @brief Default values for long and fast frames only (up to 8 Mbps).
 *  @{ */
#define TOUCAN_FD_BR_1M8M(x)    do { x.btr.frequency=0;x.btr.nominal.brp=0;x.btr.nominal.tseg1=0;x.btr.nominal.tseg2=0;x.btr.nominal.sjw=0;x.btr.nominal.sam=0;x.btr.data.brp=0;x.btr.data.tseg1=0;x.btr.data.tseg2=0;x.btr.data.sjw=0; } while(0)
#define TOUCAN_FD_BR_500K4M(x)  do { x.btr.frequency=0;x.btr.nominal.brp=0;x.btr.nominal.tseg1=0;x.btr.nominal.tseg2=0;x.btr.nominal.sjw=0;x.btr.nominal.sam=0;x.btr.data.brp=0;x.btr.data.tseg1=0;x.btr.data.tseg2=0;x.btr.data.sjw=0; } while(0)
#define TOUCAN_FD_BR_250K2M(x)  do { x.btr.frequency=0;x.btr.nominal.brp=0;x.btr.nominal.tseg1=0;x.btr.nominal.tseg2=0;x.btr.nominal.sjw=0;x.btr.nominal.sam=0;x.btr.data.brp=0;x.btr.data.tseg1=0;x.btr.data.tseg2=0;x.btr.data.sjw=0; } while(0)
#define TOUCAN_FD_BR_125K1M(x)  do { x.btr.frequency=0;x.btr.nominal.brp=0;x.btr.nominal.tseg1=0;x.btr.nominal.tseg2=0;x.btr.nominal.sjw=0;x.btr.nominal.sam=0;x.btr.data.brp=0;x.btr.data.tseg1=0;x.btr.data.tseg2=0;x.btr.data.sjw=0; } while(0)
/** @} */
#endif
#ifdef __cplusplus
}
#endif
#endif /* CANBTR_TOUCAN_H_INCLUDED */
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de, Homepage: https://www.uv-software.de/
 */
