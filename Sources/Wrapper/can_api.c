/*  SPDX-License-Identifier: BSD-2-Clause OR GPL-3.0-or-later */
/*
 *  CAN Interface API, Version 3 (for Rusoku TouCAN Adapters)
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
#if defined(__APPLE__)
#define PLATFORM  "macOS"
#else
#error Platform not supported
#endif

/*  -----------  includes  -----------------------------------------------
 */
#include "can_defs.h"
#include "can_api.h"
#include "can_btr.h"

#include "TouCAN_Driver.h"
#include "Version.h"

#include <stdio.h>
#include <string.h>
#include <assert.h>

/*  -----------  options  ------------------------------------------------
 */
#if (OPTION_CAN_2_0_ONLY != 0)
#ifdef _MSC_VER
#pragma message ( "Compilation with legacy CAN 2.0 frame format!" )
#else
#warning Compilation with legacy CAN 2.0 frame format!
#endif
#endif
#if (OPTION_CANAPI_TOUCAN_DYLIB != 0)
__attribute__((constructor))
static void _initializer() {
    // default initializer
}
__attribute__((destructor))
static void _finalizer() {
    // default finalizer
}
#define EXPORT  __attribute__((visibility("default")))
#else
#define EXPORT
#endif

/*  -----------  defines  ------------------------------------------------
 */
#ifndef CAN_MAX_HANDLES
#define CAN_MAX_HANDLES         (8)     // maximum number of open handles
#endif
#define INVALID_HANDLE          (-1)
#define IS_HANDLE_VALID(hnd)    ((0 <= (hnd)) && ((hnd) < CAN_MAX_HANDLES))
#ifndef DLC2LEN
#define DLC2LEN(x)              dlc_table[((x) < 16) ? (x) : 15]
#endif
#ifndef LEN2DLC
#define LEN2DLC(x)              ((x) > 48) ? 0xF : \
                                ((x) > 32) ? 0xE : \
                                ((x) > 24) ? 0xD : \
                                ((x) > 20) ? 0xC : \
                                ((x) > 16) ? 0xB : \
                                ((x) > 12) ? 0xA : \
                                ((x) > 8) ?  0x9 : (x)
#endif
#define FILTER_STD_CODE         (uint32_t)(0x000)
#define FILTER_STD_MASK         (uint32_t)(0x000)
#define FILTER_XTD_CODE         (uint32_t)(0x00000000)
#define FILTER_XTD_MASK         (uint32_t)(0x00000000)

#define LIB_ID                  TOUCAN_LIB_ID
#define LIB_DLLNAME             TOUCAN_LIB_WRAPPER
#define DEV_VENDOR              TOUCAN_LIB_VENDOR
#define DEV_DLLNAME             TOUCAN_LIB_CANLIB
#define NUM_CHANNELS            TOUCAN_BOARDS

/*  -----------  types  --------------------------------------------------
 */
typedef struct {                        // message filtering:
    struct {                            //   acceptance filter:
        uint32_t code;                  //     acceptance code
        uint32_t mask;                  //     acceptance mask
    } std, xtd;                         //   for standard and extended frames
    bool reset;                         //   flag to reset std. and xtd. filter
}   can_filter_t;

typedef struct {                        // frame counters:
    uint64_t tx;                        //   number of transmitted CAN frames
    uint64_t rx;                        //   number of received CAN frames
    uint64_t err;                       //   number of receiced error frames
}   can_counter_t;

typedef struct {                        // TouCAN interface:
    TouCAN_Device_t device;             //   USB device descriptor
    can_mode_t mode;                    //   CAN operation mode
    can_filter_t filter;                //   message filter settings
    can_status_t status;                //   8-bit status register
    can_counter_t counters;             //   statistical counters
}   can_interface_t;

/*  -----------  prototypes  ---------------------------------------------
 */
static void var_init(void);             // initialize variables

static int set_filter(int handle, uint64_t filter, bool xtd);
static int reset_filter(int handle);

static int lib_parameter(uint16_t param, void *value, size_t nbyte);
static int drv_parameter(int handle, uint16_t param, void *value, size_t nbyte);

/*  -----------  variables  ----------------------------------------------
 */
static const char version[] = "CAN API V3 for Rusoku TouCAN USB Interfaces, Version " VERSION_STRING;

EXPORT
can_board_t can_boards[NUM_CHANNELS+1] = {  // list of supported CAN channels
    {TOUCAN_USB_CHANNEL0, (char *)"TouCAN-USB1"},
    {TOUCAN_USB_CHANNEL1, (char *)"TouCAN-USB2"},
    {TOUCAN_USB_CHANNEL2, (char *)"TouCAN-USB3"},
    {TOUCAN_USB_CHANNEL3, (char *)"TouCAN-USB4"},
    {TOUCAN_USB_CHANNEL4, (char *)"TouCAN-USB5"},
    {TOUCAN_USB_CHANNEL5, (char *)"TouCAN-USB6"},
    {TOUCAN_USB_CHANNEL6, (char *)"TouCAN-USB7"},
    {TOUCAN_USB_CHANNEL7, (char *)"TouCAN-USB8"},
    {EOF, NULL}
};
//static const uint8_t dlc_table[16] = {  // DLC to length
//    0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
//};
static can_interface_t can[CAN_MAX_HANDLES];  // interface handles
static int init = 0;                    // initialization flag

/*  -----------  functions  ----------------------------------------------
 */
EXPORT
int can_test(int32_t channel, uint8_t mode, const void *param, int *result)
{
    int rc = CANERR_FATAL;              // return value

    if (result)                         // the last resort
        *result = CANBRD_NOT_TESTABLE;
    if (!init) {                        // when not initialized:
        var_init();                     //   initialize the variables
        // initialize the driver (MacCAN-Core driver)
        if ((rc = TouCAN_InitializeDriver()) != CANERR_NOERROR)
            return rc;
        init = 1;
    }
    if (!init)                          // must be initialized
        return CANERR_FATAL;
    if (!IS_HANDLE_VALID(channel))      // must be a valid channel!
#ifndef OPTION_CANAPI_RETVALS
        return CANERR_HANDLE;
#else
        // note: can_test shall return vendor-specific error code or
        //       CANERR_NOTINIT in this case
        return CANERR_NOTINIT;
#endif
    // probe the CAN channel and check it selected operation mode is supported by the CAN controller
    rc = TouCAN_ProbeChannel(channel, mode, result);
    // note: 1. parameter 'result' is checked for NULL pointer by the called function
    //       2. error code CANERR_ILLPARA is return in case the operation mode is not supported
    (void) param;
    return (int)rc;
}

EXPORT
int can_init(int32_t channel, uint8_t mode, const void *param)
{
    int rc = CANERR_FATAL;              // return value

    if (!init) {                        // when not initialized:
        var_init();                     //   initialize the variables
        // initialize the driver (MacCAN-Core driver)
        if ((rc = TouCAN_InitializeDriver()) != CANERR_NOERROR)
            return rc;
        init = 1;
    }
    if (!init)                          // must be initialized
        return CANERR_FATAL;
    if (!IS_HANDLE_VALID(channel))      // must be a valid channel!
#ifndef OPTION_CANAPI_RETVALS
        return CANERR_HANDLE;
#else
        // note: can_init shall return vendor-specific error code or
        //       CANERR_NOTINIT in this case
        return CANERR_NOTINIT;
#endif
    // initialize CAN channel with selected operation mode
    if ((rc = TouCAN_InitializeChannel(channel, mode, &can[channel].device)) < CANERR_NOERROR)
        return rc;
    can[channel].mode.byte = mode;      // store selected operation mode
    can[channel].status.byte = CANSTAT_RESET; // CAN not started yet
    (void)param;
    return (int)channel;                // return the handle (channel)
}

EXPORT
int can_exit(int handle)
{
    int rc, i;                          // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (handle != CANEXIT_ALL) {
        if (!IS_HANDLE_VALID(handle))   // must be a valid handle
            return CANERR_HANDLE;
        if (!can[handle].device.configured) // must be an opened handle
            return CANERR_HANDLE;
        /*if (!can[handle].status.can_stopped) // go to CAN INIT mode (bus off)*/
            (void)TouCAN_StopCan(&can[handle].device);
        if ((rc = TouCAN_TeardownChannel(&can[handle].device)) < CANERR_NOERROR)
            return rc;
        can[handle].status.byte |= CANSTAT_RESET; // CAN controller in INIT state
        can[handle].device.configured = false;    // handle can be used again
    }
    else {
        for (i = 0; i < CAN_MAX_HANDLES; i++) {
            if (can[i].device.configured) // must be an opened handle
            {
                /*if (!can[handle].status.can_stopped) // go to CAN INIT mode (bus off)*/
                    (void)TouCAN_StopCan(&can[i].device);
                (void)TouCAN_TeardownChannel(&can[i].device);
                can[i].status.byte |= CANSTAT_RESET; // CAN controller in INIT state
                can[i].device.configured = false;    // handle can be used again
            }
        }
    }
    // teardown the driver when all interfaces released
    for (i = 0; i < CAN_MAX_HANDLES; i++) {
        if (can[i].device.configured)
            break;
    }
    if (i == CAN_MAX_HANDLES) {
        (void)TouCAN_TeardownDriver();
        init = 0;
    }
    return CANERR_NOERROR;
}

EXPORT
int can_kill(int handle)
{
    int rc, i;                          // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (handle != CANEXIT_ALL) {
        if (!IS_HANDLE_VALID(handle))   // must be a valid handle
            return CANERR_HANDLE;
        if (!can[handle].device.configured) // must be an opened handle
            return CANERR_HANDLE;
        if ((rc = TouCAN_SignalChannel(&can[handle].device)) < CANERR_NOERROR)
            return rc;
    }
    else {
        for (i = 0; i < CAN_MAX_HANDLES; i++) {
            if (can[i].device.configured) // must be an opened handle
                (void)TouCAN_SignalChannel(&can[i].device);
        }
    }
    return CANERR_NOERROR;
}

EXPORT
int can_start(int handle, const can_bitrate_t *bitrate)
{
    int rc = CANERR_FATAL;              // return value

    TouCAN_Bitrate_t touBitrate;        // TouCAN bit-rate settings
    memset(&touBitrate, 0, sizeof(TouCAN_Bitrate_t));

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!can[handle].device.configured) // must be an opened handle
        return CANERR_HANDLE;
    if (bitrate == NULL)                // check for null-pointer
        return CANERR_NULLPTR;
    if (!can[handle].status.can_stopped) // must be stopped
        return CANERR_ONLINE;

    // check bit-rate settings (possibly after conversion from index)
    if (bitrate->index <= 0) {
        // note: we have vendor-specific bit-timing (clock domain is 50MHz)
        //       the method from the base class uses the SJA1000 clock domain
        if (!TouCAN_Index2Bitrate(&can[handle].device, bitrate->index, &touBitrate))
            return CANERR_BAUDRATE;
    } else {
#if (OPTION_CAN_2_0_ONLY == 0)
        bool fdoe = can[handle].mode.fdoe ? true : false;
        bool brse = can[handle].mode.brse ? true : false;
#else
        bool fdoe = false;
        bool brse = false;
#endif
        // note: only one valid CAN clock provided by the TouCAN device
        if (bitrate->btr.frequency != (int32_t)can[handle].device.canClock)
            return CANERR_BAUDRATE;
        // note: bit-rate settings are checked by the conversion function
        if (btr_check_bitrate(bitrate, fdoe, brse) < CANERR_NOERROR)
            return CANERR_BAUDRATE;
        touBitrate.brp   = bitrate->btr.nominal.brp;
        touBitrate.tseg1 = bitrate->btr.nominal.tseg1;
        touBitrate.tseg2 = bitrate->btr.nominal.tseg2;
        touBitrate.sjw   = bitrate->btr.nominal.sjw;
    }
    // set bit-rate (with respect of the selected operation mode)
    if ((rc = TouCAN_SetBitrate(&can[handle].device, &touBitrate)) < CANERR_NOERROR)
        return (rc != CANUSB_ERROR_ILLPARA) ? rc : CANERR_BAUDRATE;
    // clear status, counters, and the receive queue
    can[handle].status.byte = CANSTAT_RESET;
    can[handle].counters.tx = 0U;
    can[handle].counters.rx = 0U;
    can[handle].counters.err = 0U;
    (void)CANQUE_Reset(can[handle].device.recvData.msgQueue);

    // start the CAN controller with the selected operation mode
    if ((rc = TouCAN_StartCan(&can[handle].device)) < CANERR_NOERROR)
        return rc;
    // set acceptance filter (can only be set when the CAN controller has been started)
    if (!can[handle].filter.reset) {
        if ((rc = TouCAN_SetStdFilter(&can[handle].device,
                                       can[handle].filter.std.code,
                                       can[handle].filter.std.mask)) == CANERR_NOERROR)
             rc = TouCAN_SetXtdFilter(&can[handle].device,
                                       can[handle].filter.xtd.code,
                                       can[handle].filter.xtd.mask);
    } else {
        rc = TouCAN_ResetFilters(&can[handle].device);
    }
    if (rc != CANERR_NOERROR)           // on error: stop the CAN controller
        (void)TouCAN_StopCan(&can[handle].device);
    // set the status bit accordingly
    can[handle].status.can_stopped = (rc == CANUSB_SUCCESS) ? 0 : 1;
    return rc;
}

EXPORT
int can_reset(int handle)
{
    int rc = CANERR_FATAL;              // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!can[handle].device.configured) // must be an opened handle
        return CANERR_HANDLE;
    if (can[handle].status.can_stopped) // must be running
#ifndef OPTION_CANAPI_RETVALS
        return CANERR_OFFLINE;
#else
        // note: can_reset shall return CANERR_NOERROR even if
        //       the CAN controller has not been started
        return CANERR_NOERROR;
#endif
    // stop the CAN controller (INIT state)
    rc = TouCAN_StopCan(&can[handle].device);
    can[handle].status.can_stopped = (rc == CANUSB_SUCCESS) ? 1 : 0;
    return rc;
}

EXPORT
int can_write(int handle, const can_message_t *message, uint16_t timeout)
{
    int rc = CANERR_FATAL;              // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!can[handle].device.configured) // must be an opened handle
        return CANERR_HANDLE;
    if (message == NULL)                // check for null-pointer
        return CANERR_NULLPTR;
    if (can[handle].status.can_stopped) // must be running
        return CANERR_OFFLINE;

    if (message->id > (uint32_t)(message->xtd ? CAN_MAX_XTD_ID : CAN_MAX_STD_ID))
        return CANERR_ILLPARA;          // invalid identifier
    if (message->xtd && can[handle].mode.nxtd)
        return CANERR_ILLPARA;          // suppress extended frames
    if (message->rtr && can[handle].mode.nrtr)
        return CANERR_ILLPARA;          // suppress remote frames
#if (OPTION_CAN_2_0_ONLY == 0)
    if (message->fdf && !can[handle].mode.fdoe)
        return CANERR_ILLPARA;          // long frames only with CAN FD
    if (message->brs && !can[handle].mode.brse)
        return CANERR_ILLPARA;          // fast frames only with CAN FD
    if (message->brs && !message->fdf)
        return CANERR_ILLPARA;          // bit-rate switching only with CAN FD
#endif
    if (message->sts)
        return CANERR_ILLPARA;          // error frames cannot be sent

    if (message->dlc > CAN_MAX_LEN)     //   data length 0 .. 8!
        return CANERR_ILLPARA;

    // transmit the given CAN message (w/ or w/o acknowledgment)
    rc = TouCAN_WriteMessage(&can[handle].device, message, timeout);
    can[handle].status.transmitter_busy = (rc != CANUSB_SUCCESS) ? 1 : 0;
    can[handle].counters.tx += (rc == CANUSB_SUCCESS) ? 1U : 0U;
    return rc;
}

EXPORT
int can_read(int handle, can_message_t *message, uint16_t timeout)
{
    int rc = CANERR_FATAL;              // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!can[handle].device.configured) // must be an opened handle
        return CANERR_HANDLE;
    if (message == NULL)                // check for null-pointer
        return CANERR_NULLPTR;
    if (can[handle].status.can_stopped) // must be running
        return CANERR_OFFLINE;

    // read one CAN message from the message queue, if any
    rc = TouCAN_ReadMessage(&can[handle].device, message, timeout);
    can[handle].status.receiver_empty = (rc != CANUSB_SUCCESS) ? 1 : 0;
    can[handle].status.queue_overrun = CANQUE_OverflowFlag(can[handle].device.recvData.msgQueue) ? 1 : 0;
    can[handle].counters.rx += ((rc == CANUSB_SUCCESS) && !message->sts) ? 1U : 0U;
    can[handle].counters.err += ((rc == CANUSB_SUCCESS) && message->sts) ? 1U : 0U;
    return rc;
}

EXPORT
int can_status(int handle, uint8_t *status)
{
    int rc = CANERR_FATAL;              // return value

    TouCAN_Status_t busStatus = 0x00U;  // CAN bus status

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!can[handle].device.configured) // must be an opened handle
        return CANERR_HANDLE;

    // get status-register from device (CAN API V1 compatible)
    if ((rc = TouCAN_GetBusStatus(&can[handle].device, &busStatus)) == CANUSB_SUCCESS) {
        can[handle].status.byte &= ~(CANSTAT_BOFF | CANSTAT_EWRN | CANSTAT_BERR);
        can[handle].status.byte |= ((CANSTAT_BOFF | CANSTAT_EWRN | CANSTAT_BERR) & busStatus);
        // note: only bit 6 to 4 are set or cleared by the device
    }
    if (status)                         // status-register
      *status = can[handle].status.byte;

    return rc;
}

EXPORT
int can_busload(int handle, uint8_t *load, uint8_t *status)
{
    int rc = CANERR_FATAL;              // return value

    uint8_t busLoad = 0U;

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!can[handle].device.configured) // must be an opened handle
        return CANERR_HANDLE;

#if (0)
    // get bus load from device (0..10000 ==> 0%..100%)
    if ((rc = TouCAN_GetBusLoad(&can[handle].device, &busLoad)) == CANUSB_SUCCESS) {
        // get status-register from device
        rc = can_status(handle, status);
    }
    else
        busLoad = 0U;
    // TODO: realize bus-load measurement
#else
    rc = can_status(handle, status);
#endif
    if (load)
        *load = (uint8_t)((busLoad + 50U) / 100U);
    return rc;
}

EXPORT
int can_bitrate(int handle, can_bitrate_t *bitrate, can_speed_t *speed)
{
    int rc = CANERR_FATAL;              // return value

    can_bitrate_t tmpBitrate;           // bit-rate settings
    can_speed_t tmpSpeed;               // transmission speed

    memset(&tmpBitrate, 0, sizeof(can_bitrate_t));
    memset(&tmpSpeed, 0, sizeof(can_speed_t));

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!can[handle].device.configured) // must be an opened handle
        return CANERR_HANDLE;

    // get bit-rate settings from device
    tmpBitrate.btr.frequency = can[handle].device.canClock;
    tmpBitrate.btr.nominal.brp = can[handle].device.bitRate.brp;
    tmpBitrate.btr.nominal.tseg1 = can[handle].device.bitRate.tseg1;
    tmpBitrate.btr.nominal.tseg2 = can[handle].device.bitRate.tseg2;
    tmpBitrate.btr.nominal.sjw = can[handle].device.bitRate.sjw;
	tmpBitrate.btr.nominal.sam = 0U;    // note: SAM not used by TouCAN
    // calculate bus speed from bit-rate settings
    if ((rc = btr_bitrate2speed(&tmpBitrate, &tmpSpeed)) < CANERR_NOERROR)
        return CANERR_BAUDRATE;
    if (bitrate)
        memcpy(bitrate, &tmpBitrate, sizeof(can_bitrate_t));
    if (speed)
        memcpy(speed, &tmpSpeed, sizeof(can_speed_t));
#ifdef OPTION_CANAPI_RETVALS
    // note: can_bitrate shall return CANERR_OFFLINE if
    //       the CAN controller has not been started
    if (can[handle].status.can_stopped)
        rc = CANERR_OFFLINE;
#endif
    return rc;
}

EXPORT
int can_property(int handle, uint16_t param, void *value, uint32_t nbyte)
{
    if (!init || !IS_HANDLE_VALID(handle)) {
        // note: library properties can be queried w/o a handle
        return lib_parameter(param, value, (size_t)nbyte);
    }
    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!can[handle].device.configured) // must be an opened handle
        return CANERR_HANDLE;
    // note: device properties must be queried with a valid handle
    return drv_parameter(handle, param, value, (size_t)nbyte);
}

EXPORT
char *can_hardware(int handle)
{
    static char string[CANPROP_MAX_BUFFER_SIZE] = "(unknown)";

    if (!init)                          // must be initialized
        return NULL;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return NULL;
    if (!can[handle].device.configured) // must be an opened handle
        return NULL;

    // return hardware version (zero-terminated string)
    uint8_t major = (uint8_t)(can[handle].device.deviceInfo.hardware >> 24);
    uint8_t minor = (uint8_t)(can[handle].device.deviceInfo.hardware >> 16);
    uint8_t patch = (uint8_t)(can[handle].device.deviceInfo.hardware >> 8);
    sprintf(string, "%s, hardware %u.%u.%u (s/n %08x)", can[handle].device.deviceInfo.name,
            major, minor, patch, can[handle].device.deviceInfo.serialNo);
    return string;
}

EXPORT
char *can_firmware(int handle)
{
    static char string[CANPROP_MAX_BUFFER_SIZE] = "(unknown)";

    if (!init)                          // must be initialized
        return NULL;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return NULL;
    if (!can[handle].device.configured) // must be an opened handle
        return NULL;

    // return firmware version (zero-terminated string)
    uint8_t major = (uint8_t)(can[handle].device.deviceInfo.firmware >> 24);
    uint8_t minor = (uint8_t)(can[handle].device.deviceInfo.firmware >> 16);
    uint8_t patch = (uint8_t)(can[handle].device.deviceInfo.firmware >> 8);
    sprintf(string, "%s, firmware %u.%u.%u (%s)", can[handle].device.deviceInfo.name,
            major, minor, patch, can[handle].device.website);
    return string;
}

/*  -----------  local functions  ----------------------------------------
 */
static void var_init(void)
{
    int i;

    for (i = 0; i < CAN_MAX_HANDLES; i++) {
        memset(&can[i], 0, sizeof(can_interface_t));
        can[i].device.configured = false;
        can[i].mode.byte = CANMODE_DEFAULT;
        can[i].status.byte = CANSTAT_RESET;
        can[i].filter.reset = true;
        can[i].filter.std.code = FILTER_STD_CODE;
        can[i].filter.std.mask = FILTER_STD_MASK;
        can[i].filter.xtd.code = FILTER_XTD_CODE;
        can[i].filter.xtd.mask = FILTER_XTD_MASK;
        //can[i].error.lec = 0x00u;
        //can[i].error.rx_err = 0u;
        //can[i].error.tx_err = 0u;
        can[i].counters.tx = 0ull;
        can[i].counters.rx = 0ull;
        can[i].counters.err = 0ull;
    }
}

static int set_filter(int handle, uint64_t filter, bool xtd)
{
    assert(IS_HANDLE_VALID(handle));    // just to make sure

    /* the acceptance code and mask are coded together in a 64-bit value, each of them using 4 bytes
     * the acceptance code is in the most significant bytes, the mask in the least significant bytes
     */
    unsigned int code = (unsigned int)((filter >> 32)) & (unsigned int)(xtd ? CAN_MAX_XTD_ID : CAN_MAX_STD_ID);
    unsigned int mask = (unsigned int)((filter >>  0)) & (unsigned int)(xtd ? CAN_MAX_XTD_ID : CAN_MAX_STD_ID);

    /* store the acceptance filter values:
     * a) they can only be set when the controller has been started
     * b) they cannot be read directly from the controller
     */
    if (!xtd) {
        can[handle].filter.std.code = code;
        can[handle].filter.std.mask = mask;
    } else {
        can[handle].filter.xtd.code = code;
        can[handle].filter.xtd.mask = mask;
    }
    can[handle].filter.reset = false;
    
    return CANERR_NOERROR;
}

static int reset_filter(int handle)
{
    assert(IS_HANDLE_VALID(handle));    // just to make sure

    /* reset the acceptance filter for standard and extended identifier */
    can[handle].filter.reset = true;
    can[handle].filter.std.code = FILTER_STD_CODE;
    can[handle].filter.std.mask = FILTER_STD_MASK;
    can[handle].filter.xtd.code = FILTER_XTD_CODE;
    can[handle].filter.xtd.mask = FILTER_XTD_MASK;

    return CANERR_NOERROR;
}

/*  - - - - - -  CAN API V3 properties  - - - - - - - - - - - - - - - - -
 */
static int lib_parameter(uint16_t param, void *value, size_t nbyte)
{
    int rc = CANERR_ILLPARA;            // suppose an invalid parameter
    static int idx_board = EOF;         // actual index in the interface list

    if (value == NULL) {                // check for null-pointer
        if ((param != CANPROP_SET_FIRST_CHANNEL) &&
            (param != CANPROP_SET_NEXT_CHANNEL) &&
            (param != CANPROP_SET_FILTER_RESET))
            return CANERR_NULLPTR;
    }
    /* CAN library properties */
    switch (param) {
    case CANPROP_GET_SPEC:              // version of the wrapper specification (uint16_t)
        if (nbyte >= sizeof(uint16_t)) {
            *(uint16_t*)value = (uint16_t)CAN_API_SPEC;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_VERSION:           // version number of the library (uint16_t)
        if (nbyte >= sizeof(uint16_t)) {
            *(uint16_t*)value = ((uint16_t)VERSION_MAJOR << 8)
                              | ((uint16_t)VERSION_MINOR & 0xFu);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_PATCH_NO:          // patch number of the library (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            *(uint8_t*)value = (uint8_t)VERSION_PATCH;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_BUILD_NO:          // build number of the library (uint32_t)
        if (nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)VERSION_BUILD;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_LIBRARY_ID:        // library id of the library (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            *(int32_t*)value = (int32_t)LIB_ID;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_LIBRARY_VENDOR:    // vendor name of the library (char[256])
        if ((nbyte > strlen(CAN_API_VENDOR)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, CAN_API_VENDOR);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_LIBRARY_DLLNAME:   // file name of the library (char[256])
        if ((nbyte > strlen(LIB_DLLNAME)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, LIB_DLLNAME);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_VENDOR:     // vendor name of the CAN interface (char[256])
        if ((nbyte > strlen(DEV_VENDOR)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, DEV_VENDOR);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_DLLNAME:    // file name of the CAN interface DLL (char[256])
        if ((nbyte > strlen(DEV_DLLNAME)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, DEV_DLLNAME);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_SET_FIRST_CHANNEL:     // set index to the first entry in the interface list (NULL)
        idx_board = 0;
        rc = (can_boards[idx_board].type != EOF) ? CANERR_NOERROR : CANERR_RESOURCE;
        break;
    case CANPROP_SET_NEXT_CHANNEL:      // set index to the next entry in the interface list (NULL)
        if ((0 <= idx_board) && (idx_board < NUM_CHANNELS)) {
            if (can_boards[idx_board].type != EOF)
                idx_board++;
            rc = (can_boards[idx_board].type != EOF) ? CANERR_NOERROR : CANERR_RESOURCE;
        }
        else
            rc = CANERR_RESOURCE;
        break;
    case CANPROP_GET_CHANNEL_NO:        // get channel no. at actual index in the interface list (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                *(int32_t*)value = (int32_t)can_boards[idx_board].type;
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_NAME:      // get channel name at actual index in the interface list (char[256])
        if ((0U < nbyte) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                strncpy((char*)value, can_boards[idx_board].name, nbyte);
                ((char*)value)[(nbyte - 1)] = '\0';
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_DLLNAME:   // get file name of the DLL at actual index in the interface list (char[256])
        if ((0U < nbyte) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                strncpy((char*)value, DEV_DLLNAME, nbyte);
                ((char*)value)[(nbyte - 1)] = '\0';
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_VENDOR_ID: // get library id at actual index in the interface list (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                *(int32_t*)value = (int32_t)LIB_ID;
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_VENDOR_NAME: // get vendor name at actual index in the interface list (char[256])
        if ((0U < nbyte) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                strncpy((char*)value, DEV_VENDOR, nbyte);
                ((char*)value)[(nbyte - 1)] = '\0';
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_DEVICE_TYPE:       // device type of the CAN interface (int32_t)
    case CANPROP_GET_DEVICE_NAME:       // device name of the CAN interface (char[256])
    case CANPROP_GET_OP_CAPABILITY:     // supported operation modes of the CAN controller (uint8_t)
    case CANPROP_GET_OP_MODE:           // active operation mode of the CAN controller (uint8_t)
    case CANPROP_GET_BITRATE:           // active bit-rate of the CAN controller (can_bitrate_t)
    case CANPROP_GET_SPEED:             // active bus speed of the CAN controller (can_speed_t)
    case CANPROP_GET_STATUS:            // current status register of the CAN controller (uint8_t)
    case CANPROP_GET_BUSLOAD:           // current bus load of the CAN controller (uint16_t)
    case CANPROP_GET_NUM_CHANNELS:      // numbers of CAN channels on the CAN interface (uint8_t)
    case CANPROP_GET_CAN_CHANNEL:       // active CAN channel on the CAN interface (uint8_t)
    case CANPROP_GET_CAN_CLOCK:         // frequency of the CAN controller clock in [Hz] (int32_t)
    case CANPROP_GET_TX_COUNTER:        // total number of sent messages (uint64_t)
    case CANPROP_GET_RX_COUNTER:        // total number of reveiced messages (uint64_t)
    case CANPROP_GET_ERR_COUNTER:       // total number of reveiced error frames (uint64_t)
    case CANPROP_GET_RCV_QUEUE_SIZE:    // maximum number of message the receive queue can hold (uint32_t)
    case CANPROP_GET_RCV_QUEUE_HIGH:    // maximum number of message the receive queue has hold (uint32_t)
    case CANPROP_GET_RCV_QUEUE_OVFL:    // overflow counter of the receive queue (uint64_t)
    case CANPROP_GET_FILTER_11BIT:      // acceptance filter code and mask for 11-bit identifier (uint64_t)
    case CANPROP_GET_FILTER_29BIT:      // acceptance filter code and mask for 29-bit identifier (uint64_t)
    case CANPROP_SET_FILTER_11BIT:      // set value for acceptance filter code and mask for 11-bit identifier (uint64_t)
    case CANPROP_SET_FILTER_29BIT:      // set value for acceptance filter code and mask for 29-bit identifier (uint64_t)
    case CANPROP_SET_FILTER_RESET:      // reset acceptance filter code and mask to default values (NULL)
    case TOUCAN_GET_HARDWARE_VERSION:   // TouCAN USB: hardware version as "0xggrrss00" (uint32_t)
    case TOUCAN_GET_FIRMWARE_VERSION:   // TouCAN USB: firmware version as "0xggrrss00" (uint32_t)
    case TOUCAN_GET_BOOTLOADER_VERSION: // TouCAN USB: boot-loader version as "0xggrrss00" (uint32_t)
    case TOUCAN_GET_SERIAL_NUMBER:      // TouCAN USB: serial no. in hex (uint32_t)
    case TOUCAN_GET_VID_PID:            // TouCAN USB: VID & PID (uint32_t)
    case TOUCAN_GET_DEVICE_ID:          // TouCAN USB: device id. (uint32_t)
    case TOUCAN_GET_VENDOR_URL:         // TouCAN USB: URL of Rusoku's website (uint32_t)
        // note: a device parameter requires a valid handle.
        if (!init)
            rc = CANERR_NOTINIT;
        else
            rc = CANERR_HANDLE;
        break;
    default:
        rc = CANERR_NOTSUPP;
        break;
    }
    return rc;
}

static int drv_parameter(int handle, uint16_t param, void *value, size_t nbyte)
{
    int rc = CANERR_ILLPARA;            // suppose an invalid parameter
    can_bitrate_t bitrate;
    can_speed_t speed;
    uint8_t status;
    uint8_t load;

    assert(IS_HANDLE_VALID(handle));    // just to make sure

    if (value == NULL) {                // check for null-pointer
        if ((param != CANPROP_SET_FIRST_CHANNEL) &&
            (param != CANPROP_SET_NEXT_CHANNEL) &&
            (param != CANPROP_SET_FILTER_RESET))
            return CANERR_NULLPTR;
    }
    /* CAN interface properties */
    switch (param) {
    case CANPROP_GET_DEVICE_TYPE:       // device type of the CAN interface (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            *(int32_t*)value = (int32_t)can[handle].device.deviceInfo.type;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_NAME:       // device name of the CAN interface (char[256])
        if ((nbyte > strlen(can[handle].device.deviceInfo.name)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, can[handle].device.deviceInfo.name);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_VENDOR:     // vendor name of the CAN interface (char[256])
        if ((nbyte > strlen(can[handle].device.vendor)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, can[handle].device.vendor);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_DLLNAME:    // file name of the CAN interface DLL (char[256])
        if ((nbyte > strlen("(driverless)")) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strncpy((char*)value, "(driverless)", nbyte);  // note: there is no kernel driver!
            ((char*)value)[nbyte-1] = '\0';
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_OP_CAPABILITY:     // supported operation modes of the CAN controller (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            *(uint8_t*)value = (uint8_t)can[handle].device.opCapa;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_OP_MODE:           // active operation mode of the CAN controller (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            *(uint8_t*)value = (uint8_t)can[handle].mode.byte;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_BITRATE:           // active bit-rate of the CAN controller (can_bitrate_t)
        if (nbyte >= sizeof(can_bitrate_t)) {
            if (((rc = can_bitrate(handle, &bitrate, NULL)) == CANERR_NOERROR) || (rc == CANERR_OFFLINE)) {
                memcpy(value, &bitrate, sizeof(can_bitrate_t));
                rc = CANERR_NOERROR;
            }
        }
        break;
    case CANPROP_GET_SPEED:             // active bus speed of the CAN controller (can_speed_t)
        if (nbyte >= sizeof(can_speed_t)) {
            if (((rc = can_bitrate(handle, NULL, &speed)) == CANERR_NOERROR) || (rc == CANERR_OFFLINE)) {
                memcpy(value, &speed, sizeof(can_speed_t));
                rc = CANERR_NOERROR;
            }
        }
        break;
    case CANPROP_GET_STATUS:            // current status register of the CAN controller (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            if ((rc = can_status(handle, &status)) == CANERR_NOERROR) {
                *(uint8_t*)value = (uint8_t)status;
                rc = CANERR_NOERROR;
            }
        }
        break;
    case CANPROP_GET_BUSLOAD:           // current bus load of the CAN controller (uint16_t)
        if (nbyte >= sizeof(uint8_t)) {
            if ((rc = can_busload(handle, &load, NULL)) == CANERR_NOERROR) {  // FIXME: legacy resolution
                if (nbyte >= sizeof(uint16_t))
                    *(uint16_t*)value = (uint16_t)load * 100U;  // 0 - 10000 ==> 0.00% - 100.00%
                else
                    *(uint8_t*)value = (uint8_t)load;           // 0  -  100 ==> 0.00% - 100.00%
                rc = CANERR_NOERROR;
            }
        }
        break;
    case CANPROP_GET_NUM_CHANNELS:      // numbers of CAN channels on the CAN interface (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            *(uint8_t*)value = (uint8_t)1;  // FIXME: replace magic number
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_CAN_CHANNEL:       // active CAN channel on the CAN interface (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            *(uint8_t*)value = (uint8_t)0;  // FIXME: replace magic number
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_CAN_CLOCK:         // frequency of the CAN controller clock in [Hz] (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            *(int32_t*)value = (int32_t)can[handle].device.canClock;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_TX_COUNTER:        // total number of sent messages (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            *(uint64_t*)value = (uint64_t)can[handle].counters.tx;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_RX_COUNTER:        // total number of reveiced messages (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            *(uint64_t*)value = (uint64_t)can[handle].counters.rx;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_ERR_COUNTER:       // total number of reveiced error frames (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            *(uint64_t*)value = (uint64_t)can[handle].counters.err;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_RCV_QUEUE_SIZE:    // maximum number of message the receive queue can hold (uint32_t)
        if (nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)CANQUE_QueueSize(can[handle].device.recvData.msgQueue);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_RCV_QUEUE_HIGH:    // maximum number of message the receive queue has hold (uint32_t)
        if (nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)CANQUE_QueueHigh(can[handle].device.recvData.msgQueue);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_RCV_QUEUE_OVFL:    // overflow counter of the receive queue (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            *(uint64_t*)value = (uint64_t)CANQUE_OverflowCounter(can[handle].device.recvData.msgQueue);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_FILTER_11BIT:      // acceptance filter code and mask for 11-bit identifier (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            *(uint64_t*)value = ((uint64_t)can[handle].filter.std.code << 32)
                              | ((uint64_t)can[handle].filter.std.mask);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_FILTER_29BIT:      // acceptance filter code and mask for 29-bit identifier (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            *(uint64_t*)value = ((uint64_t)can[handle].filter.xtd.code << 32)
                              | ((uint64_t)can[handle].filter.xtd.mask);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_SET_FILTER_11BIT:      // set value for acceptance filter code and mask for 11-bit identifier (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            if (!(*(uint64_t*)value & 0xFFFFF800FFFFF800ULL)) {
                // note: code and mask must not exceed 11-bit identifier
                if (can[handle].status.can_stopped) {
                    // note: set filter only if the CAN controller is in INIT mode
                    rc= set_filter(handle, *(uint64_t*)value, false);
                }
                else
                    rc = CANERR_ONLINE;
            }
            else
                rc = CANERR_ILLPARA;
        }
        break;
    case CANPROP_SET_FILTER_29BIT:      // set value for acceptance filter code and mask for 29-bit identifier (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            if (!(*(uint64_t*)value & 0xE0000000E0000000ULL) && !can[handle].mode.nxtd) {
                // note: code and mask must not exceed 29-bit identifier and 29-bit mode must not be suppressed
                if (can[handle].status.can_stopped) {
                    // note: set filter only if the CAN controller is in INIT mode
                    rc = set_filter(handle, *(uint64_t*)value, true);
                }
                else
                    rc = CANERR_ONLINE;
            }
            else
                rc = CANERR_ILLPARA;
        }
        break;
    case CANPROP_SET_FILTER_RESET:      // reset acceptance filter code and mask to default values (NULL)
        if (can[handle].status.can_stopped) {
            // note: reset filter only if the CAN controller is in INIT mode
            rc = reset_filter(handle);
        }
        else
            rc = CANERR_ONLINE;
        break;
    /* TouCAN specific properties */
    case TOUCAN_GET_HARDWARE_VERSION:   // TouCAN USB: hardware version as "0xggrrss00" (uint32_t)
        if ((size_t)nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)can[handle].device.deviceInfo.hardware;
            rc = CANERR_NOERROR;
        }
        break;
    case TOUCAN_GET_FIRMWARE_VERSION:   // TouCAN USB: firmware version as "0xggrrss00" (uint32_t)
        if ((size_t)nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)can[handle].device.deviceInfo.firmware;
            rc = CANERR_NOERROR;
        }
        break;
    case TOUCAN_GET_BOOTLOADER_VERSION: // TouCAN USB: boot-loader version as "0xggrrss00" (uint32_t)
        if ((size_t)nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)can[handle].device.deviceInfo.bootloader;
            rc = CANERR_NOERROR;
        }
        break;
    case TOUCAN_GET_SERIAL_NUMBER:      // TouCAN USB: serial no. in hex (uint32_t)
        if ((size_t)nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)can[handle].device.deviceInfo.serialNo;
            rc = CANERR_NOERROR;
        }
        break;
    case TOUCAN_GET_VID_PID:            // TouCAN USB: VID & PID (uint32_t)
        if ((size_t)nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)can[handle].device.deviceInfo.vid_pid;
            rc = CANERR_NOERROR;
        }
        break;
    case TOUCAN_GET_DEVICE_ID:          // TouCAN USB: device id. (uint32_t)
        if ((size_t)nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)can[handle].device.deviceInfo.deviceId;
            rc = CANERR_NOERROR;
        }
        break;
    case TOUCAN_GET_VENDOR_URL:         // TouCAN USB: URL of Rusoku's website (uint32_t)
            if ((nbyte > strlen(can[handle].device.website)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
                strcpy((char*)value, can[handle].device.website);
                rc = CANERR_NOERROR;
            }
        break;
    default:
        rc = lib_parameter(param, value, nbyte);   // library properties (see lib_parameter)
        break;
    }
    return rc;
}

/*  -----------  revision control  ---------------------------------------
 */
EXPORT
char *can_version(void)
{
    return (char*)version;
}
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de, Homepage: https://www.uv-software.de/
 */
