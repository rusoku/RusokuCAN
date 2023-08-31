//  SPDX-License-Identifier: GPL-3.0-or-later
//
//  CAN Monitor for generic Interfaces (CAN API V3)
//
//  Copyright (c) 2007,2012-2023 Uwe Vogt, UV Software, Berlin (info@mac-can.com)
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
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include "Driver.h"
#include "Timer.h"
#include "Message.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <getopt.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

#include <inttypes.h>

#ifdef _MSC_VER
//not #if defined(_WIN32) || defined(_WIN64) because we have strncasecmp in mingw
#define strncasecmp _strnicmp
#define strcasecmp _stricmp
#endif

#define MAX_ID  (CAN_MAX_STD_ID + 1)

static int get_exclusion(const char *arg);  // TODO: make it a member function

class CCanDevice : public CCanDriver {
public:
    uint64_t ReceptionLoop();
public:
    static int ListCanDevices(void);
    static int TestCanDevices(CANAPI_OpMode_t opMode);
};

static void sigterm(int signo);
static void usage(FILE *stream, const char *program);
static void version(FILE *stream, const char *program);

static int can_id[MAX_ID];
static int can_id_xtd = 1;
static volatile int running = 1;

static CCanDevice canDevice = CCanDevice();

static const char APPLICATION[] = "CAN Monitor for " MONITOR_INTEFACE ", Version " VERSION_STRING;
static const char COPYRIGHT[]   = "Copyright (c) " MONITOR_COPYRIGHT;
static const char WARRANTY[]    = "This program comes with ABSOLUTELY NO WARRANTY!\n\n" \
                                  "This is free software, and you are welcome to redistribute it\n" \
                                  "under certain conditions; type '--version' for details.";
static const char LICENSE[]     = "This program is free software: you can redistribute it and/or modify\n" \
                                  "it under the terms of the GNU General Public License as published by\n" \
                                  "the Free Software Foundation, either version 3 of the License, or\n" \
                                  "(at your option) any later version.\n\n" \
                                  "This program is distributed in the hope that it will be useful,\n" \
                                  "but WITHOUT ANY WARRANTY; without even the implied warranty of\n" \
                                  "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n" \
                                  "GNU General Public License for more details.\n\n" \
                                  "You should have received a copy of the GNU General Public License\n" \
                                  "along with this program.  If not, see <http://www.gnu.org/licenses/>.";
#define basename(x)  "can_moni" // FIXME: Where is my `basename' function?

int main(int argc, const char * argv[]) {
    int opt;
    CCanDevice::SChannelInfo channel;
    int op = 0, rf = 0, xf = 0, ef = 0, lo = 0, sh = 0;
    long baudrate = CANBDR_250; int bd = 0;
    CCanMessage::EFormatTimestamp modeTime = CCanMessage::OptionZero; int mt = 0;
    CCanMessage::EFormatNumber modeId = CCanMessage::OptionHex; int mi = 0;
    CCanMessage::EFormatNumber modeData = CCanMessage::OptionHex; int md = 0;
    CCanMessage::EFormatOption modeAscii = CCanMessage::OptionOn; int ma = 0;
    CCanMessage::EFormatWraparound wraparound = CCanMessage::OptionWraparoundNo; int mw = 0;
    int exclude = 0;
//    char *script_file = NULL;
    int verbose = 0;
    int num_boards = 0;
    int show_version = 0;
    char *device, *firmware, *software;
    char property[CANPROP_MAX_BUFFER_SIZE] = "";
    struct option long_options[] = {
        {"baudrate", required_argument, 0, 'b'},
        {"bitrate", required_argument, 0, 'B'},
        {"verbose", no_argument, 0, 'v'},
        {"mode", required_argument, 0, 'm'},
        {"shared", no_argument, 0, 'S'},
        {"listen-only", no_argument, 0, 'M'},
        {"error-frames", no_argument, 0, 'E'},
        {"no-remote-frames", no_argument, 0, 'R'},
        {"no-extended-frames", no_argument, 0, 'X'},
        {"time", required_argument, 0, 't'},
        {"id", required_argument, 0, 'i'},
        {"data", required_argument, 0, 'd'},
        {"ascii", required_argument, 0, 'a'},
        {"wrap", required_argument, 0, 'w'},
        {"wraparound", required_argument, 0, 'w'},
        {"exclude", required_argument, 0, 'x'},
        {"script", required_argument, 0, 's'},
        {"list-boards", no_argument, 0, 'L'},
        {"test-boards", no_argument, 0, 'T'},
        {"help", no_argument, 0, 'h'},
        {"version", no_argument, &show_version, 1},
        {0, 0, 0, 0}
    };
    CANAPI_Bitrate_t bitrate = {};
    bitrate.index = CANBTR_INDEX_250K;
    bool hasDataPhase = false;
    bool hasNoSamp = true;
    CANAPI_OpMode_t opMode = {};
    opMode.byte = CANMODE_DEFAULT;
    CANAPI_Return_t retVal = 0;

    /* default bit-timing */
    CANAPI_BusSpeed_t speed = {};
    (void)CCanDevice::MapIndex2Bitrate(bitrate.index, bitrate);
    (void)CCanDevice::MapBitrate2Speed(bitrate, speed);
    (void)op;

    /* default format options */
    (void)CCanMessage::SetTimestampFormat(modeTime);
    (void)CCanMessage::SetIdentifierFormat(modeId);
    (void)CCanMessage::SetDataFormat(modeData);
    (void)CCanMessage::SetAsciiFormat(modeAscii);
    (void)CCanMessage::SetWraparound(wraparound);
    (void)mw;

    /* exclude list (11-bit IDs only) */
    for (int i = 0; i < MAX_ID; i++) {
        can_id[i] = 1;
    }
    /* signal handler */
    if ((signal(SIGINT, sigterm) == SIG_ERR) ||
#if !defined(_WIN32) && !defined(_WIN64)
       (signal(SIGHUP, sigterm) == SIG_ERR) ||
#endif
       (signal(SIGTERM, sigterm) == SIG_ERR)) {
        perror("+++ error");
        return errno;
    }
    /* scan command-line */
    while ((opt = getopt_long(argc, (char * const *)argv, "b:vm:t:i:d:a:w:x:s:LTh", long_options, NULL)) != -1) {
        switch (opt) {
        case 'b':  /* option `--baudrate=<baudrate>' (-b) */
            if (bd++) {
                fprintf(stderr, "%s: duplicated option `--baudrate' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (sscanf(optarg, "%li", &baudrate) != 1) {
                fprintf(stderr, "%s: illegal argument for option `--baudrate' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            switch (baudrate) {
                case 0: case 1000: case 1000000: bitrate.index = (int32_t)CANBTR_INDEX_1M; break;
                case 1: case 800:  case 800000:  bitrate.index = (int32_t)CANBTR_INDEX_800K; break;
                case 2: case 500:  case 500000:  bitrate.index = (int32_t)CANBTR_INDEX_500K; break;
                case 3: case 250:  case 250000:  bitrate.index = (int32_t)CANBTR_INDEX_250K; break;
                case 4: case 125:  case 125000:  bitrate.index = (int32_t)CANBTR_INDEX_125K; break;
                case 5: case 100:  case 100000:  bitrate.index = (int32_t)CANBTR_INDEX_100K; break;
                case 6: case 50:   case 50000:   bitrate.index = (int32_t)CANBTR_INDEX_50K; break;
                case 7: case 20:   case 20000:   bitrate.index = (int32_t)CANBTR_INDEX_20K; break;
                case 8: case 10:   case 10000:   bitrate.index = (int32_t)CANBTR_INDEX_10K; break;
                default:                         bitrate.index = (int32_t)-baudrate; break;
            }
            if (CCanDevice::MapIndex2Bitrate(bitrate.index, bitrate) != CCanApi::NoError) {
                fprintf(stderr, "%s: illegal argument for option `--baudrate' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (CCanDevice::MapBitrate2Speed(bitrate, speed) != CCanApi::NoError) {
                fprintf(stderr, "%s: illegal argument for option `--baudrate' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            break;
        case 'B':  /* option `--bitrate=<bit-rate>' as string */
            if (bd++) {
                fprintf(stderr, "%s: duplicated option `--bitrate'\n", basename(argv[0]));
                return 1;
            }
            if (CCanDevice::MapString2Bitrate(optarg, bitrate, hasDataPhase, hasNoSamp) != CCanApi::NoError) {
                fprintf(stderr, "%s: illegal argument for option `--bitrate'\n", basename(argv[0]));
                return 1;
            }
            if (CCanDevice::MapBitrate2Speed(bitrate, speed) != CCanApi::NoError) {
                fprintf(stderr, "%s: illegal argument for option `--bitrate'\n", basename(argv[0]));
                return 1;
            }
            break;
        case 'v':  /* option `--verbose' (-v) */
            if (verbose) {
                fprintf(stderr, "%s: duplicated option `--verbose' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            verbose = 1;
            break;
#if (CAN_FD_SUPPORTED != 0)
        case 'm':  /* option `--mode=(2.0|FDF[+BSR])' (-m)*/
            if (op++) {
                fprintf(stderr, "%s: duplicated option `--mode' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!strcasecmp(optarg, "default") || !strcasecmp(optarg, "classic") ||
                !strcasecmp(optarg, "CAN2.0") || !strcasecmp(optarg, "CAN20") || !strcasecmp(optarg, "2.0"))
                opMode.byte |= CANMODE_DEFAULT;
            else if (!strcasecmp(optarg, "CANFD") || !strcasecmp(optarg, "FDF") || !strcasecmp(optarg, "FD"))
                opMode.byte |= CANMODE_FDOE;
            else if (!strcasecmp(optarg, "CANFD+BRS") || !strcasecmp(optarg, "FDF+BRS") || !strcasecmp(optarg, "FD+BRS"))
                opMode.byte |= CANMODE_FDOE | CANMODE_BRSE;
            else {
                fprintf(stderr, "%s: illegal argument for option `--mode' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            break;
#endif
        case 'S':  /* option `--shared' */
            if (sh++) {
                fprintf(stderr, "%s: duplicated option `--shared'\n", basename(argv[0]));
                return 1;
            }
            opMode.shrd = 1;
            break;
        case 'M':  /* option `--listen-only' */
            if (lo++) {
                fprintf(stderr, "%s: duplicated option `--listen-only'\n", basename(argv[0]));
                return 1;
            }
            opMode.mon = 1;
            break;
        case 'E':  /* option `--error-frames' */
            if (ef++) {
                fprintf(stderr, "%s: duplicated option `--error-frames'\n", basename(argv[0]));
                return 1;
            }
            opMode.err = 1;
            break;
        case 'X':  /* option `--no-extended-frames' */
            if (xf++) {
                fprintf(stderr, "%s: duplicated option `--no-extended-frames'\n", basename(argv[0]));
                return 1;
            }
            opMode.nxtd = 1;
            break;
        case 'R':  /* option `--no-remote-frames' */
            if (rf++) {
                fprintf(stderr, "%s: duplicated option `--no-remote-frames'\n", basename(argv[0]));
                return 1;
            }
            opMode.nrtr = 1;
            break;
        case 't':  /* option `--time=(ABS|REL|ZERO)' (-t) */
            if (mt++) {
                fprintf(stderr, "%s: duplicated option `--time' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!strcasecmp(optarg, "ABSOLUTE") || !strcasecmp(optarg, "ABS") || !strcasecmp(optarg, "a"))
                modeTime = CCanMessage::OptionAbsolute;
            else if (!strcasecmp(optarg, "RELATIVE") || !strcasecmp(optarg, "REL") || !strcasecmp(optarg, "r"))
                modeTime = CCanMessage::OptionRelative;
            else if (!strcasecmp(optarg, "ZERO") || !strcasecmp(optarg, "0") || !strcasecmp(optarg, "z"))
                modeTime = CCanMessage::OptionZero;
            else {
                fprintf(stderr, "%s: illegal argument for option `--time' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!CCanMessage::SetTimestampFormat(modeTime)) {
                fprintf(stderr, "%s: illegal argument for option `--time' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            break;
        case 'i':  /* option `--id=(HEX|DEC|OCT)' (-i) */
            if (mi++) {
                fprintf(stderr, "%s: duplicated option `--id' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!strcasecmp(optarg, "HEXADECIMAL") || !strcasecmp(optarg, "HEX") || !strcasecmp(optarg, "h") || !strcasecmp(optarg, "16"))
                modeId = CCanMessage::OptionHex;
            else if (!strcasecmp(optarg, "DECIMAL") || !strcasecmp(optarg, "DEC") || !strcasecmp(optarg, "d") || !strcasecmp(optarg, "10"))
                modeId = CCanMessage::OptionDec;
            else if (!strcasecmp(optarg, "OCTAL") || !strcasecmp(optarg, "OCT") || !strcasecmp(optarg, "o") || !strcasecmp(optarg, "8"))
                modeId = CCanMessage::OptionOct;
            else {
                fprintf(stderr, "%s: illegal argument for option `--id' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!CCanMessage::SetIdentifierFormat(modeId)) {
                fprintf(stderr, "%s: illegal argument for option `--id' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            break;
        case 'd':  /* option `--data=(HEX|DEC|OCT)' (-d) */
            if (md++) {
                fprintf(stderr, "%s: duplicated option `--data' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!strcasecmp(optarg, "HEXADECIMAL") || !strcasecmp(optarg, "HEX") || !strcasecmp(optarg, "h") || !strcasecmp(optarg, "16"))
                modeData = CCanMessage::OptionHex;
            else if (!strcasecmp(optarg, "DECIMAL") || !strcasecmp(optarg, "DEC") || !strcasecmp(optarg, "d") || !strcasecmp(optarg, "10"))
                modeData = CCanMessage::OptionDec;
            else if (!strcasecmp(optarg, "OCTAL") || !strcasecmp(optarg, "OCT") || !strcasecmp(optarg, "o") || !strcasecmp(optarg, "8"))
                modeData = CCanMessage::OptionOct;
            else {
                fprintf(stderr, "%s: illegal argument for option `--data' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!CCanMessage::SetDataFormat(modeData)) {
                fprintf(stderr, "%s: illegal argument for option `--data' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            break;
        case 'a':  /* option `--ascii=(ON|OFF)' (-a) */
            if (ma++) {
                fprintf(stderr, "%s: duplicated option `--ascii' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!strcasecmp(optarg, "OFF") || !strcasecmp(optarg, "NO") || !strcasecmp(optarg, "n") || !strcasecmp(optarg, "0"))
                modeAscii = CCanMessage::OptionOff;
            else if (!strcasecmp(optarg, "ON") || !strcasecmp(optarg, "YES") || !strcasecmp(optarg, "y") || !strcasecmp(optarg, "1"))
                modeAscii = CCanMessage::OptionOn;
            else {
                fprintf(stderr, "%s: illegal argument for option `--ascii' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!CCanMessage::SetAsciiFormat(modeAscii)) {
                fprintf(stderr, "%s: illegal argument for option `--ascii' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            break;
#if (CAN_FD_SUPPORTED != 0)
        case 'w':  /* option `--wrap=....' (-w) */
            if (mw++) {
                fprintf(stderr, "%s: duplicated option `--wrap' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!strcasecmp(optarg, "NO") || !strcasecmp(optarg, "n") || !strcasecmp(optarg, "0"))
                wraparound = CCanMessage::OptionWraparoundNo;
            else if (!strcasecmp(optarg, "8"))
                wraparound = CCanMessage::OptionWraparound8;
            else if (!strcasecmp(optarg, "10"))
                wraparound = CCanMessage::OptionWraparound10;
            else if (!strcasecmp(optarg, "16"))
                wraparound = CCanMessage::OptionWraparound16;
            else if (!strcasecmp(optarg, "32"))
                wraparound = CCanMessage::OptionWraparound32;
            else if (!strcasecmp(optarg, "64"))
                wraparound = CCanMessage::OptionWraparound64;
            else {
                    fprintf(stderr, "%s: illegal argument for option `--wrap' (%c)\n", basename(argv[0]), opt);
                    return 1;
            }
            if (!CCanMessage::SetWraparound(wraparound)) {
                fprintf(stderr, "%s: illegal argument for option `--wrap' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            break;
#endif
        case 'x':  /* option `--exclude=[~]<id-list>' (-x) */
            if (exclude++) {
                fprintf(stderr, "%s: duplicated option `--exclude' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            if (!get_exclusion(optarg)) {
                fprintf(stderr, "%s: illegal argument for option `--exclude' (%c)\n", basename(argv[0]), opt);
                return 1;
            }
            break;
        case 'L':  /* option `--list-boards[=<vendor>]' (-L) */
            fprintf(stdout, "%s\n%s\n\n%s\n\n", APPLICATION, COPYRIGHT, WARRANTY);
            /* list all supported interfaces */
            num_boards = CCanDevice::ListCanDevices(/*optarg*/);
            fprintf(stdout, "Number of supported CAN interfaces: %i\n", num_boards);
            return (num_boards >= 0) ? 0 : 1;
        case 'T':  /* option `--test-boards[=<vendor>]' (-T) */
            fprintf(stdout, "%s\n%s\n\n%s\n\n", APPLICATION, COPYRIGHT, WARRANTY);
            /* list all available interfaces */
            num_boards = CCanDevice::TestCanDevices(opMode/*, optarg*/);
            fprintf(stdout, "Number of present CAN interfaces: %i\n", num_boards);
            return (num_boards >= 0) ? 0 : 1;
        case 'h':  /* option `--help' (-h) */
            usage(stdout, basename(argv[0]));
            return 0;
        case '?':
            if (!opterr)
                usage(stderr, basename(argv[0]));
            return 1;
        default:
            if (show_version) {
                version(stdout, basename(argv[0]));
                return 0;
            }
            else {
                usage(stderr, basename(argv[0]));
                return 1;
            }
        }
    }
    /* - check if one and only one <interface> is given */
    if (optind + 1 != argc) {
        if (optind == argc)
            fprintf(stderr, "%s: no interface given\n", basename(argv[0]));
        else
            fprintf(stderr, "%s: too many arguments given\n", basename(argv[0]));
        return 1;
    }
    /* - search the <interface> by its name in the device list */
    bool result = CCanDevice::GetFirstChannel(channel);
    while (result) {
        if (strcasecmp(argv[optind], channel.m_szDeviceName) == 0) {
            break;
        }
        result = CCanDevice::GetNextChannel(channel);
    }
    if (!result) {
        fprintf(stderr, "%s: illegal argument `%s'\n", basename(argv[0]), argv[optind]);
        return 1;
    }
    /* - check bit-timing index (n/a for CAN FD) */
    if (opMode.fdoe && (bitrate.btr.frequency <= 0)) {
        fprintf(stderr, "%s: illegal combination of options `--mode' (m) and `--bitrate'\n", basename(argv[0]));
        return 1;
    }
    /* CAN Monitor for genericc CAN interfaces */
    fprintf(stdout, "%s\n%s\n\n%s\n\n", APPLICATION, COPYRIGHT, WARRANTY);
    /* - show operation mode and bit-rate settings */
    if (verbose) {
        fprintf(stdout, "Op.-mode=%s", (opMode.byte & CANMODE_FDOE) ? "CANFD" : "CAN2.0");
        if ((opMode.byte & CANMODE_BRSE)) fprintf(stdout, "+BRS");
        if ((opMode.byte & CANMODE_NISO)) fprintf(stdout, "+NISO");
        if ((opMode.byte & CANMODE_SHRD)) fprintf(stdout, "+SHRD");
        if ((opMode.byte & CANMODE_NXTD)) fprintf(stdout, "+NXTD");
        if ((opMode.byte & CANMODE_NRTR)) fprintf(stdout, "+NRTR");
        if ((opMode.byte & CANMODE_ERR)) fprintf(stdout, "+ERR");
        if ((opMode.byte & CANMODE_MON)) fprintf(stdout, "+MON");
        fprintf(stdout, " (op_mode=%02Xh)\n", opMode.byte);
        if (bitrate.btr.frequency > 0) {
            fprintf(stdout, "Bit-rate=%.0fkbps@%.1f%%",
                speed.nominal.speed / 1000.,
                speed.nominal.samplepoint * 100.);
            if (opMode.byte & CANMODE_BRSE)
                fprintf(stdout, ":%.0fkbps@%.1f%%",
                    speed.data.speed / 1000.,
                    speed.data.samplepoint * 100.);
            (void)CCanDevice::MapBitrate2String(bitrate, property, CANPROP_MAX_BUFFER_SIZE,
                                                (opMode.byte & CANMODE_BRSE), hasNoSamp);
            fprintf(stdout, " (%s)\n\n", property);
        }
        else {
            fprintf(stdout, "Baudrate=%.0fkbps@%.1f%% (index %i)\n\n",
                             speed.nominal.speed / 1000.,
                             speed.nominal.samplepoint * 100., -bitrate.index);
        }
    }
    /* - initialize interface */
    fprintf(stdout, "Hardware=%s...", channel.m_szDeviceName);
    fflush (stdout);
    retVal = canDevice.InitializeChannel(channel.m_nChannelNo, opMode);
    if (retVal != CCanApi::NoError) {
        fprintf(stdout, "FAILED!\n");
        fprintf(stderr, "+++ error: CAN Controller could not be initialized (%i)", retVal);
        if (retVal == CCanApi::NotSupported)
            fprintf(stderr, "\n           - possibly CAN operating mode %02Xh not supported", opMode.byte);
        fputc('\n', stderr);
        goto finalize;
    }
    fprintf(stdout, "OK!\n");
    /* - start communication */
    if (bitrate.btr.frequency > 0) {
        fprintf(stdout, "Bit-rate=%.0fkbps", speed.nominal.speed / 1000.);
        if (opMode.byte & CANMODE_BRSE)
            fprintf(stdout, ":%.0fkbps", speed.data.speed / 1000.);
        else if (opMode.byte & CANMODE_FDOE)
            fprintf(stdout, ":%.0fkbps", speed.nominal.speed / 1000.);
        fprintf(stdout, "...");
    }
    else {
        fprintf(stdout, "Baudrate=%skbps...",
            bitrate.index == CANBTR_INDEX_1M   ? "1000" :
            bitrate.index == CANBTR_INDEX_800K ? "800" :
            bitrate.index == CANBTR_INDEX_500K ? "500" :
            bitrate.index == CANBTR_INDEX_250K ? "250" :
            bitrate.index == CANBTR_INDEX_125K ? "125" :
            bitrate.index == CANBTR_INDEX_100K ? "100" :
            bitrate.index == CANBTR_INDEX_50K  ? "50" :
            bitrate.index == CANBTR_INDEX_20K  ? "20" :
            bitrate.index == CANBTR_INDEX_10K  ? "10" : "?");
    }
    fflush(stdout);
    retVal = canDevice.StartController(bitrate);
    if (retVal != CCanApi::NoError) {
        fprintf(stdout, "FAILED!\n");
        fprintf(stderr, "+++ error: CAN Controller could not be started (%i)\n", retVal);
        goto teardown;
    }
    fprintf(stdout, "OK!\n");
    /* - reception loop */
    canDevice.ReceptionLoop();
    /* - show interface information */
    if ((device = canDevice.GetHardwareVersion()) != NULL)
        fprintf(stdout, "Hardware: %s\n", device);
    if ((firmware = canDevice.GetFirmwareVersion()) != NULL)
        fprintf(stdout, "Firmware: %s\n", firmware);
    if ((software = CCanDevice::GetVersion()) != NULL)
        fprintf(stdout, "Software: %s\n", software);
teardown:
    /* - teardown the interface*/
    retVal = canDevice.TeardownChannel();
    if (retVal != CCanApi::NoError) {
        fprintf(stderr, "+++ error: CAN Controller could not be reset (%i)\n", retVal);
        goto finalize;
    }
finalize:
    /* So long and farewell! */
    fprintf(stdout, "%s\n", COPYRIGHT);
    return retVal;
}

int CCanDevice::ListCanDevices(void) {
    CCanDevice::SChannelInfo info;
    int n = 0;

    fprintf(stdout, "Suppored hardware:\n");
    bool result = CCanDevice::GetFirstChannel(info);
    while (result) {
        fprintf(stdout, "\"%s\" (VendorName=\"%s\", LibraryId=%" PRIi32 ", ChannelNo=%" PRIi32 ")\n",
                         info.m_szDeviceName, info.m_szVendorName, info.m_nLibraryId, info.m_nChannelNo);
        n++;
        result = CCanDevice::GetNextChannel(info);
    }
    return n;
}

int CCanDevice::TestCanDevices(CANAPI_OpMode_t opMode) {
    CCanDevice::SChannelInfo info;
    int n = 0;

    bool result = CCanDevice::GetFirstChannel(info);
    while (result) {
        fprintf(stdout, "Hardware=%s...", info.m_szDeviceName);
        fflush(stdout);
        EChannelState state;
        CANAPI_Return_t retVal = CCanDevice::ProbeChannel(info.m_nChannelNo, opMode, state);
        if ((retVal == CCanApi::NoError) || (retVal == CCanApi::IllegalParameter)) {
            CTimer::Delay(333U * CTimer::MSEC);  // to fake probing a hardware
            switch (state) {
                case CCanApi::ChannelOccupied: fprintf(stdout, "occupied\n"); n++; break;
                case CCanApi::ChannelAvailable: fprintf(stdout, "available\n"); n++; break;
                case CCanApi::ChannelNotAvailable: fprintf(stdout, "not available\n"); break;
                default: fprintf(stdout, "not testable\n"); break;
            }
            if (retVal == CCanApi::IllegalParameter)
                fprintf(stderr, "+++ warning: CAN operation mode not supported (%02xh)\n", opMode.byte);
        } else
            fprintf(stdout, "FAILED!\n");
        result = CCanDevice::GetNextChannel(info);
    }
    return n;
}

uint64_t CCanDevice::ReceptionLoop() {
    CANAPI_Message_t message;
    CANAPI_Return_t retVal;
    uint64_t frames = 0U;

    char string[CANPROP_MAX_STRING_LENGTH+1];
    memset(string, 0, CANPROP_MAX_STRING_LENGTH+1);

    fprintf(stderr, "\nPress ^C to abort.\n\n");
    while(running) {
        if ((retVal = ReadMessage(message)) == CCanApi::NoError) {
            if ((((message.id < MAX_ID) && can_id[message.id]) || ((message.id >= MAX_ID) && can_id_xtd))) {
                (void)CCanMessage::Format(message, ++frames, string, CANPROP_MAX_STRING_LENGTH);
                fprintf(stdout, "%s\n", string);
            }
        }
    }
    fprintf(stdout, "\n");
    return frames;
}

static int get_exclusion(const char *arg)
{
    char *val, *end;
    int i, inv = 0;
    long id, last = -1;

    if (!arg)
        return 0;

    val = (char *)arg;
    if (*val == '~') {
        inv = 1;
        val++;
    }
    for (;;) {
        errno = 0;
        id = strtol(val, &end, 0);

        if (errno == ERANGE && (id == LONG_MAX || id == LONG_MIN))
            return 0;
        if (errno != 0 && id == 0)
            return 0;
        if (val == end)
            return 0;

        if (id < MAX_ID)
            can_id[id] = 0;

        if (*end == '\0') {
            if (last != -1) {
                while (last != id) {
                    if (last < id)
                        last++;
                    else
                        last--;
                    can_id[last] = 0;
                }
                /*last = -1; <<< dead store */
            }
            break;
        }
        if (*end == ',') {
            if (last != -1) {
                while (last != id) {
                    if (last < id)
                        last++;
                    else
                        last--;
                    can_id[last] = 0;
                }
                last = -1;
            }
        }
        else if (*end == '-')
            last = id;
        else
            return 0;

        val = ++end;
    }
    if (inv) {
        for (i = 0; i < MAX_ID; i++)
            can_id[i] = !can_id[i];
    }
    can_id_xtd = !inv;
    return 1;
}

/** @brief       signal handler to catch Ctrl+C.
 *
 *  @param[in]   signo - signal number (SIGINT, SIGHUP, SIGTERM)
 */
static void sigterm(int signo)
{
    //fprintf(stderr, "%s: got signal %d\n", __FILE__, signo);
    (void)canDevice.SignalChannel();
    running = 0;
    (void)signo;
}

/** @brief       shows a help screen with all command-line options.
 *
 *  @param[in]   stream  - output stream (e.g. stdout)
 *  @param[in]   program - base name of the program
 */
static void usage(FILE *stream, const char *program)
{
    fprintf(stream, "Usage: %s <interface> [<option>...]\n", program);
    fprintf(stream, "Options:\n");
    fprintf(stream, " -t, --time=(ZERO|ABS|REL)     absolute or relative time (default=0)\n");
    fprintf(stream, " -i  --id=(HEX|DEC|OCT)        display mode of CAN-IDs (default=HEX)\n");
    fprintf(stream, " -d, --data=(HEX|DEC|OCT)      display mode of data bytes (default=HEX)\n");
    fprintf(stream, " -a, --ascii=(ON|OFF)          display data bytes in ASCII (default=ON)\n");
#if (CAN_FD_SUPPORTED != 0)
    fprintf(stream, " -w, --wrap=(NO|8|10|16|32|64) wraparound after n data bytes (default=NO)\n");
#endif
    fprintf(stream, " -x, --exclude=[~]<id-list>    exclude CAN-IDs: <id>[-<id>]{,<id>[-<id>]}\n");
//    fprintf(stream, " -s, --script=<filename>       execute a script file\n"); // TODO: script engine
#if (CAN_FD_SUPPORTED != 0)
    fprintf(stream, " -m, --mode=(2.0|FDF[+BSR])    CAN operation mode: CAN 2.0 or CAN FD format\n");
#endif
    fprintf(stream, "     --shared                  shared CAN controller access (if supported)\n");
    fprintf(stream, "     --listen-only             monitor mode (listen-only, transmitter is off)\n");
    fprintf(stream, "     --error-frames            allow reception of error frames\n");
    fprintf(stream, "     --no-remote-frames        suppress remote frames (RTR frames)\n");
    fprintf(stream, "     --no-extended-frames      suppress extended frames (29-bit identifier)\n");
    fprintf(stream, " -b, --baudrate=<baudrate>     CAN bit-timing in kbps (default=250), or\n");
    fprintf(stream, "     --bitrate=<bit-rate>      CAN bit-rate settings (as a string)\n");
    fprintf(stream, " -v, --verbose                 show detailed bit-rate settings\n");
#if (OPTION_CANAPI_LIBRARY != 0)
    fprintf(stream, " -L, --list-boards[=<vendor>]  list all supported CAN interfaces and exit\n");
    fprintf(stream, " -T, --test-boards[=<vendor>]  list all available CAN interfaces and exit\n");
#else
    fprintf(stream, " -L, --list-boards             list all supported CAN interfaces and exit\n");
    fprintf(stream, " -T, --test-boards             list all available CAN interfaces and exit\n");
#endif
    fprintf(stream, " -h, --help                    display this help screen and exit\n");
    fprintf(stream, "     --version                 show version information and exit\n");
    fprintf(stream, "Hazard note:\n");
    fprintf(stream, "  If you connect your CAN device to a real CAN network when using this program,\n");
    fprintf(stream, "  you might damage your application.\n");
}

/** @brief       shows version information of the program.
 *
 *  @param[in]   stream  - output stream (e.g. stdout)
 *  @param[in]   program - base name of the program
 */
static void version(FILE *stream, const char *program)
{
    fprintf(stdout, "%s\n%s\n\n%s\n\n", APPLICATION, COPYRIGHT, LICENSE);
    (void)program;
    fprintf(stream, "Written by Uwe Vogt, UV Software, Berlin <https://www.mac-can.net/>\n");
}
