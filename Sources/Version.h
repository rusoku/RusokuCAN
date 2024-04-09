/*  SPDX-License-Identifier: GPL-3.0-or-later */
/*
 *  CAN Interface API, Version 3 (for Rusoku TouCAN Interface)
 *
 *  Copyright (C) 2020-2024 Uwe Vogt, UV Software, Berlin (info@mac-can.com)
 *
 *  This file is part of MacCAN-TouCAN.
 *
 *  MacCAN-TouCAN is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  MacCAN-TouCAN is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with MacCAN-TouCAN.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef VERSION_H_INCLUDED
#define VERSION_H_INCLUDED
#include "build_no.h"
#define VERSION_MAJOR    0
#define VERSION_MINOR    3
#define VERSION_PATCH    0
#define VERSION_BUILD    BUILD_NO
#if (VERSION_PATCH == 0)
#define VERSION_STRING   TOSTRING(VERSION_MAJOR) "." TOSTRING(VERSION_MINOR) " (" TOSTRING(BUILD_NO) ")"
#else
#define VERSION_STRING   TOSTRING(VERSION_MAJOR) "." TOSTRING(VERSION_MINOR) "." TOSTRING(VERSION_PATCH) " (" TOSTRING(BUILD_NO) ")"
#endif
#endif
