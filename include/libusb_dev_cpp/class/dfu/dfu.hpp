/**
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include <cstdint>

enum class DFU_CLASS_CODE : uint8_t
{
	DFU = 0xFE
};

enum class DFU_SUBCLASS_CODE : uint8_t
{
	DFU = 0x01
};

enum class DFU_PROTO_CODE : uint8_t
{
	RUNTIME = 0x01,
	DFU = 0x02,
};

enum class DTYPE_DFU : uint8_t
{
	FUNCTIONAL = 0x21
};

enum class DFU_REQUESTS : uint8_t
{
	DETACH     = 0x00,
	DNLOAD     = 0x01,
	UPLOAD     = 0x02,
	GETSTATUS  = 0x03,
	CLRSTATAUS = 0x04,
	GETSTATE   = 0x05,
	ABORT      = 0x06
};

enum class DFU_ATTR : uint8_t
{
	CAN_DNLOAD  = 0x01,
	CAN_UPLOAD  = 0x02,
	MANIF_TOL   = 0x04,
	WILL_DETACH = 0x08
};

enum class DFU_STATUS : uint8_t
{
	OK               = 0x00,
	ERR_TARGET       = 0x01,
	ERR_FILE         = 0x02,
	ERR_WRITE        = 0x03,
	ERR_ERASE        = 0x04,
	ERR_CHECK_ERASED = 0x05,
	ERR_PROG         = 0x06,
	ERR_VERIFY       = 0x07,
	ERR_ADDRESS      = 0x08,
	ERR_NOTDONE      = 0x09,
	ERR_FIRMWARE     = 0x0A,
	ERR_VENDOR       = 0x0B,
	ERR_USBR         = 0x0C,
	ERR_POR          = 0x0D,
	ERR_UNKNOWN      = 0x0E,
	ERR_STALLEDPKT   = 0x0F
};

enum class DFU_STATE : uint8_t
{
	APP_IDLE         = 0x00,
	APP_DETACH       = 0x01,
	DFU_IDLE         = 0x02,
	DFU_DNLOADSYNC   = 0x03,
	DFU_DNBUSY       = 0x04,
	DFU_DNLOADIDLE   = 0x05,
	DFU_MANIFESTSYNC = 0x06,
	DFU_MANIFEST     = 0x07,
	DFU_MANIFESTWR   = 0x08,
	DFU_UPLOADIDLE   = 0x09,
	DFU_ERROR        = 0x0A
};

class DFU_functional_descriptor
{
	uint8_t  bLength;
	uint8_t  bDescriptorType;
	uint8_t  bmAttributes;
	uint16_t wDetachTimeout;
	uint16_t wTransferSize;
	uint16_t bcdDFUVersion;
};

class DFU_status
{
	uint8_t  bStatus;
	uint8_t  bPollTimeout;
	uint16_t wPollTimeout;
	uint8_t  bState;
	uint8_t  iString;
};