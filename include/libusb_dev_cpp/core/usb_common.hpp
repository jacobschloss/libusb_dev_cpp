/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "common_util/Byte_util.hpp"

#include <functional>

#include <cstdint>


class USB_common_private_func
{
public:
	//c++ does not allow constexpr functions to be used to init static members in the same class on some compilers
	//part of a strict ordering interdependance issue
	static constexpr uint16_t build_bcd(const uint8_t major, const uint8_t minor, const uint8_t subminor)
	{
		//0xJJMN
		return Byte_util::make_u16(major, ((minor & 0x0F) << 4) | (subminor & 0x0F));
	}
};

class USB_common
{
public:

	enum class USB_EVENTS
	{
		EARLY_SUSPEND,
		SUSPEND,
		RESET,
		ENUM_DONE,
		CTRL_SETUP_PHASE_DONE,
		CTRL_DATA_PHASE_DONE,
		EP_RX,
		EP_TX,
		WAKEUP,
		SOF,
		NONE
	};
	static constexpr size_t USB_EVENTS_MAX = 8;

	enum class USB_CONTROL_TYPE
	{
		NO_DATA,
		HOST_WRITE,
		HOST_READ
	};

	enum class USB_CONTROL_STATE
	{
		WAIT_SETUP,
		WAIT_DATA_OUT,
		WAIT_DATA_IN,
		WAIT_STATUS_OUT,
		WAIT_STATUS_IN
	};

	enum class USB_RESP
	{
		FAIL,
		ACK,
		NAK
	};

	enum class USB_SPEED
	{
		UNK,
		LS,           // aka USB 1.0, 1.5 Mbps
		FS,           // aka USB 1.1, 12 Mbps
		HS,           // aka USB 2.0, 480 Mbps
		SS_32_GEN1x1, // aka USB 3.0, USB 3.1 Gen 1, USB 3.2 Gen 1, 5.0 Gbps
		SS_32_GEN2x1, // aka USB 3.1, USB 3.1 Gen 2, USB 3.2 Gen 2, 10.0 Gbps
		SS_32_GEN2x2, // USB 3.2 Gen 2x2, 20.0 Gbps
		SS_40_GEN3x2  // USB 4 Gen 3x2, 40.0 Gbps
	};

	enum class STANDARD_REQUEST_CODES : uint8_t
	{
		// USB 2.0 Table 9-4 Standard Request Codes
		GET_STATUS          = 0x00, // 0
		CLEAR_FEATURE       = 0x01, // 1
		SET_FEATURE         = 0x03, // 3
		SET_ADDRESS         = 0x05, // 5
		GET_DESCRIPTOR      = 0x06, // 6
		SET_DESCRIPTOR      = 0x07, // 7
		GET_CONFIGURATION   = 0x08, // 8
		SET_CONFIGURATION   = 0x09, // 9
		GET_INTERFACE       = 0x0A, // 10
		SET_INTERFACE       = 0x0B, // 11
		SYNCH_FRAME         = 0x0C, // 12

		// USB 3.2 Revision 1.0 Table 9-5 Standard Request Codes
		SET_ENCRYPTION      = 0x0D, // 13
		GET_ENCRYPTION      = 0x0E, // 14
		SET_HANDSHAKE       = 0x0F, // 15
		GET_HANDSHAKE       = 0x10, // 16
		SET_CONNECTION      = 0x11, // 17
		SET_SECURITY_DATA   = 0x12, // 18
		GET_SECURITY_DATA   = 0x13, // 19
		SET_WUSB_DATA       = 0x14, // 20
		LOOPBACK_DATA_WRITE = 0x15, // 21
		LOOPBACK_DATA_READ  = 0x16, // 22
		SET_INTERFACE_DS    = 0x17, // 23
		SET_SEL             = 0x30, // 48
		SET_ISOCH_DELAY     = 0x31  // 49
	};

	enum class DESCRIPTOR_TYPE : uint8_t
	{
		// USB 2.0 Table 9-5
		DEVICE                        = 0x01,
		CONFIGURATION                 = 0x02,
		STRING                        = 0x03,
		INTERFACE                     = 0x04,
		ENDPOINT                      = 0x05,
		DEVICE_QUALIFIER              = 0x06,
		OTHER_SPEED_CONFIGURATION     = 0x07,
		INTERFACE_POWER               = 0x08, // See USB Interface Power Management Specification

		// USB 3.2 Revision 1.0 Table 9-6
		OTG                           = 0x09,
		DEBUG                         = 0x0A,
		INTERFACE_ASSOCIATION         = 0x0B,

		// CDC120-20101103 Table 12
		CLASS_SPECIFIC_INTERFACE      = 0x24,
		CLASS_SPECIFIC_ENDPOINT       = 0x25,

		// USB 3.2 Revision 1.0 Table 9-6
		BOS                           = 0x0F,
		DEVICE_CAPABILITY             = 0x10,
		SUPERSPEED_USB_ENDPOINT_COMPANION = 0x30,
		SUPERSPEEDPLUS_ISOCHRONOUS_ENDPOINT_COMPANION = 0x31
	};

	enum class DEVICE_CAPABILITY_CODE : uint8_t
	{
		// USB 3.2 Revision 1.0 Table 9-14
		WIRELESS_USB                = 0x01, // Defines the set of Wireless USB-specific device level capabilities
		USB_20_EXTENSION            = 0x02, // USB 2.0 Extension Descriptor
		SUPERSPEED_USB              = 0x03, // Defines the set of SuperSpeed USB specific device level capabilities
		CONTAINER_ID                = 0x04, // Defines the instance unique ID used to identify the instance across all operating modes
		PLATFORM                    = 0x05, // Defines a device capability specific to a particular platform/operating system
		POWER_DELIVERY_CAPABILITY   = 0x06, // Defines the various PD Capabilities of this device
		BATTERY_INFO_CAPABILITY     = 0x07, // Provides information on each battery supported by the device
		PD_CONSUMER_PORT_CAPABILITY = 0x08, // The consumer characteristics of a port on the device
		PD_PROVIDER_PORT_CAPABILITY = 0x09, // The provider characteristics of a port on the device
		SUPERSPEED_PLUS             = 0x0A, // Defines the set of SuperSpeed Plus USB specific device level capabilities
		PRECISION_TIME_MEASUREMENT  = 0x0B, // Precision Time Measurement (PTM) Capability Descriptor
		WIRELESS_USB_EXT            = 0x0C, // Defines the set of Wireless USB 1.1-specific device level capabilities
		BILLBOARD                   = 0x0D, // Billboard capability
		AUTHENTICATION              = 0x0E, // Authentication Capability Descriptor
		BILLBOARD_EX                = 0x0F, // Billboard Ex capability
		CONFIGURATION_SUMMARY       = 0x10  // Summarizes configuration information for a function implemented by the device
	};

	enum class CONTROL_EP_SIZE
	{
		USB10 = 8,
		USB11 = 64,
		USB20 = 64,
		USB30 = 512,
		USB31 = 512,
		USB32 = 512
	};

	enum class BULK_EP_SIZE
	{
		USB10 = 8,
		USB11 = 64,
		USB20 = 64,
		USB30 = 1024,
		USB31 = 1024,
		USB32 = 1024
	};

	typedef std::function<void(const USB_EVENTS event, const uint8_t ep)> Event_callback;

	static bool is_in_ep(const uint8_t ep)
	{
		return (ep & 0x80) != 0;
	}

	static bool is_out_ep(const uint8_t ep)
	{
		return (ep & 0x80) == 0;
	}

	static uint8_t get_ep_addr(const uint8_t ep)
	{
		return ep & 0x7F;
	}

	static constexpr uint16_t build_bcd(const uint8_t major, const uint8_t minor, const uint8_t subminor)
	{
		return USB_common_private_func::build_bcd(major, minor, subminor);
	}

	//basic USB HS device
	static constexpr uint16_t bcdUSB_200 = USB_common_private_func::build_bcd(2, 0, 0);

	//??? error in standard?
	static constexpr uint16_t bcdUSB_201 = USB_common_private_func::build_bcd(2, 0, 1);

	//USB HS device that supports GetDescriptor(BOS Descriptor)
	//Usually a USB 3.1 device that is running in HS mode
	static constexpr uint16_t bcdUSB_210 = USB_common_private_func::build_bcd(2, 1, 0);

	//Basic USB SuperSpeed device
	static constexpr uint16_t bcdUSB_300 = USB_common_private_func::build_bcd(3, 0, 0);

	//Enhanced USB SuperSpeed device
	static constexpr uint16_t bcdUSB_310 = USB_common_private_func::build_bcd(3, 1, 0);
};
