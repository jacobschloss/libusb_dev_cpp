/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2020 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

#include "libusb_dev_cpp/core/usb_common.hpp"

// USB 3.2 Revision 1.0 Table 9-13
class Device_capability_descriptor : public Descriptor_base
{
public:
	// uint8_t bLength;
	static constexpr uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::DEVICE_CAPABILITY);
	// uint8_t bDevCapabilityType;
	//more data
};

// USB 3.2 Revision 1.0 Table 9-15
class Device_capability_usb20exension : public Device_capability_descriptor
{
public:
	typedef std::array<uint8_t, 7> Device_capability_usb20exension_array;

	bool serialize(Device_capability_usb20exension_array* const out_array) const;
	bool serialize(Buffer_adapter_tx* const out_array) const override;

	size_t size() const override
	{
		return bLength;
	}

	static constexpr uint8_t bLength = 7;
	// uint8_t bDescriptorType; // DESCRIPTOR_TYPE::DEVICE_CAPABILITY
	static constexpr uint8_t bDevCapabilityType = static_cast<uint8_t>(USB_common::DEVICE_CAPABILITY_CODE::USB_20_EXTENSION);
	
	enum class ATTRIBUTES : uint8_t
	{
		LPM   = 1U << 1
	};

	// b0 reserved
	// b1 LPM supported
	// b31..2 reserved
	uint32_t bmAttributes;
};

// USB 3.2 Revision 1.0 Table 9-16
class Device_capability_superspeed : public Device_capability_descriptor
{
public:
	typedef std::array<uint8_t, 10> Device_capability_superspeed_array;

	bool serialize(Device_capability_superspeed_array* const out_array) const;
	bool serialize(Buffer_adapter_tx* const out_array) const override;

	size_t size() const override
	{
		return bLength;
	}

	static constexpr uint8_t bLength = 10;
	// uint8_t bDescriptorType; // DESCRIPTOR_TYPE::DEVICE_CAPABILITY
	static constexpr uint8_t bDevCapabilityType = static_cast<uint8_t>(USB_common::DEVICE_CAPABILITY_CODE::SUPERSPEED_USB);
	
	enum class ATTRIBUTES : uint8_t
	{
		LTM   = 1U << 1
	};

	enum class SPEEDS_SUPPORTED : uint16_t
	{
		LOW_SPEED   = 1U << 0,
		FULL_SPEED  = 1U << 1,
		HIGH_SPEED  = 1U << 2,
		SUPER_SPEED = 1U << 3
	};

	// b0 reserved
	// b1 LTM capable
	// b7..2 reserved
	uint8_t bmAttributes;

	// b0 low-speed supported
	// b1 full-speed supported
	// b2 high-speed supported
	// b3 gen1-speed supported
	// b15..4 reserved
	uint16_t wSpeedsSupported;

	//lowest speed which all functionality is availible
	//use code from wSpeedsSupported
	uint8_t bFunctionalitySupport;

	// U1 Device Exit Latency. Worst-case latency to
	// transition from U1 to U0, assuming the latency is
	// limited only by the device and not the device’s link
	// partner.
	// 0x00 - 0x0A, in us
	uint8_t bU1DevExitLat;

	// 2 Device Exit Latency. Worst-case latency to
	// transition from U2 to U0, assuming the latency is
	// limited only by the device and not the device’s link
	// partner.
	// 0x0000 - 0x07FF, in us
	uint16_t wU2DevExitLat;
};

// USB 3.2 Revision 1.0 Table 9-17
class Device_capability_containerid : public Device_capability_descriptor
{
public:
	typedef std::array<uint8_t, 20> Device_capability_containerid_array;

	bool serialize(Device_capability_containerid_array* const out_array) const;
	bool serialize(Buffer_adapter_tx* const out_array) const override;

	size_t size() const override
	{
		return bLength;
	}

	static constexpr uint8_t bLength = 20;
	// uint8_t bDescriptorType; // DESCRIPTOR_TYPE::DEVICE_CAPABILITY
	static constexpr uint8_t bDevCapabilityType = static_cast<uint8_t>(USB_common::DEVICE_CAPABILITY_CODE::CONTAINER_ID);

	uint8_t bReserved;

	// This is a 128-bit number that is unique to a device
	// instance that is used to uniquely identify the device
	// instance across all modes of operation. This same
	// value may be provided over other technologies as
	// well to allow the host to identify the device
	// independent of means of connectivity.
	// Refer to IETF RFC 4122 for details on generation of
	// a UUID.
	std::array<uint8_t, 16> ContainerID;
};

// USB 3.2 Revision 1.0 Table 9-18
// class Device_capability_platform : public Device_capability_descriptor
// {
// public:
// 	bool serialize(Buffer_adapter_tx* const out_array) const override;

// 	size_t size() const override
// 	{
// 		return bLength;
// 	}

// 	static constexpr uint8_t bLength;
// 	// uint8_t bDescriptorType; // DESCRIPTOR_TYPE::DEVICE_CAPABILITY
// 	static constexpr uint8_t bDevCapabilityType = static_cast<uint8_t>(USB_common::DEVICE_CAPABILITY_CODE::PLATFORM);

// 	uint8_t bReserved;

// 	// This is a 128-bit number that uniquely identifies
// 	// a platform specific capability of the device.
// 	std::array<16, uint8_t> PlatformCapabilityUUID;
// 	// CapabilityData
// };
