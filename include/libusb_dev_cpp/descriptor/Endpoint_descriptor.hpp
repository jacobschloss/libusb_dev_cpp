/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/core/usb_common.hpp"
#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

#include <array>

#include <cstdint>

class Endpoint_descriptor : public Descriptor_base
{
public:

	enum class ATTRIBUTE_TRANSFER
	{
		CONTROL     = 0x00,
		ISOCHRONOUS = 0x01,
		BULK        = 0x02,
		INTERRUPT   = 0x03
	};

	enum class ATTRIBUTE_INTERRUPT
	{
		PERIODIC     = 0X00,
		NOTIFICATION = 0X01
	};

	//For Isoc
	enum class ATTRIBUTE_SYNCHRONIZATION_TYPE
	{
		NONE         = 0x00,
		ASYNCHRONOUS = 0x01,
		ADAPTIVE     = 0x02,
		SYNCHRONOUS  = 0x03
	};

	//For Isoc
	enum class ATTRIBUTE_USAGE_TYPE
	{
		DATA              = 0x00,
		FEEDBACK          = 0x01,
		IMPLICIT_FEEDBACK = 0x02
	};

	typedef std::array<uint8_t, 7> Endpoint_descriptor_array;

	bool serialize(Endpoint_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter_tx* const out_array) const override;
	bool deserialize(const Endpoint_descriptor_array& array);

	size_t size() const override
	{
		return bLength;
	}

	ATTRIBUTE_TRANSFER get_ATTRIBUTE_TRANSFER() const
	{
		return static_cast<ATTRIBUTE_TRANSFER>(bmAttributes & 0x03);
	}

	static constexpr uint8_t bLength = 7;
	static constexpr uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::ENDPOINT);
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};

// USB 3.2 Revision 1.0 Table 9-27
class Endpoint_companion_descriptor : public Descriptor_base
{
public:
	typedef std::array<uint8_t, 6> Endpoint_companion_descriptor_array;

	bool serialize(Endpoint_companion_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter_tx* const out_array) const override;

	size_t size() const override
	{
		return bLength;
	}

	static constexpr uint8_t bLength = 6;
	static constexpr uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::SUPERSPEED_USB_ENDPOINT_COMPANION);
	//number of packets per burst, 0-15. 0 means 1 packet, 15 means 16 packets
	//control endpoints this must be zero
	uint8_t bMaxBurst;
	//bulk
	//4:0 Max Streams
	//7:5 reserved, set to 0
	//control & interrupt
	//7:0 reserved, set to 0
	//isochronous
	//1:0 Mult
	//6:2 Reserved, set to 0
	//7 SSP ISO Companion, a SSP ISO Companion descriptor follows this descriptor
	uint8_t bmAttributes;
	//total number of bytes this endpoint will transfer every service interval
	//for periodic endpoints
	//	The total number of bytes this endpoint will transfer
	//	every service interval (SI). This field is only valid
	//	for periodic endpoints.
	//	For isochronous endpoints:
	//	If the SSP ISO Companion bit in the bmAttributes
	//	field is set to zero, this value is used to reserve the
	//	bus time in the schedule, required for the frame data
	//	payloads per SI. The pipe may, on an ongoing basis,
	//	actually use less bandwidth than that reserved. The
	//	device reports, if necessary, the actual bandwidth
	//	used via its normal, non-USB defined mechanisms.
	//	If the SSP ISO Companion bit in the bmAttributes
	//	field is set to one, this field shall be set to one, and
	//	the total number of bytes this endpoint will transfer
	//	shall be reported via the endpoint’s SuperSpeedPlus
	//	Isochronous Endpoint Companion descriptor.
	//	wBytesPerInterval is reserved and must be set to
	//	zero for control and bulk endpoints.
	uint16_t wBytesPerInterval;
};

class SSP_Isoc_companion_descriptor : public Descriptor_base
{
	typedef std::array<uint8_t, 8> SSP_Isoc_companion_descriptor_array;

	bool serialize(SSP_Isoc_companion_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter_tx* const out_array) const override;

	size_t size() const override
	{
		return bLength;
	}

	static constexpr uint8_t bLength = 8;
	static constexpr uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::SUPERSPEEDPLUS_ISOCHRONOUS_ENDPOINT_COMPANION);
	uint16_t wReserved;
	uint32_t dwBytesPerInterval;
};
