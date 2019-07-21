/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/class/cdc/cdc.hpp"

#include "libusb_dev_cpp/usb_common.hpp"

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

#include <tuple>

namespace CDC
{

class CDC_header_descriptor : public Descriptor_base
{
public:
	typedef std::array<uint8_t, 5> CDC_header_descriptor_array;

	bool serialize(CDC_header_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const override;

	size_t size() const override
	{
		return bFunctionLength;
	}

	constexpr static uint8_t bFunctionLength = 5;
	constexpr static uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::CLASS_SPECIFIC_INTERFACE);
	constexpr static uint8_t bDescriptorSubType = static_cast<uint8_t>(FUNC_DESCRIPTOR_TYPE::HEADER);
	uint16_t bcdCDC;

	static_assert(std::tuple_size<CDC_header_descriptor_array>::value == bFunctionLength);
};

class CDC_call_management_descriptor : public Descriptor_base
{
public:
	typedef std::array<uint8_t, 5> CDC_call_management_descriptor_array;

	bool serialize(CDC_call_management_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const override;

	size_t size() const override
	{
		return bFunctionLength;
	}

	enum class CALL_MGMT_CHANNEL
	{
		COMM_CLASS_IFACE,
		DATA_CLASS_IFACE
	};

	bool set_call_mgmt(const CALL_MGMT_CHANNEL ch)
	{
		bool ret = false;
		switch(ch)
		{
			case CALL_MGMT_CHANNEL::COMM_CLASS_IFACE:
			{
				bmCapabilities &= ~(0x02);
				ret = true;
				break;
			}
			case CALL_MGMT_CHANNEL::DATA_CLASS_IFACE:
			{
				bmCapabilities |= 0x02;
				ret = true;
				break;
			}
			default:
			{
				ret = false;
				break;
			}
		}
		return ret;
	}
	void set_self_call_mgmt_handle(const bool self)
	{
		if(self)
		{
			bmCapabilities |= 0x01;
		}
		else
		{
			bmCapabilities &= ~(0x01);
		}
	}

	constexpr static uint8_t bFunctionLength = 5;
	constexpr static uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::CLASS_SPECIFIC_INTERFACE);
	constexpr static uint8_t bDescriptorSubType = static_cast<uint8_t>(FUNC_DESCRIPTOR_TYPE::CALL_MGMT);
	uint8_t bmCapabilities;
	uint8_t bDataInterface;

	static_assert(std::tuple_size<CDC_call_management_descriptor_array>::value == bFunctionLength);
};

class CDC_acm_descriptor : public Descriptor_base
{
public:
	typedef std::array<uint8_t, 4> CDC_acm_descriptor_array;

	bool serialize(CDC_acm_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const override;

	size_t size() const override
	{
		return bFunctionLength;
	}

	constexpr static uint8_t bFunctionLength = 4;
	constexpr static uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::CLASS_SPECIFIC_INTERFACE);
	constexpr static uint8_t bDescriptorSubType = static_cast<uint8_t>(FUNC_DESCRIPTOR_TYPE::ACM);
	uint8_t bmCapabilities;

	static_assert(std::tuple_size<CDC_acm_descriptor_array>::value == bFunctionLength);
};

class CDC_union_descriptor : public Descriptor_base
{
public:
	typedef std::array<uint8_t, 5> CDC_union_descriptor_array;

	bool serialize(CDC_union_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const override;

	size_t size() const override
	{
		return bFunctionLength;
	}

	constexpr static uint8_t bFunctionLength = 5;
	constexpr static uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::CLASS_SPECIFIC_INTERFACE);
	constexpr static uint8_t bDescriptorSubtype = static_cast<uint8_t>(FUNC_DESCRIPTOR_TYPE::UNION);
	uint8_t bMasterInterface;
	uint8_t bSlaveInterface0;

	static_assert(std::tuple_size<CDC_union_descriptor_array>::value == bFunctionLength);
};

template<size_t DATALEN>
class CDC_notification
{
public:
	uint16_t bmRequestType;
	uint8_t  bNotificationType;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	std::array<uint8_t, DATALEN> Data;
};
}