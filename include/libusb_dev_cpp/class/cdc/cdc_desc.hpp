#pragma once

#include "libusb_dev_cpp/class/cdc/cdc.hpp"

#include "libusb_dev_cpp/usb_common.hpp"

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

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
	constexpr static uint8_t bDescriptorSubType = static_cast<uint8_t>(DESCRIPTOR_TYPE_CDC::HEADER);
	uint16_t bcdCDC;
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

	constexpr static uint8_t bFunctionLength = 5;
	constexpr static uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::CLASS_SPECIFIC_INTERFACE);
	constexpr static uint8_t bDescriptorSubType = static_cast<uint8_t>(DESCRIPTOR_TYPE_CDC::CALL_MGMT);
	uint8_t bmCapabilities;
	uint8_t bDataInterface;
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
	constexpr static uint8_t bDescriptorSubType = static_cast<uint8_t>(DESCRIPTOR_TYPE_CDC::ACM);
	uint8_t bmCapabilities;	
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
	constexpr static uint8_t bDescriptorSubtype = static_cast<uint8_t>(DESCRIPTOR_TYPE_CDC::UNION);
	uint8_t bMasterInterface;
	uint8_t bSlaveInterface0;
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

class CDC_line_coding
{
public:
	uint32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;
};
