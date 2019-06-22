#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

class CDC_header_descriptor : public Descriptor_base
{
	typedef std::array<uint8_t, 5> CDC_header_descriptor_array;

	bool serialize(CDC_header_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const override;

	size_t size() const override
	{
		return bFunctionLength;
	}

	constexpr static uint8_t bFunctionLength = 5;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubType;
	uint16_t bcdCDC;
};

class CDC_call_management_descriptor : public Descriptor_base
{
	typedef std::array<uint8_t, 5> CDC_call_management_descriptor_array;

	bool serialize(CDC_call_management_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const override;

	size_t size() const override
	{
		return bFunctionLength;
	}

	constexpr static uint8_t bFunctionLength = 5;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubType;
	uint8_t bmCapabilities;
	uint8_t bDataInterface;
};

class CDC_acm_descriptor : public Descriptor_base
{
	typedef std::array<uint8_t, 5> CDC_acm_descriptor_array;

	bool serialize(CDC_acm_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const override;

	size_t size() const override
	{
		return bFunctionLength;
	}

	constexpr static uint8_t bFunctionLength = 4;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubType;
	uint8_t bmCapabilities;	
};

class CDC_union_descriptor : public Descriptor_base
{
	typedef std::array<uint8_t, 5> CDC_union_descriptor_array;

	bool serialize(CDC_union_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const override;

	size_t size() const override
	{
		return bFunctionLength;
	}

	constexpr static uint8_t bFunctionLength = 5;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bMasterInterface;
	uint8_t bSlaveInterface0;
};

template<size_t DATALEN>
class CDC_notification
{
	uint16_t bmRequestType;
	uint8_t  bNotificationType;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	std::array<uint8_t, DATALEN> Data;
};

class CDC_line_coding
{
	uint32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;
};
