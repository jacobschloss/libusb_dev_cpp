/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

#include "common_util/Intrusive_list.hpp"

#include <array>

#include <cstdint>

class Configuration_descriptor : public Descriptor_base
{
public:

	Configuration_descriptor()
	{

	}

	Configuration_descriptor(const Configuration_descriptor& rhs) = delete;
	Configuration_descriptor& operator=(const Configuration_descriptor& rhs) = delete;

	enum class ATTRIBUTES
	{
		NONE          = 0x00,
		SELF_POWERED  = 0xC0,
		REMOTE_WAKEUP = 0xA0
	};

	typedef std::array<uint8_t, 9> Configuration_descriptor_array;

	bool serialize(Configuration_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter_tx* const out_array) const override;

	bool deserialize(const Configuration_descriptor_array& array);

	size_t size() const override
	{
		return bLength;
	}

	static constexpr uint8_t ma_to_maxpower(const uint8_t ma)
	{
		return (ma + 1U) / 2U;
	}

	static constexpr uint8_t bLength = 9;
	static constexpr uint8_t bDescriptorType = 0x02;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;

	size_t get_total_size() const
	{
		size_t total_size = size();
		
		Descriptor_base const * desc_node = get_desc_list().front<Descriptor_base>();
		while(desc_node)
		{
			total_size += desc_node->size();

			desc_node = desc_node->next<Descriptor_base>();
		}

		return total_size;
	}

	Intrusive_list& get_desc_list()
	{
		return m_desc_list;
	}

	const Intrusive_list& get_desc_list() const
	{
		return m_desc_list;
	}

protected:
	Intrusive_list m_desc_list;
};
