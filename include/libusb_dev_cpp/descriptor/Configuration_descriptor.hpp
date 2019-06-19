/**
 * @brief Configuration_descriptor
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "freertos_cpp_util/util/Intrusive_list.hpp"

#include <array>

#include <cstdint>

class Configuration_descriptor
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
	bool deserialize(const Configuration_descriptor_array& array);

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

	Intrusive_list& get_iface_desc_list()
	{
		return m_iface_desc_list;
	}

	const Intrusive_list& get_iface_desc_list() const
	{
		return m_iface_desc_list;
	}

protected:
	Intrusive_list m_iface_desc_list;
};
