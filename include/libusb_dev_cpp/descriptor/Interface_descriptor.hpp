/**
 * @brief Interface_descriptor
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "freertos_cpp_util/util/Intrusive_list.hpp"

#include <array>

#include <cstdint>

class Interface_descriptor : public Intrusive_list_node
{
public:

	Interface_descriptor()
	{

	}

	Interface_descriptor(const Interface_descriptor& rhs) = delete;
	Interface_descriptor& operator=(const Interface_descriptor& rhs) = delete;
	
	typedef std::array<uint8_t, 9> Interface_descriptor_array;

	bool serialize(Interface_descriptor_array* const out_array) const;
	bool deserialize(const Interface_descriptor_array& array);

	static constexpr uint8_t bLength = 9;
	static constexpr uint8_t bDescriptorType = 0x04;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;

	Intrusive_list& get_ep_desc_list()
	{
		return m_ep_desc_list;
	}

	const Intrusive_list& get_ep_desc_list() const
	{
		return m_ep_desc_list;
	}
protected:
	Intrusive_list m_ep_desc_list;
};
