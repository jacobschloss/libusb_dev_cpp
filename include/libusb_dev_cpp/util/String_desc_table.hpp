/**
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/descriptor/String_descriptor_base.hpp"

#include "libusb_dev_cpp/util/Desc_table_base.hpp"

#include <string>

class String_desc_table : public Desc_table_base<String_descriptor_base>
{
public:

	using String_desc_ptr = Desc_ptr;
	using String_desc_const_ptr = Desc_const_ptr;

private:
	// std::map<uint16_t, std::map<uint8_t, String_descriptor_n> > m_map;
};