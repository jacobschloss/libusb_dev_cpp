/**
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/util/String_desc_table.hpp"

String_desc_table* Multilang_string_desc_table::get_table(const String_descriptor_zero::LANGID lang)
{
	auto& it = m_table[lang];

	return &(it);
}
const String_desc_table* Multilang_string_desc_table::get_table(const String_descriptor_zero::LANGID lang) const
{
	auto it = m_table.find(lang);

	if(it == m_table.end())
	{
		return nullptr;
	}

	return &(it->second);
}
