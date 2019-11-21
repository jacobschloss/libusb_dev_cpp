/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/descriptor/String_descriptor_base.hpp"

#include "common_util/Byte_util.hpp"

#include <cstring>

constexpr uint8_t String_descriptor_base::bDescriptorType;
constexpr size_t String_descriptor_base::str_len_max;

bool String_descriptor_base::serialize(Buffer_adapter_tx* const out_array) const
{
	const uint8_t len = size();
	out_array->insert(len);
	out_array->insert(bDescriptorType);
	
	std::array<uint8_t, 2> m_char;
	if(m_str)
	{
		for(size_t i = 0; i < str_len_max; i++)
		{
			const char c = m_str[i];
			if(c)
			{
				ascii_to_utf16le(c, &m_char);

				out_array->insert(m_char.data(), m_char.size());
			}
			else
			{
				break;
			}
		}
	}

	return true;
}

size_t String_descriptor_base::size() const
{;
	size_t len = 1+1;
	if(m_str)
	{
		for(size_t i = 0; i < str_len_max; i++)
		{
			if(m_str[i])
			{
				len += 2;
			}
			else
			{
				break;
			}
		}
	}
	return std::min<size_t>(len, 254U);
}



bool String_descriptor_zero::serialize(Buffer_adapter_tx* const out_array) const
{
	const uint8_t len = size();
	out_array->insert(len);
	out_array->insert(bDescriptorType);

	std::array<uint8_t, 2> u16;
	for(size_t i = 0; i < len; i++)
	{
		u16[0] = Byte_util::get_b0(static_cast<uint16_t>(m_lang[i]));
		u16[1] = Byte_util::get_b1(static_cast<uint16_t>(m_lang[i]));
		out_array->insert(u16.data(), u16.size());
	}

	return true;
}

size_t String_descriptor_zero::size() const
{
	return 1U + 1U + 2U*m_size;
}