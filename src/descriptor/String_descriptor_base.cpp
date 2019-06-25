#include "libusb_dev_cpp/descriptor/String_descriptor_base.hpp"

#include <cstring>

bool String_descriptor_base::serialize(Buffer_adapter* const out_array) const
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
	return std::min(len, 254U);
}
