#include "libusb_dev_cpp/descriptor/String_descriptor_base.hpp"

#include <cstring>

bool String_descriptor_base::serialize(Buffer_adapter* const out_array) const
{
	const uint8_t len = size();
	out_array->insert(len);
	out_array->insert(bDescriptorType);
	
	std::array<uint8_t, 2> m_char;
	char const * m_curr = m_str;
	while(m_curr)
	{
		ascii_to_utf16le(*m_curr, &m_char);

		out_array->insert(m_char.data(), m_char.size());

		m_curr++;
	}

	return true;
}

size_t String_descriptor_base::size() const
{;
	size_t len = 1+1;
	if(m_str)
	{
		len += 2U * strlen(m_str);
	}
	return len;
}