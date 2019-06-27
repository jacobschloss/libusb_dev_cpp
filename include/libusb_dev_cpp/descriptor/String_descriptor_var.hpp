/**
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "common_util/Byte_util.hpp"

#include <array>

#include <cstdint>

class String_descriptor_base
{
public:

	bool serialize(std::vector<uint8_t>* const out_array) const
	{
		out_array.resize(get_byte_len());

		(*out_array)[0] = get_byte_len();
		(*out_array)[1] = bDescriptorType;

		for(size_t i = 0; i < m_buf.size(); i++)
		{
			(*out_array)[2+2*i]   = Byte_util::get_b0(m_buf[i]);
			(*out_array)[2+2*i+1] = Byte_util::get_b1(m_buf[i]);
		}

		return true;
	}

protected:
	
	uint8_t get_byte_len() const
	{
		return 2U + 2U*m_buf.size();
	}

	static constexpr uint8_t bDescriptorType = 0x03;
	std::vector<uint16_t> m_buf;
};

class String_descriptor_zero : public String_descriptor_base
{
public:

protected:
	
};

class String_descriptor_zero_enus : public String_descriptor_zero
{
public:

	String_descriptor_zero_enus()
	{
		m_buf.resize(1);
		m_buf[0] = 0x0409;
	}
};

class String_descriptor_n : public String_descriptor_base
{
public:
	//USB uses UTF-16-LE, so strings can only be this long
	constexpr static size_t STRMAX = 126U;

	//build from ascii
	String_descriptor_n(const char msg[])
	{
		assign(msg);
	}

	size_t assign(const char* str)
	{
		const size_t len = std::min(STRMAX, strlen(str));
		m_buf.resize(len);

		for(size_t i = 0; i < len; i++)
		{
			m_buf[i] = ascii_to_utf16le(str[i]);
		}	
	}

protected:

	static constexpr uint16_t ascii_to_utf16le(const uint8_t c)
	{
		return c;
	}
};
