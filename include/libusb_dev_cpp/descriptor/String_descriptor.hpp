/**
 * @brief String_descriptor
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "common_util/Byte_util.hpp"

#include <array>

#include <cstdint>

template<size_t NUM_LANG>
class String_descriptor_zero
{
public:

	typedef std::array<uint8_t, 2 + NUM_LANG*2> String_descriptor_zero_array;

	bool serialize(String_descriptor_zero_array* const out_array)
	{
		(*out_array)[0] = bLength;
		(*out_array)[1] = bDescriptorType;

		for(size_t i = 0; i < NUM_LANG; i++)
		{
			(*out_array)[2+2*i]   = Byte_util::get_b0(wLANGID[i]);
			(*out_array)[2+2*i+1] = Byte_util::get_b1(wLANGID[i]);
		}

		return true;
	}
	//bool deserialize(const String_descriptor_zero_array& array)
	//{
	//	
	//}

protected:
	static constexpr uint8_t bLength = 2 + NUM_LANG*2;
	static constexpr uint8_t bDescriptorType = 0x03;
	std::array<uint16_t, NUM_LANG> wLANGID;
};

class String_descriptor_zero_enus : public String_descriptor_zero<1>
{
public:

	String_descriptor_zero_enus()
	{
		wLANGID[0] = 0x0409;
	}
};

template<size_t STRLEN>
class String_descriptor_n
{
public:
	//USB uses UTF-16-LE, so strings can only be this long
	static_assert(STRLEN <= 126);

	String_descriptor_n()
	{
		bLength = 2;
	}

	//build from ascii
	String_descriptor_n(const char msg[STRLEN])
	{
		for(size_t i = 0; i < STRLEN; i++)
		{
			if(msg[i] == 0)
			{
				bLength = 2+i*2;
				break;
			}

			bString[i] = ascii_to_utf16le(msg[i]);
		}

		bLength = STRLEN+2;
	}

	typedef std::array<uint8_t, 2 + STRLEN*2> String_descriptor_array;

	bool serialize(String_descriptor_array* const out_array)
	{
		if(bLength < 2)
		{
			bLength = 2;
		}

		if(bLength > out_array->size())
		{
			return false;
		}

		(*out_array)[0] = bLength;
		(*out_array)[1] = bDescriptorType;

		for(size_t i = 0; i < (bLength - 2); i++)
		{
			(*out_array)[2+2*i]   = Byte_util::get_b0(bString[i]);
			(*out_array)[2+2*i+1] = Byte_util::get_b1(bString[i]);
		}

		return true;
	}
	// bool deserialize(const String_descriptor_array& array)
	// {
	// 
	// }


protected:

	static constexpr uint16_t ascii_to_utf16le(const uint8_t c)
	{
		return c;
	}

	uint8_t bLength;
	static constexpr uint8_t bDescriptorType = 0x03;
	std::array<uint16_t, STRLEN> bString;
};
