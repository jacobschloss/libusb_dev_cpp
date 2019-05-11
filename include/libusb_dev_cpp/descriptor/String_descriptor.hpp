#pragma once

#include <array>

#include <cstdint>

template<size_t NUM_LANG>
class String_descriptor_zero
{
public:

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
	}

protected:

	static constexpr uint16_t ascii_to_utf16le(const uint8_t c)
	{
		return c;
	}

	uint8_t bLength;
	static constexpr uint8_t bDescriptorType = 0x03;
	std::array<uint16_t, STRLEN> bString;
};
