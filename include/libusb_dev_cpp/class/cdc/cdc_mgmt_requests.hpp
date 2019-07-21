/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

#include <array>

#include <cstdint>

namespace CDC
{
class SET_CONTROL_LINE_STATE
{
	struct Control_signal
	{
		bool DTR() const
		{
			return wValue & 0x0001;
		}
		bool RTS() const
		{
			return wValue & 0x0002;
		}

		void set_DTR(const bool val)
		{
			if(val)
			{
				wValue |= 0x0001;	
			}
			else
			{
				wValue &= ~(0x0001);
			}
		}
		void set_RTS(const bool val)
		{
			if(val)
			{
				wValue |= 0x0002;	
			}
			else
			{
				wValue &= ~(0x0002);
			}
		}

		uint16_t wValue;
	};
};

class LINE_CODING
{
	enum CHAR_FORMAT : uint8_t
	{
		ONE_STOP = 0,
		ONEPTFIVE_STOP = 1,
		TWO_STOP = 2
	};
	enum PARITY_TYPE : uint8_t
	{
		NONE  = 0,
		ODD   = 1,
		EVEN  = 2,
		MARK  = 3,
		SPACE = 4
	};
	enum DATA_BITS : uint8_t
	{
		FIVE    = 5,
		SIX     = 6,
		SEVEN   = 7,
		EIGHT   = 8,
		SIXTEEN = 16
	};
	uint32_t dwDTERRate;//bits per second
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;

	typedef std::array<uint8_t, 7> Line_coding_array;

	bool serialize(Line_coding_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const;

	bool deserialize(const Line_coding_array& array);
};

class SET_LINE_CODING
{


};

class GET_LINE_CODING
{

};
}