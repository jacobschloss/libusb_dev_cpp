#include "libusb_dev_cpp/core/Setup_packet.hpp"

#include "common_util/Byte_util.hpp"

bool Setup_packet::serialize(Setup_packet_array* const out_array)
{
	if(!request_type.serialize(&(*out_array)[0]))
	{
		return false;
	}

	(*out_array)[1] = request;

	(*out_array)[2] = Byte_util::get_b0(value);
	(*out_array)[3] = Byte_util::get_b1(value);

	(*out_array)[4] = Byte_util::get_b0(index);
	(*out_array)[5] = Byte_util::get_b1(index);

	(*out_array)[6] = Byte_util::get_b0(count);
	(*out_array)[7] = Byte_util::get_b1(count);
}
bool Setup_packet::deserialize(const Setup_packet_array& in_array)
{
	if(!request_type.deserialize(in_array[0]))
	{
		return false;
	}

	request = in_array[1];

	value = Byte_util::make_u16(in_array[3], in_array[2]);
	index = Byte_util::make_u16(in_array[5], in_array[4]);
	count = Byte_util::make_u16(in_array[7], in_array[6]);
}