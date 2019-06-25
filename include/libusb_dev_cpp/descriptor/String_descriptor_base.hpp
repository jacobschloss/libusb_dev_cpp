#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

class String_descriptor_base : public Descriptor_base
{
public:

	String_descriptor_base()
	{
		m_str = nullptr;
	}

	bool serialize(Buffer_adapter* const out_array) const override;

	size_t size() const override;

	void assign(const char* str)
	{
		m_str = str;
	}

	enum class LANGID : uint16_t
	{
		ENUS = 0x0409
	};

private:
	
	static void ascii_to_utf16le(const uint8_t c, std::array<uint8_t, 2>* const buf)
	{
		(*buf)[0] = c;
		(*buf)[1] = 0;
	}

	static constexpr uint8_t bDescriptorType = 0x03;
	static constexpr size_t str_len_max = 126;
	char const * m_str;
};
