#pragma once

#include <cstdint>
#include <cstddef>

class Buffer_adapter
{
public:
	Buffer_adapter()
	{
		clear();
	}

	void clear()
	{
		buf_ptr     = nullptr;
		buf_maxsize = 0;
		curr_ptr    = nullptr;
		rem_len     = 0;
	}

	void reset()
	{
		curr_ptr    = buf_ptr;
		rem_len     = 0;	
	}

	void reset(uint8_t* const buf, const size_t maxlen)
	{
		buf_ptr     = buf;
		buf_maxsize = maxlen;
		curr_ptr    = buf;
		rem_len     = 0;	
	}

	size_t insert(const uint8_t buf);
	size_t insert(const uint8_t* buf, const size_t len);

	size_t size() const
	{
		return rem_len;
	}

	size_t capacity() const
	{
		return buf_maxsize - rem_len;
	}

	bool full() const
	{
		return rem_len == buf_maxsize;
	}

	bool empty() const
	{
		return rem_len == 0;
	}

	//start of buffer
	uint8_t* buf_ptr;

	//max length
	size_t   buf_maxsize;

	//current position in buffer
	uint8_t* curr_ptr;

	//remaining length
	size_t   rem_len;
};