/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include <cstdint>
#include <cstddef>

class Buffer_adapter_base
{
public:

	Buffer_adapter_base()
	{
		clear();
	}

	Buffer_adapter_base(uint8_t* const buf_ptr, const size_t buf_max)
	{
		reset(buf_ptr, buf_max);
	}

	virtual ~Buffer_adapter_base()
	{

	}

	void clear()
	{
		m_buf_ptr  = nullptr;
		m_buf_max  = 0;
		m_buf_size = 0;
	}

	void reset()
	{
		m_buf_size  = 0;	
	}

	void reset(uint8_t* const buf, const size_t maxlen)
	{
		m_buf_ptr  = buf;
		m_buf_max  = maxlen;
		m_buf_size = 0;
	}


	size_t insert(const uint8_t buf)
	{
		return insert(&buf, 1);
	}

	size_t insert(const uint8_t* buf_ptr, const size_t len);

	uint8_t* data()
	{
		return m_buf_ptr;
	}
	const uint8_t* data() const
	{
		return m_buf_ptr;
	}

	bool full() const
	{
		return m_buf_size == m_buf_max;
	}
	bool empty() const
	{
		return m_buf_size == 0;
	}

	size_t size() const
	{
		return m_buf_size;
	}
	size_t capacity() const
	{
		return m_buf_max - m_buf_size;
	}
	size_t max_size() const
	{
		return m_buf_max;
	}

	void resize(const size_t len)
	{
		m_buf_size = len;
	}

protected:
	uint8_t* m_buf_ptr;
	size_t m_buf_max;
	size_t m_buf_size;
};

class Buffer_adapter : public Buffer_adapter_base
{
public:
	Buffer_adapter()
	{
		clear();
	}

	void clear()
	{
		Buffer_adapter_base::clear();
		curr_ptr = nullptr;
		rem_len  = 0;
	}

	void reset()
	{
		Buffer_adapter_base::reset();
		curr_ptr = m_buf_ptr;
		rem_len  = 0;	
	}

	void reset(uint8_t* const buf, const size_t maxlen)
	{
		Buffer_adapter_base::reset(buf, maxlen);
		curr_ptr = buf;
		rem_len  = 0;	
	}

	size_t insert(const uint8_t buf)
	{
		return insert(&buf, 1);
	}

	size_t insert(const uint8_t* buf, const size_t len)
	{
		const size_t num_inserted = Buffer_adapter_base::insert(buf, len);
		
		rem_len += num_inserted;

		return num_inserted;
	}

	//current position in buffer
	uint8_t* curr_ptr;

	//remaining length
	size_t   rem_len;
};