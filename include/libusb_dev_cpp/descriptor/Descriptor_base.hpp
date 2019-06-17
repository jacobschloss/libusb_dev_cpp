#pragma once

#include <memory>

class Descriptor_base
{
public:
	virtual ~Descriptor_base()
	{

	}
};

typedef std::shared_ptr<Descriptor_base> Desc_base_ptr;
typedef std::shared_ptr<const Descriptor_base> Desc_base_const_ptr;
