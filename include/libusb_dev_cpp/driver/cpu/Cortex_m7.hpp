#pragma once

#include "libusb_dev_cpp/driver/CPU_base.hpp"

class Cortex_m7
{
public:
	static inline void instruction_sync() override
	{
		asm volatile(
			"isb SY\n"
			: /* no out */
			: /* no in */
			: "memory"
		);
	}
	static inline void data_sync() override
	{
		asm volatile(
			"dsb SY\n"
			: /* no out */
			: /* no in */
			: "memory"
		);
	}
	static inline void data_instruction_sync() override
	{
		asm volatile(
			"dsb SY\n"
			"isb SY\n"
			: /* no out */
			: /* no in */
			: "memory"
		);
	}
};
