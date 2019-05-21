#pragma once

class Cortex_m7
{
public:
	static inline void instruction_sync()
	{
		asm volatile(
			"isb SY\n"
			: /* no out */
			: /* no in */
			: "memory"
		);
	}
	static inline void data_sync()
	{
		asm volatile(
			"dsb SY\n"
			: /* no out */
			: /* no in */
			: "memory"
		);
	}
	static inline void data_instruction_sync()
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
