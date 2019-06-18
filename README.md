YAUS
	Yet Another USB Stack.

A portable USB device library, written in C++ for microcontrollers.

Assumptions that may guide design choices
	Your processor runs faster than memory, has data cache, heap is availible and fast
	Access to peripheral bus is slow

Design principles
	C++
	Event based
	Thread-safe
		Hooks for pipelined processors
	DMA

Features
	CDC Class
	DFU Class

Comparison to other USB stacks
	- Descriptors are held in heap and serialized when needed. If you want to change descriptors on the fly, you can.

License