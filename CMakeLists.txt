add_library(usb_dev_cpp
	src/descriptor/Configuration_descriptor.cpp
	src/descriptor/Device_descriptor.cpp
	src/descriptor/Endpoint_descriptor.cpp
	src/descriptor/Interface_descriptor.cpp
	src/descriptor/String_descriptor.cpp

	src/class/cdc/cdc.cpp
	src/class/cdc/cdc_usb.cpp

	src/class/dfu/dfu.cpp

	src/driver/usb_driver_base.cpp
	src/driver/stm32/stm32_h7xx_otghs.cpp

	src/driver/cpu/Cortex_m7.cpp

	src/usb_core.cpp
	src/usb_std.cpp

	src/core/Setup_packet.cpp
	src/core/Control_request.cpp
	src/core/Request_type.cpp
	src/core/Get_descriptor.cpp
)

target_include_directories(usb_dev_cpp PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(usb_dev_cpp
	common_util
	freertos_cpp_util
	stm32h7xx_cmsis
)

if(${BUILD_USB_DEV_CPP_TESTS})
	add_library(usb_dev_cpp_tests
		tests/descriptor/Endpoint_descriptor_tests.cpp
	)

	target_link_libraries(usb_dev_cpp_tests
		usb_dev_cpp
		googletest
	)
endif(${BUILD_USB_DEV_CPP_TESTS})
