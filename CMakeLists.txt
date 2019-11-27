add_library(usb_dev_cpp
	src/descriptor/Configuration_descriptor.cpp
	src/descriptor/Device_descriptor.cpp
	src/descriptor/Endpoint_descriptor.cpp
	src/descriptor/Interface_descriptor.cpp
	src/descriptor/String_descriptor_base.cpp

	src/class/usb_class.cpp

	src/class/cdc/cdc.cpp
	src/class/cdc/cdc_desc.cpp
	src/class/cdc/cdc_mgmt_requests.cpp
	src/class/cdc/cdc_notification.cpp


	src/class/dfu/dfu.cpp

	src/driver/usb_driver_base.cpp

	src/core/Setup_packet.cpp
	src/core/Request_type.cpp
	src/core/Get_descriptor.cpp
	src/core/Notification_packet.cpp

	src/util/Buffer_adapter.cpp
	src/util/Desc_table_base.cpp
	src/util/Config_desc_table.cpp
	src/util/Iface_desc_table.cpp
	src/util/String_desc_table.cpp
	src/util/Descriptor_table.cpp

	src/util/EP_buffer_array.cpp
	src/util/EP_buffer_mgr_base.cpp
)

add_library(usb_dev_cpp_stm32

	src/driver/cpu/Cortex_m7.cpp

	src/class/cdc/cdc_usb.cpp
	src/driver/stm32/stm32_h7xx_otghs.cpp
	src/usb_core.cpp

	src/util/EP_buffer_mgr_freertos.cpp
)

target_include_directories(usb_dev_cpp PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(usb_dev_cpp
	common_util
	freertos_cpp_util
)

target_link_libraries(usb_dev_cpp_stm32
	usb_dev_cpp

	stm32h750xx_cmsis
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
