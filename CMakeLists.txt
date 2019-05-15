add_library(usb_dev_cpp
	src/descriptor/Configuration_descriptor.cpp
	src/descriptor/Device_descriptor.cpp
	src/descriptor/Endpoint_descriptor.cpp
	src/descriptor/Interface_descriptor.cpp
	src/descriptor/String_descriptor.cpp

	src/driver/usb_driver_base.cpp
	src/driver/stm32/stm32_h7xx_otghs.cpp
	src/usb_core.cpp
)

target_include_directories(usb_dev_cpp PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)