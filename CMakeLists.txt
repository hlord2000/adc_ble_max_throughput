cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(adc_ble_max_throughput)

target_sources(app PRIVATE 
			   src/main.c
			   src/adc_ble_max_throughput_service.c
)

target_include_directories(app PRIVATE include)
