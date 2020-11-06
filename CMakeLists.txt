# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(SUPPORTED_TARGETS esp32)
project(ESP32OlcbHub)

###############################################################################
# Switch from GNU++11 to GNU++14
#
# NOTE: C++14 would be preferred but IDF has dependencies on GNU extensions.
###############################################################################

string(REPLACE "-std=gnu++11" "-std=gnu++14" CXX_OPTIONS "${CXX_COMPILE_OPTIONS}")
idf_build_set_property(CXX_COMPILE_OPTIONS "${CXX_OPTIONS}" REPLACE)

###############################################################################
# Enable usage of std::stoi/stol/etc
###############################################################################

idf_build_set_property(COMPILE_DEFINITIONS "-D_GLIBCXX_USE_C99" APPEND)

###############################################################################
# Search for GZIP application
###############################################################################

FIND_PROGRAM(GZIP
  NAMES gzip
  PATHS /bin
        /usr/bin
        /usr/local/bin
)

if (NOT GZIP)
  message(FATAL_ERROR "Unable to find 'gzip' program")
endif()

###############################################################################
# Generate a compressed version of web content on-demand
###############################################################################
add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/web/index.html.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/web/index.html
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/web/index.html
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/web/index.html.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/web/cash.min.js.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/web/cash.min.js
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/web/cash.min.js
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/web/cash.min.js.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/web/milligram.min.css.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/web/milligram.min.css
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/web/milligram.min.css
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/web/milligram.min.css.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/web/normalize.min.css.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/web/normalize.min.css
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/web/normalize.min.css
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/web/normalize.min.css.gz")

###############################################################################
# Add web content to the binary
###############################################################################

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/web/index.html.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/web/cash.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/web/milligram.min.css.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/web/normalize.min.css.gz" BINARY)

###############################################################################
# Configuration validations
###############################################################################

if (NOT CONFIG_FREERTOS_HZ EQUAL 1000)
    message(FATAL_ERROR "FreeRTOS tick rate (hz) is required to be 1000.")
endif()

if (NOT CONFIG_PARTITION_TABLE_FILENAME STREQUAL "esp32olcbhub-partitions.csv")
    message(FATAL_ERROR "The custom partition table option is not enabled in menuconfig and is required for compilation.")
endif()

if (NOT CONFIG_PARTITION_TABLE_CUSTOM_FILENAME STREQUAL "esp32olcbhub-partitions.csv")
    message(FATAL_ERROR "The custom partition table option is not enabled in menuconfig and is required for compilation.")
endif()

if (NOT CONFIG_PARTITION_TABLE_CUSTOM)
    message(FATAL_ERROR "The custom partition table option is not enabled in menuconfig and is required for compilation.")
endif()

if (NOT CONFIG_LWIP_SO_RCVBUF)
    message(FATAL_ERROR "LwIP SO_RCVBUF is a required option in menuconfig.")
endif()

if (CONFIG_ESP_MAIN_TASK_STACK_SIZE LESS 8192)
    message(FATAL_ERROR "Main task stack size must be at least 8192 bytes.")
endif()