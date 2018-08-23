cmake_minimum_required(VERSION 2.8.7)

if (CMAKE_CROSSCOMPILING)
    message(STATUS "Skipping unit tests, cross compiling")
else()
    add_executable(rtcm_test rtcm_decoder_tests.c rtcm_encoder.c)
    target_link_libraries(rtcm_test rtcm)
    add_custom_command(
        TARGET rtcm_test POST_BUILD
        COMMENT "Running unit tests"
        COMMAND rtcm_test
    )
endif()
