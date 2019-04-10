set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
set(include_install_dir "${CMAKE_INSTALL_INCLUDEDIR}")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets.cmake")
set(namespace "${PROJECT_NAME}::")

add_library(rtcm
  decode.c
  encode.c
  msm_utils.c
  eph_decode.c
  ssr_decode.c
  bits.c
  logging.c
  )
add_library(${namespace}rtcm ALIAS rtcm)

target_include_directories(rtcm 
    PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    )
target_link_libraries(rtcm m)

if(MSVC)
else()
target_compile_options(rtcm PRIVATE "-Wall")
target_compile_options(rtcm PRIVATE "-Wextra")
target_compile_options(rtcm PRIVATE "-Werror")
target_compile_options(rtcm PRIVATE "-Wimplicit")
target_compile_options(rtcm PRIVATE "-Wshadow")
target_compile_options(rtcm PRIVATE "-Wswitch-default")
target_compile_options(rtcm PRIVATE "-Wswitch-enum")
target_compile_options(rtcm PRIVATE "-Wundef")
target_compile_options(rtcm PRIVATE "-Wuninitialized")
target_compile_options(rtcm PRIVATE "-Wpointer-arith")
target_compile_options(rtcm PRIVATE "-Wcast-align")
target_compile_options(rtcm PRIVATE "-Wformat=2")
target_compile_options(rtcm PRIVATE "-Wimplicit-function-declaration")
target_compile_options(rtcm PRIVATE "-Wredundant-decls")
target_compile_options(rtcm PRIVATE "-Wformat-security")
target_compile_options(rtcm PRIVATE "-fno-unwind-tables")
target_compile_options(rtcm PRIVATE "-fno-asynchronous-unwind-tables")
target_compile_options(rtcm PRIVATE "-std=gnu99")
target_compile_options(rtcm PRIVATE "-fPIC")
# require at least gcc 5.0
if (CMAKE_C_COMPILER_VERSION VERSION_GREATER 5.0)
   target_compile_options(rtcm PRIVATE "-Wfloat-conversion")
endif()
endif()

include(CMakePackageConfigHelpers)

write_basic_package_version_file("${version_config}" COMPATIBILITY SameMajorVersion)

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
    )

install(
    TARGETS rtcm
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${include_install_dir}"
    )

install(
    DIRECTORY "${PROJECT_SOURCE_DIR}/include/rtcm3"
    DESTINATION "${include_install_dir}"
    )

install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
    )

install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
    )
