#
# Modified By Luc Michalski - 2016
#

cmake_minimum_required(VERSION 3.5)

## Hunter

   include("cmake/HunterGate.cmake")
   HunterGate(
       URL "https://github.com/ruslo/hunter/archive/v0.14.26.tar.gz"
       SHA1 "45ac03944db2bc7fc55dfceca42d89ec7fa1dca2"
   )

## Project / Version

	project(tess2 VERSION 1.0.1)

## Sugar

  hunter_add_package(Sugar)
  include("${SUGAR_ROOT}/cmake/Sugar")
  include(sugar_include)

## Sources

	file(GLOB SRCS "Source/*.cpp" "Source/*.c" "Source/*.h" "Source/*.hpp")
	include_directories("Include")

## Library

	add_library(tess2 STATIC ${SRCS} ${SRCS_INCL})

### Install ###
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

set(config_install_dir "lib/cmake/${PROJECT_NAME}")
set(include_install_dir "include")

set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(targets_export_name "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${version_config}" COMPATIBILITY SameMajorVersion
)

# Note: use 'targets_export_name'
configure_package_config_file(
    "cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
)

install(
    TARGETS tess2
    EXPORT "${targets_export_name}"
    COMPONENT dev
    INCLUDES DESTINATION "${include_install_dir}"
)

install(
    DIRECTORY "include/rapidjson"
    DESTINATION "${include_install_dir}"
    COMPONENT dev
)

install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
    COMPONENT dev
)

install(
    EXPORT "${targets_export_name}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
    COMPONENT dev
)
