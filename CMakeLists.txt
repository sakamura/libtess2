#
# Modified By Luc Michalski - 2016
#

## Cmake

    cmake_minimum_required(VERSION 3.5)
    if(POLICY CMP0025)
      # detect Apple's Clang
      cmake_policy(SET CMP0025 NEW)
    endif()
    if(POLICY CMP0054)
      cmake_policy(SET CMP0054 NEW)
    endif()

## Hunter

   include("cmake/HunterGate.cmake")
   HunterGate(
       URL "https://github.com/ruslo/hunter/archive/v0.14.26.tar.gz"
       SHA1 "45ac03944db2bc7fc55dfceca42d89ec7fa1dca2"
   )

## Project / Version
    SET(PROJECT_NAME "tess2")
    SET(PROJECT_VERSION "1.0.1")
	project(${PROJECT_NAME} VERSION ${PROJECT_VERSION})

## Sugar

  hunter_add_package(Sugar)
  include("${SUGAR_ROOT}/cmake/Sugar")
  include(sugar_include)

## Compilation Flags

    # compile in release with debug info mode by default
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
    endif()

## Options

    # Build all binaries in a separate directory
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

    #add extra search paths for libraries and includes
    SET(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "The directory the headers are installed in")
    SET(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE STRING "Directory where lib will install")
    SET(DOC_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/doc/${PROJECT_NAME}" CACHE PATH "Path to the documentation")

## Sources

	file(GLOB SRCS "Source/*.cpp" "Source/*.c" "Source/*.h" "Source/*.hpp")
	include_directories("Include")

## Library

	add_library(${PROJECT_NAME} STATIC ${SRCS} ${SRCS_INCL})

#### install 

    set(config_install_dir "lib/cmake/${PROJECT_NAME}")
    set(include_install_dir "include")

    file(GLOB SRC "Source/*.cpp" "Source/*.c")
    file(GLOB HDRS "Source/*.h" "Source/*.hpp")
    set(TARGET_PUBLIC_HDRS ${HDRS})

    set(config_install_dir "lib/cmake/${PROJECT_NAME}")
    set(include_install_dir "include")

    set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

    # Configuration
    set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
    set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
    set(targets_export_name "${PROJECT_NAME}Targets")
    set(namespace "${PROJECT_NAME}::")

    include(CMakePackageConfigHelpers)

    write_basic_package_version_file(
        "${version_config}" COMPATIBILITY SameMajorVersion
    )

    configure_package_config_file(
        "cmake/Config.cmake.in"
        "${project_config}"
        INSTALL_DESTINATION "${config_install_dir}"
    )

    install(
        TARGETS ${PROJECT_NAME}
        EXPORT "${targets_export_name}"
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        INCLUDES DESTINATION "${include_install_dir}"
    )

    foreach(HEADER ${TARGET_PUBLIC_HDRS})
        MESSAGE(STATUS "  Header being processed: ${HEADER}")        
        install(
            FILES "${HEADER}"
            #FILES "${CMAKE_CURRENT_SOURCE_DIR}/${HEADER}"
            DESTINATION "${include_install_dir}/${PROJECT_NAME}"
        )
    endforeach()

    install(
        FILES "${project_config}" "${version_config}"
        DESTINATION "${config_install_dir}"
    )

    install(
        EXPORT "${targets_export_name}"
        NAMESPACE "${namespace}"
        DESTINATION "${config_install_dir}"
    )
