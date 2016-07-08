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

	project(tess2)

## Sugar

  hunter_add_package(Sugar)
  include("${SUGAR_ROOT}/cmake/Sugar")
  include(sugar_include)

## Sources

	file(GLOB SRCS "Source/*.cpp" "Source/*.c" "Source/*.h" "Source/*.hpp")
	include_directories("Include")

## Library

	add_library(tess2 STATIC ${SRCS} ${SRCS_INCL})
