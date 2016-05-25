cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Project's name
project(DescriptorApps)

# Colors defined for messages
if(NOT WIN32)
	string(ASCII 27 Esc)
	set(ColorReset	"${Esc}[m")
	set(Red 		"${Esc}[31m")
	set(Green 		"${Esc}[32m")
	set(Blue 		"${Esc}[34m")
	set(Magenta 	"${Esc}[35m")
	set(Cyan 		"${Esc}[36m")
endif()

# Add additional path for cmake to find OpenCV
set(CMAKE_PREFIX_PATH "/opt/ros/hydro/" ${CMAKE_PREFIX_PATH})

# Find required packages
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED)
find_package(yaml-cpp 0.5.2 REQUIRED)
# find_package(Boost 1.46 COMPONENTS unit_test_framework REQUIRED)
# find_package(OpenMP REQUIRED)

# Print information related to OpenCV version
message(STATUS "${Cyan}Found OpenCV version ${OpenCV_VERSION}${ColorReset}")
message(STATUS "${Cyan}OpenCV_INCLUDE_DIRS= ${OpenCV_INCLUDE_DIRS}${ColorReset}")

# Add relevant include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
# include_directories(${Boost_INCLUDE_DIRS})

# Add relevant linking directories
link_directories(${PCL_LIBRARY_DIRS})
# link_directories(${Boost_LIBRARY_DIRS})

# Set the build type for PCL
set(PCL_BUILD_TYPE Release)

# Add extra required definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions("-Wall")

# Add the directory of the descriptor's libraries
add_subdirectory(descriptor_lib)

# Add the include directories of the descriptor's libraries
file(GLOB lib_folders ./descriptor_lib/*)
foreach(subdir ${lib_folders})
	if (IS_DIRECTORY ${subdir})
		include_directories(${subdir}/include)
	endif()
endforeach()

# Define files for each target
file(GLOB DESCRIPTOR_SRC "Descriptor.cpp")

# Set OpenMP flags so Descriptor can be linked
# set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# Target Descriptor
add_executable(Descriptor ${DESCRIPTOR_SRC})
target_link_libraries(Descriptor
			clustering
			descriptor
			factories
			io
			metrics
			utils
			${PCL_LIBRARIES}
			${YAML_CPP_LIBRARIES}
			${OpenCV_LIBS})
