cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Project's name
project(DescriptorApps)

# Enable or disable the use of OpenMP
set(useOpenMP FALSE)

# Colors defined for messages
if(NOT WIN32)
	string(ASCII 27 Esc)
	set(ColorReset	"${Esc}[m")
	set(Red 		"${Esc}[31m")
	set(Green 		"${Esc}[32m")
	set(Yellow 		"${Esc}[33m")
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
find_package(Boost 1.46 COMPONENTS unit_test_framework REQUIRED)

# Print information related to libraries
message(STATUS "${Yellow}PCL found version ${PCL_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tPCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}${ColorReset}")

message(STATUS "${Yellow}Found OpenCV version ${OpenCV_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tOpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}${ColorReset}")

message(STATUS "${Yellow}Found yaml-cpp version ${yaml-cpp_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tYAML_CPP_INCLUDE_DIR = ${YAML_CPP_INCLUDE_DIR}${ColorReset}")

message(STATUS "${Yellow}Found Boost version ${Boost_LIB_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tBoost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}${ColorReset}")

# Add relevant include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

# Add relevant linking directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Set the build type for PCL
set(PCL_BUILD_TYPE Release)

# Add extra required definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions("-Wall")

# Add the directory of the descriptor's libraries
add_subdirectory(descriptor_lib)

# Add the include directories of the descriptor's libraries
file(GLOB lib_directories descriptor_lib/*)
list(REMOVE_ITEM lib_directories
	${CMAKE_CURRENT_SOURCE_DIR}/descriptor_lib/.git
	${CMAKE_CURRENT_SOURCE_DIR}/descriptor_lib/test)
foreach(subdir ${lib_directories})
	if (IS_DIRECTORY ${subdir})
		include_directories(${subdir}/include)
	endif()
endforeach()

# Set the linking dependencies for all the applications
set(APPS_LINKING_DEPENDENCIES
	clustering
	descriptor
	factories
	io
	metrics
	utils
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${OpenCV_LIBS})

# Target Test
file(GLOB TEST_SRC "Test.cpp" "descriptor_lib/test/*.cpp")
add_executable(Test ${TEST_SRC})
set_target_properties(Test PROPERTIES COMPILE_DEFINITIONS "BOOST_TEST_DYN_LINK")
target_link_libraries(Test ${APPS_LINKING_DEPENDENCIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

# Target Descriptor 
file(GLOB DESCRIPTOR_SRC "Descriptor.cpp")
add_executable(Descriptor ${DESCRIPTOR_SRC})
target_link_libraries(Descriptor ${APPS_LINKING_DEPENDENCIES})

# Target DenseEvaluator 
file(GLOB DENSE_EVALUATOR_SRC "DenseEvaluator.cpp")
add_executable(DenseEvaluator ${DENSE_EVALUATOR_SRC})
target_link_libraries(DenseEvaluator ${APPS_LINKING_DEPENDENCIES})

# Target BoWCalculator 
file(GLOB BOW_CALCULATOR_SRC "BoWCalculator.cpp")
add_executable(BoWCalculator ${BOW_CALCULATOR_SRC})
target_link_libraries(BoWCalculator ${APPS_LINKING_DEPENDENCIES})

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach() 