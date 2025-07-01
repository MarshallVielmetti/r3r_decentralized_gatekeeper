cmake_minimum_required(VERSION 3.10)
project(ompl_src)

# Export compile commands for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Add option to control AddressSanitizer (disable for Julia compatibility)
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)

# Add debug flags with optional sanitizers
if(ENABLE_ASAN)
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG -fsanitize=address")
else()
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Add source files
file(GLOB SOURCES "src/*.cpp")

# Create library or executable
add_library(${PROJECT_NAME} SHARED ${SOURCES})

option(USE_LOCAL_OMPL "Use OMPL source instead of installed" ON)

if(USE_LOCAL_OMPL)
        set(OMPL_LOCAL_PATH "../../../ompl" CACHE PATH "Path to local OMPL source directory")

        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${OMPL_LOCAL_PATH}/CMakeLists.txt")
            message(FATAL_ERROR "OMPL source directory not found at ${OMPL_LOCAL_PATH}. Please check the path.")
        endif()

        message(STATUS "Using local OMPL source at ${OMPL_LOCAL_PATH}")

        # Prevent OMPL from installing
        set(OMPL_REGISTRATION OFF CACHE BOOL "Disable OMPL registration" FORCE)
        set(OMPL_BUILD_DEMOS OFF CACHE BOOL "Disable OMPL demos" FORCE)
        set(OMPL_BUILD_TESTS OFF CACHE BOOL "Disable OMPL tests" FORCE)

        add_subdirectory(${OMPL_LOCAL_PATH} ${CMAKE_CURRENT_BINARY_DIR}/ompl_build)

        set(OMPL_TARGET ompl)
else()
    find_package(ompl REQUIRED)
    set(OMPL_TARGET ompl)
endif()

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link OMPL library
# The add_subdirectory command should create an 'ompl' target.
# We link it as PUBLIC so that targets that link against ompl_src (like our tests)
# also get the OMPL headers and link against the ompl library.
target_link_libraries(${PROJECT_NAME} PUBLIC ompl)

# Add sanitizer flags to linker for debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND ENABLE_ASAN)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address)
endif()

# Test configuration
option(BUILD_TESTS "Build test executables" ON)

if(BUILD_TESTS)
    # Find all test source files
    file(GLOB TEST_SOURCES "test/*.cpp")
    
    # Create test executables
    foreach(TEST_SOURCE ${TEST_SOURCES})
        # Get the filename without extension
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
        
        # Create executable for this test
        add_executable(${TEST_NAME} ${TEST_SOURCE})
        
        # Link against our library. Because we linked ompl_src to ompl as PUBLIC,
        # CMake will automatically handle linking the test executable to ompl as well.
        target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
        
        # Include directories for tests are now inherited from the main library target.
        # No need to specify them again here.
        
        # Set the same C++ standard for tests
        set_target_properties(${TEST_NAME} PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
        )
        
        # Add debug flags for tests
        if(ENABLE_ASAN)
            target_compile_options(${TEST_NAME} PRIVATE
                $<$<CONFIG:Debug>:-g -O0 -DDEBUG -fsanitize=address>
                $<$<CONFIG:Release>:-O3 -DNDEBUG>
            )
        else()
            target_compile_options(${TEST_NAME} PRIVATE
                $<$<CONFIG:Debug>:-g -O0 -DDEBUG>
                $<$<CONFIG:Release>:-O3 -DNDEBUG>
            )
        endif()
        
        # Add sanitizer link flags for debug builds
        if(ENABLE_ASAN)
            target_link_options(${TEST_NAME} PRIVATE
                $<$<CONFIG:Debug>:-fsanitize=address>
            )
        endif()
    endforeach()
endif()