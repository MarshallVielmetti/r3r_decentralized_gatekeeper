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

# Add debug flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Add source files
file(GLOB SOURCES "src/*.cpp")

# Create library or executable
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Find OMPL library installed via Homebrew
set(OMPL_ROOT "/opt/homebrew/opt/ompl")
set(OMPL_INCLUDE_DIR "${OMPL_ROOT}/include/ompl-1.7")
set(OMPL_LIBRARIES "${OMPL_ROOT}/lib/libompl.dylib")

# Find Boost library installed via Homebrew (required by OMPL)
set(BOOST_ROOT "/opt/homebrew/opt/boost")
set(BOOST_INCLUDE_DIR "${BOOST_ROOT}/include")

# Find Eigen library installed via Homebrew (required by OMPL)
set(EIGEN_ROOT "/opt/homebrew/opt/eigen")
set(EIGEN_INCLUDE_DIR "${EIGEN_ROOT}/include/eigen3")

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OMPL_INCLUDE_DIR}
    ${BOOST_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}
)

# Link OMPL library
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})

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
        
        # Link against our library and its dependencies
        target_link_libraries(${TEST_NAME} ${PROJECT_NAME} ${OMPL_LIBRARIES})
        
        # Include directories for tests
        target_include_directories(${TEST_NAME} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${OMPL_INCLUDE_DIR}
            ${BOOST_INCLUDE_DIR}
            ${EIGEN_INCLUDE_DIR}
        )
        
        # Set the same C++ standard for tests
        set_target_properties(${TEST_NAME} PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
        )
        
        # Add debug flags for tests
        target_compile_options(${TEST_NAME} PRIVATE
            $<$<CONFIG:Debug>:-g -O0 -DDEBUG>
            $<$<CONFIG:Release>:-O3 -DNDEBUG>
        )
    endforeach()
endif()