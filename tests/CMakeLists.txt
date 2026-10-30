set(CMAKE_PREFIX_PATH "/usr/local")
include(FetchContent)

# Find GoogleTest
find_package(GTest)
# fetch google test if not installed globally
if (NOT GTest_FOUND)
  message(STATUS "Could not find googletest, fetching.")
  FetchContent_Declare(
    GTest 
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2 # latest release
    )
    FetchContent_MakeAvailable(GTest)
endif()

# Find GoogleBenchmark
find_package(benchmark)
if (NOT benchmark_FOUND)
message(STATUS "Could not find google benchmark, fetching.")
FetchContent_Declare(
  benchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG v1.9.1 # Use the latest stable release
  )
FetchContent_MakeAvailable(benchmark)
endif()

file(GLOB SOURCES "src/*.cpp" )
# remove benchmark sources from the list
# list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/benchmark_tests.cpp")
# list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/path_conversion_benchmark.cpp")

# Create a unit test executable
add_executable(unit_tests ${SOURCES})

# add includes 
include_directories("inc")

# Link GoogleTest and the MAPF library
target_link_libraries(unit_tests PRIVATE MAPF_lib GTest::gtest_main GTest::gmock)

# Create a benchmark tests

# Create a benchmark for path conversion
add_executable(path_conv_bm src/benchmarks/path_conv_bm.cpp src/test_utils.cpp)
target_link_libraries(path_conv_bm PRIVATE MAPF_lib GTest::gtest_main GTest::gmock benchmark::benchmark benchmark::benchmark_main)

# Create a benchmark for vector operations
add_executable(vector_operations_bm src/benchmarks/vector_operations_bm.cpp src/test_utils.cpp)
target_link_libraries(vector_operations_bm PRIVATE MAPF_lib GTest::gtest_main GTest::gmock benchmark::benchmark benchmark::benchmark_main)

# Create benchmark for SIPP
add_executable(sipp_bm src/benchmarks/sipp_bm.cpp src/test_utils.cpp)
target_link_libraries(sipp_bm PRIVATE MAPF_lib GTest::gtest_main GTest::gmock benchmark::benchmark benchmark::benchmark_main)

# Enable CTest integration
add_test(NAME unit_tests COMMAND unit_tests)
