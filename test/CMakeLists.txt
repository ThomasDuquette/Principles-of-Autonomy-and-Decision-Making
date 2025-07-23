include(FetchContent)
FetchContent_Declare(
  catch
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v2.13.10)
FetchContent_MakeAvailable(catch)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${CMAKE_SOURCE_DIR}/test/CMake/Modules/)
include(ParseAndAddCatchTests)

# Define a function to automate adding benchamrks
function(automate_add_benchmarks target_name source_file)
  add_executable(${target_name} ${source_file})
  target_compile_features(${target_name} PUBLIC cxx_std_17)
  target_link_libraries(${target_name} PRIVATE Catch2::Catch2 blast_rush)

  # Set runtime output directories for each configuration
  set_target_properties(${target_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG       "${CMAKE_CURRENT_BINARY_DIR}/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE     "${CMAKE_CURRENT_BINARY_DIR}/Release"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_CURRENT_BINARY_DIR}/RelWithDebInfo"
  )
endfunction()

# Define a function to automate adding tests
function(automate_add_tests target_name source_file)
  add_executable(${target_name} ${source_file})
  target_compile_features(${target_name} PUBLIC cxx_std_17)
  target_link_libraries(${target_name} PRIVATE Catch2::Catch2 blast_rush)
  add_test(NAME ${target_name}_test COMMAND ${target_name})

  # Set runtime output directories for each configuration
  set_target_properties(${target_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG       "${CMAKE_CURRENT_BINARY_DIR}/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE     "${CMAKE_CURRENT_BINARY_DIR}/Release"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_CURRENT_BINARY_DIR}/RelWithDebInfo"
  )
endfunction()

# note: Add new benchmark files as follows :
# NAME_OF_BENCHMARK BENCHMARK_TARGET_LOCATION
# List of benchmarks with their source files
set(BENCHMARKS
  bench_A_star bench_A_star.cpp
)

set(TESTS
  assn04 assn04.cpp
  test_A_star A_star.cpp
  test_task task.cpp
)

# Loop through and add benchmarks
while(BENCHMARKS)
  list(POP_FRONT BENCHMARKS target_name)
  list(POP_FRONT BENCHMARKS source_file)
  automate_add_benchmarks(${target_name} ${source_file})
endwhile()

# Loop through and add tests
while(TESTS)
  list(POP_FRONT TESTS target_name)
  list(POP_FRONT TESTS source_file)
  automate_add_tests(${target_name} ${source_file})
endwhile()