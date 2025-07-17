# Windows functions
function(automate_add_windows_files target_name source_file)
  add_executable(${target_name} ${source_file})
  target_compile_features(${target_name} PUBLIC cxx_std_17)
  target_link_libraries(${target_name} PRIVATE blast_rush Catch2::Catch2)

  # Set runtime output directories for each configuration
  set_target_properties(${target_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG       "${CMAKE_CURRENT_BINARY_DIR}/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE     "${CMAKE_CURRENT_BINARY_DIR}/Release"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_CURRENT_BINARY_DIR}/RelWithDebInfo"
  )
endfunction()

set(ASSIGNMENTS
  # Assignment 1 purely on Java, thus I skip.
  # Assignment 2 I built graphs.hpp.
  # Assignment 3 did not require any programming.
  
  
)

# Loop through and add examples
while(ASSIGNMENTS)
  list(POP_FRONT ASSIGNMENTS target_name)
  list(POP_FRONT ASSIGNMENTS source_file)
  automate_add_windows_files(${target_name} ${source_file})
endwhile()
