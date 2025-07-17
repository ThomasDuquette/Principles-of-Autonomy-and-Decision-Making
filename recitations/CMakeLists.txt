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

set(RECITATIONS
  # Currently done recitation 4, and still no recitation has needed any programming. I suspect recitations are done in class on the blackboard.
)

# Loop through and add examples
while(RECITATIONS)
  list(POP_FRONT RECITATIONS target_name)
  list(POP_FRONT RECITATIONS source_file)
  automate_add_windows_files(${target_name} ${source_file})
endwhile()
