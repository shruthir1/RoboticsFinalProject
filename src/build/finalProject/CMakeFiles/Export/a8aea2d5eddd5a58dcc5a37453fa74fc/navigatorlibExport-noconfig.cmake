#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "finalProject::navigator" for configuration ""
set_property(TARGET finalProject::navigator APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(finalProject::navigator PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libnavigator.a"
  )

list(APPEND _cmake_import_check_targets finalProject::navigator )
list(APPEND _cmake_import_check_files_for_finalProject::navigator "${_IMPORT_PREFIX}/lib/libnavigator.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
