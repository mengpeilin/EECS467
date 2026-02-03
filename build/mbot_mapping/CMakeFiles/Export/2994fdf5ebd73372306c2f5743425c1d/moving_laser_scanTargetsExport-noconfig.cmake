#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mbot_mapping::moving_laser_scan" for configuration ""
set_property(TARGET mbot_mapping::moving_laser_scan APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(mbot_mapping::moving_laser_scan PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmoving_laser_scan.a"
  )

list(APPEND _cmake_import_check_targets mbot_mapping::moving_laser_scan )
list(APPEND _cmake_import_check_files_for_mbot_mapping::moving_laser_scan "${_IMPORT_PREFIX}/lib/libmoving_laser_scan.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
