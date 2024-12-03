#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "my_model" for configuration ""
set_property(TARGET my_model APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(my_model PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/my_model/libmy_model.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS my_model )
list(APPEND _IMPORT_CHECK_FILES_FOR_my_model "${_IMPORT_PREFIX}/lib/my_model/libmy_model.a" )

# Import target "RVO" for configuration ""
set_property(TARGET RVO APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(RVO PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/my_model/libRVO.so"
  IMPORTED_SONAME_NOCONFIG "libRVO.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS RVO )
list(APPEND _IMPORT_CHECK_FILES_FOR_RVO "${_IMPORT_PREFIX}/lib/my_model/libRVO.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
