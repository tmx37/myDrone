if(DEFINED LIB_PARENT_TARGET) # Use same target

  target_include_directories( ${LIB_PARENT_TARGET} PRIVATE include)

  target_sources( ${LIB_PARENT_TARGET} PRIVATE
      src/GenUtl.c)

else() # Create new target

  # add name of the library
  set(LIBRARY_NAME GENUTL)

  PROJECT(${LIBRARY_NAME})

  if (NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    message(STATUS "This project [${LIBRARY_NAME}] has a top-level one called [${CMAKE_PROJECT_NAME}]")
  else()
    message( SEND_ERROR "You are trying to compile this module as stand alone" )
  endif()

  # add the library source files
  add_library(${LIBRARY_NAME} STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/GenUtl.c
    # ...
  )

  # add private include directory if necessary
  target_include_directories(${LIBRARY_NAME}
    PRIVATE
    # ...
  )

  # add public include directory
  target_include_directories(${LIBRARY_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/module_libs
  )

  # add library with cfg include
  target_link_libraries(${LIBRARY_NAME} PUBLIC ${LIBRARY_NAME}_CFG)

endif()
