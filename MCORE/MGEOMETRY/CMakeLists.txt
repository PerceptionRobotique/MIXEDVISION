#----------------------------------------------------------------------
# Include project files 
#----------------------------------------------------------------------
# include all the Example project .cpp files
INCLUDE(${MGEOMETRY_SOURCE_DIR}../../CMakeSourceFileList.cmake)
# include all the Example project .h files
INCLUDE(${MGEOMETRY_SOURCE_DIR}../../CMakeHeaderFileList.cmake)

#----------------------------------------------------------------------
# Create rule to copy all the headers from src to include/ORFCV
#----------------------------------------------------------------------
# For each header, we create a rule
SET(HEADER_IN_INCLUDE_DIR "")
SET(ADDED_SRC_DEPEND_PROPERTIES "")
FOREACH(header ${HEADER_MGEOMETRY})
  GET_FILENAME_COMPONENT(headerName ${header} NAME) 

  ADD_CUSTOM_COMMAND(
    OUTPUT ${MIXEDVISION_BINARY_DIR}/include/MIXEDVISION/${headerName}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header} ${MIXEDVISION_BINARY_DIR}/include/MIXEDVISION/${headerName}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${header}
    )
  SET(HEADER_IN_INCLUDE_DIR ${HEADER_IN_INCLUDE_DIR} ${MIXEDVISION_BINARY_DIR}/include/MIXEDVISION/${headerName}
    )
ENDFOREACH(header)





#----------------------------------------------------------------------
# Create a top level rule to copy all the headers from src to include/ORFCV
#----------------------------------------------------------------------
ADD_CUSTOM_TARGET(mgheader ALL
  DEPENDS ${HEADER_IN_INCLUDE_DIR}
  )


#----------------------------------------------------------------------
# build rule for the library
#----------------------------------------------------------------------
ADD_LIBRARY (${MGEOMETRY_INTERN_LIBS} SHARED
  ${SRC_MGEOMETRY} 
  ${HEADER_MGEOMETRY}
  ) 

# create the headers in include/ORFCV before compiling the lib
ADD_DEPENDENCIES(${MGEOMETRY_INTERN_LIBS} mgheader)

# Append the library version information to the library target
# properties. 
SET_TARGET_PROPERTIES(${MGEOMETRY_INTERN_LIBS}
  PROPERTIES
  VERSION ${MGEOMETRY_VERSION_MAJOR}.${MGEOMETRY_VERSION_MINOR}
  SOVERSION ${MGEOMETRY_VERSION_PATCH}
  )

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for the library
INSTALL_TARGETS(/lib ${MGEOMETRY_INTERN_LIBS})
# install rule for all the headers
INSTALL_FILES(/include/MIXEDVISION FILES ${HEADER_MGEOMETRY})
