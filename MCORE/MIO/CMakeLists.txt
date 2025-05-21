#----------------------------------------------------------------------
# Include project files 
#----------------------------------------------------------------------
# include all the Example project .cpp files
INCLUDE(${MIO_SOURCE_DIR}../../CMakeSourceFileList.cmake)
# include all the Example project .h files
INCLUDE(${MIO_SOURCE_DIR}../../CMakeHeaderFileList.cmake)

#----------------------------------------------------------------------
# Create rule to copy all the headers from src to include/ORFCV
#----------------------------------------------------------------------
# For each header, we create a rule
SET(HEADER_IN_INCLUDE_DIR "")
SET(ADDED_SRC_DEPEND_PROPERTIES "")
FOREACH(header ${HEADER_MIO})
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
ADD_CUSTOM_TARGET(mioheader ALL
  DEPENDS ${HEADER_IN_INCLUDE_DIR}
  )


#----------------------------------------------------------------------
# build rule for the library
#----------------------------------------------------------------------
ADD_LIBRARY (${MIO_INTERN_LIBS} SHARED
  ${SRC_MIO} 
  ${HEADER_MIO}
  ) 

# create the headers in include/ORFCV before compiling the lib
ADD_DEPENDENCIES(${MIO_INTERN_LIBS} mioheader MGEOMETRY)

TARGET_LINK_LIBRARIES(${MIO_INTERN_LIBS} MGEOMETRY)

# Append the library version information to the library target
# properties. 
SET_TARGET_PROPERTIES(${MIO_INTERN_LIBS}
  PROPERTIES
  VERSION ${MIO_VERSION_MAJOR}.${MIO_VERSION_MINOR}
  SOVERSION ${MIO_VERSION_PATCH}
  )

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for the library
INSTALL_TARGETS(/lib ${MIO_INTERN_LIBS})
# install rule for all the headers
INSTALL_FILES(/include/MIXEDVISION FILES ${HEADER_MIO})
