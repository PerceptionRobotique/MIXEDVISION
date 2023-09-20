#----------------------------------------------------------------------
# Include project files 
#----------------------------------------------------------------------
# include all the Example project .h files
INCLUDE(${MIXEDVISION_SOURCE_DIR}/CMakeHeaderFileList.cmake)

#----------------------------------------------------------------------
# Create rule to copy all the headers from src to include/ORFCV
#----------------------------------------------------------------------
# For each header, we create a rule
SET(HEADER_IN_INCLUDE_DIR "")
FOREACH(header ${HEADER_MIXEDVISION})
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
ADD_CUSTOM_TARGET(header ALL
  DEPENDS ${HEADER_IN_INCLUDE_DIR}
  )


# create the headers in include/ORFCV before compiling the lib
#ADD_DEPENDENCIES(${MIXEDVISION_INTERN_LIBS} header)

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for all the headers
INSTALL_FILES(/include/MIXEDVISION FILES ${HEADER_MIXEDVISION})

SUBDIRS(MGEOMETRY)
SUBDIRS(MIMGPROC)
SUBDIRS(MIO)
SUBDIRS(MPOSE)
SUBDIRS(MCALIB)
