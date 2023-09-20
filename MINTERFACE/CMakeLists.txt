#----------------------------------------------------------------------
# Include project files 
#----------------------------------------------------------------------
# include all the Example project .cpp files
INCLUDE(${MINTERFACE_SOURCE_DIR}../CMakeSourceFileList.cmake)
# include all the Example project .h files
INCLUDE(${MINTERFACE_SOURCE_DIR}../CMakeHeaderFileList.cmake)


#----------------------------------------------------------------------
# Create rule to copy all the headers from src to include/ORFCV
#----------------------------------------------------------------------
# For each header, we create a rule
SET(HEADER2_IN_INCLUDE_DIR "")
SET(ADDED_SRC_DEPEND_PROPERTIES "")
FOREACH(header2 ${HEADER_MINTERFACE})
  GET_FILENAME_COMPONENT(headerName ${header2} NAME) 
  ADD_CUSTOM_COMMAND(
    OUTPUT ${MIXEDVISION_BINARY_DIR}/include/MINTERFACE/${headerName}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header2} ${MIXEDVISION_BINARY_DIR}/include/MINTERFACE/${headerName}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${header2}
    )
  SET(HEADER2_IN_INCLUDE_DIR ${HEADER2_IN_INCLUDE_DIR} ${MIXEDVISION_BINARY_DIR}/include/MINTERFACE/${headerName}
    )
ENDFOREACH(header2)





#----------------------------------------------------------------------
# Create a top level rule to copy all the headers from src to include/ORFCV
#----------------------------------------------------------------------
ADD_CUSTOM_TARGET(header2 ALL
  DEPENDS ${HEADER2_IN_INCLUDE_DIR}
  )


#----------------------------------------------------------------------
# build rule for the library
#----------------------------------------------------------------------
ADD_LIBRARY (${MINTERFACE_INTERN_LIBS} SHARED
  ${SRC_MINTERFACE} 
  ${HEADER_MINTERFACE}
  ) 

# create the headers in include/MINTERFACE before compiling the lib
ADD_DEPENDENCIES(${MINTERFACE_INTERN_LIBS} header2)

TARGET_LINK_LIBRARIES(${MINTERFACE_INTERN_LIBS} MGEOMETRY MIMGPROC MIO MPOSE MCALIB)

# Append the library version information to the library target
# properties. 
SET_TARGET_PROPERTIES(${MINTERFACE_INTERN_LIBS}
  PROPERTIES
  VERSION ${MINTERFACE_VERSION_MAJOR}.${MINTERFACE_VERSION_MINOR}
  SOVERSION ${MINTERFACE_VERSION_PATCH}
  )

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for the library
INSTALL_TARGETS(/lib ${MINTERFACE_INTERN_LIBS})
# install rule for all the headers
INSTALL_FILES(/include/MINTERFACE FILES ${HEADER_MINTERFACE})
