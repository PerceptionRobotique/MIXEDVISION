CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND CMAKE_POLICY)
PROJECT(MIXEDVISION)

#----------------------------------------------------------------------
# Third party libraries
#----------------------------------------------------------------------

# ViSP-2
FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

FIND_PACKAGE(LibXml2 REQUIRED)
IF (LibXml2_FOUND)
  INCLUDE_DIRECTORIES(${LIBXML2_INCLUDE_DIR})
ENDIF(LibXml2_FOUND)

#First, MGEOMETRY
#-----------------------------------------------------------------------------
# MGEOMETRY version number.  An even minor number corresponds to releases.
SET(MGEOMETRY_VERSION_MAJOR 1)
SET(MGEOMETRY_VERSION_MINOR 0)
SET(MGEOMETRY_VERSION_PATCH 0)
SET(MGEOMETRY_VERSION "${MGEOMETRY_VERSION_MAJOR}.${MGEOMETRY_VERSION_MINOR}")
SET(MGEOMETRY_VERSION_FULL "${MGEOMETRY_VERSION}.${MGEOMETRY_VERSION_PATCH}")      

SET(MGEOMETRY_INCLUDE_DIR "${MIXEDVISION_BINARY_DIR}/include/MIXEDVISION")
SET(MGEOMETRY_DOC_DIR "${MIXEDVISION_BINARY_DIR}/doc")

#-----------------------------------------------------------------------------

# The location in which to install Example libraries.
SET(LIBRARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/lib)

# The location in which to install ORFCV binaries.
SET(BINARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/bin)

# the include directory we depend on
SET(MGEOMETRY_INTERN_INCLUDE_DIR ${MIXEDVISION_BINARY_DIR}/include)
# the third party include directories we depend on
SET(MGEOMETRY_EXTERN_INCLUDE_DIR "")
# the internal visp libs we want to createn
SET(MGEOMETRY_INTERN_LIBS MGEOMETRY)
# the third party libraries we depend on
SET(MGEOMETRY_EXTERN_LIBS "")
# the third party libraries we depend on
SET(MGEOMETRY_DEFS "")

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

## where are user-specific cmake modules
## make_dist.sh relies on $ENV{CMAKE_MODULES_PATH} (TODO, FIXME)
SET(CMAKE_MODULE_PATH ${MGEOMETRY_SOURCE_DIR}/CMakeModules)

INCLUDE(CheckLibraryExists)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${MGEOMETRY_INCLUDE_DIR})

#---------------------------------------------------------------------
# Platform dependent 
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(MGEOMETRY_DEFS "${MGEOMETRY_DEFS} -DWIN32")
  SET(MGEOMETRY_DEFS "${MGEOMETRY_DEFS} -DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm"  getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    SET(MGEOMETRY_EXTERN_LIBS ${MGEOMETRY_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
  IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(ART_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
 IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(MBT_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
ENDIF(WIN32 AND NOT CYGWIN)
IF(UNIX)
  SET(MGEOMETRY_DEFS "${MGEOMETRY_DEFS} -DUNIX")
  IF(APPLE)
    # We will not use rpath support.  Tell CMake to skip it.
    SET(CMAKE_SKIP_RPATH 1 CACHE INTERNAL "Whether to build with rpath." FORCE)
    SET(MGEOMETRY_DEFS "${MGEOMETRY_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${MGEOMETRY_INTERN_INCLUDE_DIR} ${MGEOMETRY_EXTERN_INCLUDE_DIR})

#----------------------------------------------------------------------
# Add library path
#----------------------------------------------------------------------
LINK_DIRECTORIES(${MGEOMETRY_SOURCE_DIR}/lib)

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
ADD_DEFINITIONS(${MGEOMETRY_DEFS})

# Then (1), MIO
#-----------------------------------------------------------------------------
# MIO version number.  An even minor number corresponds to releases.
SET(MIO_VERSION_MAJOR 1)
SET(MIO_VERSION_MINOR 0)
SET(MIO_VERSION_PATCH 0)
SET(MIO_VERSION "${MIO_VERSION_MAJOR}.${MIO_VERSION_MINOR}")
SET(MIO_VERSION_FULL "${MIO_VERSION}.${MIO_VERSION_PATCH}")      

SET(MIO_INCLUDE_DIR "${MIXEDVISION_BINARY_DIR}/include/MIXEDVISION")
SET(MIO_DOC_DIR "${MIXEDVISION_BINARY_DIR}/doc")

#-----------------------------------------------------------------------------

# The location in which to install Example libraries.
SET(LIBRARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/lib)

# The location in which to install ORFCV binaries.
SET(BINARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/bin)

# the include directory we depend on
SET(MIO_INTERN_INCLUDE_DIR ${MIXEDVISION_BINARY_DIR}/include)
# the third party include directories we depend on
SET(MIO_EXTERN_INCLUDE_DIR "")
# the internal visp libs we want to createn
SET(MIO_INTERN_LIBS MIO)
# the third party libraries we depend on
SET(MIO_EXTERN_LIBS "")
# the third party libraries we depend on
SET(MIO_DEFS "")

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

## where are user-specific cmake modules
## make_dist.sh relies on $ENV{CMAKE_MODULES_PATH} (TODO, FIXME)
SET(CMAKE_MODULE_PATH ${MIO_SOURCE_DIR}/CMakeModules)

INCLUDE(CheckLibraryExists)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${MIO_INCLUDE_DIR})

#---------------------------------------------------------------------
# Platform dependent 
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(MIO_DEFS "${MIO_DEFS} -DWIN32")
  SET(MIO_DEFS "${MIO_DEFS} -DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm"  getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    SET(MIO_EXTERN_LIBS ${MIO_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
  IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(ART_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
 IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(MBT_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
ENDIF(WIN32 AND NOT CYGWIN)
IF(UNIX)
  SET(MIO_DEFS "${MIO_DEFS} -DUNIX")
  IF(APPLE)
    # We will not use rpath support.  Tell CMake to skip it.
    SET(CMAKE_SKIP_RPATH 1 CACHE INTERNAL "Whether to build with rpath." FORCE)
    SET(MIO_DEFS "${MIO_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${MIO_INTERN_INCLUDE_DIR} ${MIO_EXTERN_INCLUDE_DIR})

#----------------------------------------------------------------------
# Add library path
#----------------------------------------------------------------------
LINK_DIRECTORIES(${MIO_SOURCE_DIR}/lib)

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
ADD_DEFINITIONS(${MIO_DEFS})

# Then (2), MPOSE
#-----------------------------------------------------------------------------
# MPOSE version number.  An even minor number corresponds to releases.
SET(MPOSE_VERSION_MAJOR 1)
SET(MPOSE_VERSION_MINOR 0)
SET(MPOSE_VERSION_PATCH 0)
SET(MPOSE_VERSION "${MPOSE_VERSION_MAJOR}.${MPOSE_VERSION_MINOR}")
SET(MPOSE_VERSION_FULL "${MPOSE_VERSION}.${MPOSE_VERSION_PATCH}")      

SET(MPOSE_INCLUDE_DIR "${MIXEDVISION_BINARY_DIR}/include/MIXEDVISION")
SET(MPOSE_DOC_DIR "${MIXEDVISION_BINARY_DIR}/doc")

#-----------------------------------------------------------------------------

# The location in which to install Example libraries.
SET(LIBRARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/lib)

# The location in which to install ORFCV binaries.
SET(BINARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/bin)

# the include directory we depend on
SET(MPOSE_INTERN_INCLUDE_DIR ${MIXEDVISION_BINARY_DIR}/include)
# the third party include directories we depend on
SET(MPOSE_EXTERN_INCLUDE_DIR "")
# the internal visp libs we want to createn
SET(MPOSE_INTERN_LIBS MPOSE)
# the third party libraries we depend on
SET(MPOSE_EXTERN_LIBS "")
# the third party libraries we depend on
SET(MPOSE_DEFS "")

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

## where are user-specific cmake modules
## make_dist.sh relies on $ENV{CMAKE_MODULES_PATH} (TODO, FIXME)
SET(CMAKE_MODULE_PATH ${MPOSE_SOURCE_DIR}/CMakeModules)

INCLUDE(CheckLibraryExists)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${MPOSE_INCLUDE_DIR})

#---------------------------------------------------------------------
# Platform dependent 
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(MPOSE_DEFS "${MPOSE_DEFS} -DWIN32")
  SET(MPOSE_DEFS "${MPOSE_DEFS} -DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm"  getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    SET(MPOSE_EXTERN_LIBS ${MPOSE_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
  IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(ART_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
 IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(MBT_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
ENDIF(WIN32 AND NOT CYGWIN)
IF(UNIX)
  SET(MPOSE_DEFS "${MPOSE_DEFS} -DUNIX")
  IF(APPLE)
    # We will not use rpath support.  Tell CMake to skip it.
    SET(CMAKE_SKIP_RPATH 1 CACHE INTERNAL "Whether to build with rpath." FORCE)
    SET(MPOSE_DEFS "${MPOSE_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${MPOSE_INTERN_INCLUDE_DIR} ${MPOSE_EXTERN_INCLUDE_DIR})

#----------------------------------------------------------------------
# Add library path
#----------------------------------------------------------------------
LINK_DIRECTORIES(${MPOSE_SOURCE_DIR}/lib)

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
ADD_DEFINITIONS(${MPOSE_DEFS})

# Then (3), MCALIB
#-----------------------------------------------------------------------------
# MCALIB version number.  An even minor number corresponds to releases.
SET(MCALIB_VERSION_MAJOR 1)
SET(MCALIB_VERSION_MINOR 0)
SET(MCALIB_VERSION_PATCH 0)
SET(MCALIB_VERSION "${MCALIB_VERSION_MAJOR}.${MCALIB_VERSION_MINOR}")
SET(MCALIB_VERSION_FULL "${MCALIB_VERSION}.${MCALIB_VERSION_PATCH}")      

SET(MCALIB_INCLUDE_DIR "${MIXEDVISION_BINARY_DIR}/include/MIXEDVISION")
SET(MCALIB_DOC_DIR "${MIXEDVISION_BINARY_DIR}/doc")

#-----------------------------------------------------------------------------

# The location in which to install Example libraries.
SET(LIBRARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/lib)

# The location in which to install ORFCV binaries.
SET(BINARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/bin)

# the include directory we depend on
SET(MCALIB_INTERN_INCLUDE_DIR ${MIXEDVISION_BINARY_DIR}/include)
# the third party include directories we depend on
SET(MCALIB_EXTERN_INCLUDE_DIR "")
# the internal visp libs we want to createn
SET(MCALIB_INTERN_LIBS MCALIB)
# the third party libraries we depend on
SET(MCALIB_EXTERN_LIBS "")
# the third party libraries we depend on
SET(MCALIB_DEFS "")

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

## where are user-specific cmake modules
## make_dist.sh relies on $ENV{CMAKE_MODULES_PATH} (TODO, FIXME)
SET(CMAKE_MODULE_PATH ${MCALIB_SOURCE_DIR}/CMakeModules)

INCLUDE(CheckLibraryExists)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${MCALIB_INCLUDE_DIR})

#---------------------------------------------------------------------
# Platform dependent 
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(MCALIB_DEFS "${MCALIB_DEFS} -DWIN32")
  SET(MCALIB_DEFS "${MCALIB_DEFS} -DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm"  getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    SET(MCALIB_EXTERN_LIBS ${MCALIB_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
  IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(ART_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
 IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(MBT_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
ENDIF(WIN32 AND NOT CYGWIN)
IF(UNIX)
  SET(MCALIB_DEFS "${MCALIB_DEFS} -DUNIX")
  IF(APPLE)
    # We will not use rpath support.  Tell CMake to skip it.
    SET(CMAKE_SKIP_RPATH 1 CACHE INTERNAL "Whether to build with rpath." FORCE)
    SET(MCALIB_DEFS "${MCALIB_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${MCALIB_INTERN_INCLUDE_DIR} ${MCALIB_EXTERN_INCLUDE_DIR})

#----------------------------------------------------------------------
# Add library path
#----------------------------------------------------------------------
LINK_DIRECTORIES(${MCALIB_SOURCE_DIR}/lib)

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
ADD_DEFINITIONS(${MCALIB_DEFS})

# Then (3), MIMGPROC
#-----------------------------------------------------------------------------
# MIMGPROC version number.  An even minor number corresponds to releases.
SET(MIMGPROC_VERSION_MAJOR 1)
SET(MIMGPROC_VERSION_MINOR 0)
SET(MIMGPROC_VERSION_PATCH 0)
SET(MIMGPROC_VERSION "${MIMGPROC_VERSION_MAJOR}.${MIMGPROC_VERSION_MINOR}")
SET(MIMGPROC_VERSION_FULL "${MIMGPROC_VERSION}.${MIMGPROC_VERSION_PATCH}")      

SET(MIMGPROC_INCLUDE_DIR "${MIXEDVISION_BINARY_DIR}/include/MIXEDVISION")
SET(MIMGPROC_DOC_DIR "${MIXEDVISION_BINARY_DIR}/doc")

#-----------------------------------------------------------------------------

FIND_PACKAGE(GSL REQUIRED)

# The location in which to install Example libraries.
SET(LIBRARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/lib)

# The location in which to install ORFCV binaries.
SET(BINARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/bin)

# the include directory we depend on
SET(MIMGPROC_INTERN_INCLUDE_DIR ${MIXEDVISION_BINARY_DIR}/include)
# the third party include directories we depend on
SET(MIMGPROC_EXTERN_INCLUDE_DIR ${GSL_INCLUDE_DIRS})
# the internal visp libs we want to createn
SET(MIMGPROC_INTERN_LIBS MIMGPROC)
# the third party libraries we depend on
SET(MIMGPROC_EXTERN_LIBS ${GSL_LIBRARIES})
# the third party libraries we depend on
SET(MIMGPROC_DEFS "")

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

## where are user-specific cmake modules
## make_dist.sh relies on $ENV{CMAKE_MODULES_PATH} (TODO, FIXME)
SET(CMAKE_MODULE_PATH ${MIMGPROC_SOURCE_DIR}/CMakeModules)

INCLUDE(CheckLibraryExists)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${MIMGPROC_INCLUDE_DIR})

#---------------------------------------------------------------------
# Platform dependent 
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(MIMGPROC_DEFS "${MIMGPROC_DEFS} -DWIN32")
  SET(MIMGPROC_DEFS "${MIMGPROC_DEFS} -DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm"  getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    SET(MIMGPROC_EXTERN_LIBS ${MIMGPROC_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
  IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(ART_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
 IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(MBT_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
ENDIF(WIN32 AND NOT CYGWIN)
IF(UNIX)
  SET(MIMGPROC_DEFS "${MIMGPROC_DEFS} -DUNIX")
  IF(APPLE)
    # We will not use rpath support.  Tell CMake to skip it.
    SET(CMAKE_SKIP_RPATH 1 CACHE INTERNAL "Whether to build with rpath." FORCE)
    SET(MIMGPROC_DEFS "${MIMGPROC_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${MIMGPROC_INTERN_INCLUDE_DIR} ${MIMGPROC_EXTERN_INCLUDE_DIR})

#----------------------------------------------------------------------
# Add library path
#----------------------------------------------------------------------
LINK_DIRECTORIES(${MIMGPROC_SOURCE_DIR}/lib)

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
ADD_DEFINITIONS(${MIMGPROC_DEFS})

# Finally, MINTERFACE
#-----------------------------------------------------------------------------
# MINTERFACE version number.  An even minor number corresponds to releases.
SET(MINTERFACE_VERSION_MAJOR 1)
SET(MINTERFACE_VERSION_MINOR 0)
SET(MINTERFACE_VERSION_PATCH 0)
SET(MINTERFACE_VERSION "${MINTERFACE_VERSION_MAJOR}.${MINTERFACE_VERSION_MINOR}")
SET(MINTERFACE_VERSION_FULL "${MINTERFACE_VERSION}.${MINTERFACE_VERSION_PATCH}")      

SET(MINTERFACE_INCLUDE_DIR "${MIXEDVISION_BINARY_DIR}/include/MINTERFACE")
SET(MINTERFACE_DOC_DIR "${MIXEDVISION_BINARY_DIR}/doc")

#-----------------------------------------------------------------------------

# The location in which to install Example libraries.
SET(LIBRARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/lib)

# The location in which to install ORFCV binaries.
SET(BINARY_OUTPUT_PATH ${MIXEDVISION_BINARY_DIR}/bin)

# the include directory we depend on
SET(MINTERFACE_INTERN_INCLUDE_DIR ${MIXEDVISION_BINARY_DIR}/include)
# the third party include directories we depend on
SET(MINTERFACE_EXTERN_INCLUDE_DIR "")
# the internal visp libs we want to createn
SET(MINTERFACE_INTERN_LIBS MINTERFACE)
# the third party libraries we depend on
SET(MINTERFACE_EXTERN_LIBS "")
# the third party libraries we depend on
SET(MINTERFACE_DEFS "")

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

## where are user-specific cmake modules
## make_dist.sh relies on $ENV{CMAKE_MODULES_PATH} (TODO, FIXME)
SET(CMAKE_MODULE_PATH ${MINTERFACE_SOURCE_DIR}/CMakeModules)

INCLUDE(CheckLibraryExists)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${MINTERFACE_INCLUDE_DIR})



#---------------------------------------------------------------------
# Platform dependent 
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(MINTERFACE_DEFS "${MINTERFACE_DEFS} -DWIN32")
  SET(MINTERFACE_DEFS "${MINTERFACE_DEFS} -DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm"  getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    SET(MINTERFACE_EXTERN_LIBS ${MINTERFACE_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
  IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(ART_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
 IF(MSVC) #check if MSVC is the compiler and set ART_USE_MSVC accordingly
    SET(MBT_USE_MSVC   TRUE)  # for header mbtConfig.h
  ENDIF(MSVC)
ENDIF(WIN32 AND NOT CYGWIN)
IF(UNIX)
  SET(MINTERFACE_DEFS "${MINTERFACE_DEFS} -DUNIX")
  IF(APPLE)
    # We will not use rpath support.  Tell CMake to skip it.
    SET(CMAKE_SKIP_RPATH 1 CACHE INTERNAL "Whether to build with rpath." FORCE)
    SET(MINTERFACE_DEFS "${MINTERFACE_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

#----------------------------------------------------------------------
# Third party libraries
#----------------------------------------------------------------------

# ViSP-2
FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)
#LINK_DIRECTORIES(${VISP_LINK_DIRECTORIES})
#  ADD_DEFINITIONS(  -msse -msse2 -ffast-math -funroll-loops -mfpmath=sse -fomit-frame-pointer) #-Wall
#INCLUDE_DIRECTORIES(	${CMAKE_SOURCE_DIR}/src 	${VISP_INCLUDE_DIR}  )

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${MINTERFACE_INTERN_INCLUDE_DIR} ${MINTERFACE_EXTERN_INCLUDE_DIR})

#----------------------------------------------------------------------
# Add library path
#----------------------------------------------------------------------
LINK_DIRECTORIES(${MINTERFACE_SOURCE_DIR}/lib)


#----------------------------------------------------------------------
# add html-doc target for doxygen documentation building
#----------------------------------------------------------------------
#FIND_PACKAGE(Doxygen)
#IF (DOXYGEN_FOUND)  
#  ## we need latex for doxygen because of the formulas
#  FIND_PACKAGE(LATEX)
#  IF    (NOT LATEX_COMPILER)
#    MESSAGE(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user inetraction on doxy run.")
#  ENDIF (NOT LATEX_COMPILER)
#  IF    (NOT MAKEINDEX_COMPILER)
#    MESSAGE(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
#  ENDIF (NOT MAKEINDEX_COMPILER)
#  IF    (NOT DVIPS_CONVERTER)
#    MESSAGE(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
#  ENDIF (NOT DVIPS_CONVERTER)
#   
#  CONFIGURE_FILE(${MINTERFACE_SOURCE_DIR}/doc/config-doxygen.in
#    ${MINTERFACE_SOURCE_DIR}/doc/config-doxygen
#    @ONLY )
#  ADD_CUSTOM_TARGET(html-doc ${DOXYGEN_EXECUTABLE} ${MINTERFACE_SOURCE_DIR}/doc/config-doxygen) 
#ENDIF(DOXYGEN_FOUND)

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
ADD_DEFINITIONS(${MINTERFACE_DEFS})


#----------------------------------------------------------------------
# Propagation in sub dirs
#----------------------------------------------------------------------
SUBDIRS(MCORE)
SUBDIRS(MINTERFACE)
SUBDIRS(MUI)
#SUBDIRS(MUI-web)

