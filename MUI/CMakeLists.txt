#
# Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
# www: http://www.irisa.fr/lagadic
#
# Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
#
# SOURCE variable corresponds to the list of all the sources to build binaries.
# The generate binary comes by removing the .cpp extension to
# the source name.
#
# If you want to add/remove a source, modify here

#FIND_PACKAGE(QT4 REQUIRED)
#SET(QT_USE_QTOPENGL TRUE)

SET(CalibrateSystem_SOURCES 
		calibrateSystem.cpp

#		glwidget.cpp
)

SET(CalibrateSystem_HEADERSQT
#		glwidget.h
)

SET(CalibrateSystem_HEADERS
		../MCORE/commun.h
		../MINTERFACE/CInterface.h

#		glwidget.h
)


#QT4_WRAP_CPP(CalibrateSystem_HEADERS_MOC ${CalibrateSystem_HEADERSQT})

#INCLUDE(${QT_USE_FILE})

INCLUDE(${VISP_USE_FILE})
##INCLUDE(${OPENCV_USE_FILE})

ADD_DEFINITIONS(${VISP_DEFINITIONS})# ${QT_DEFINITIONS} ${OPENCV_DEFINITIONS})

ADD_EXECUTABLE(CalibrateSystem
		${CalibrateSystem_HEADERS}
#		${CalibrateSystem_HEADERS_MOC}
		${CalibrateSystem_SOURCES}
)

TARGET_LINK_LIBRARIES(CalibrateSystem
		${VISP_LIBRARIES}
#		${QT_LIBRARIES}
		${MINTERFACE_INTERN_LIBS}
		${MINTERFACE_EXTERN_LIBS}
)

#SET(CMAKE_EXE_LINKER_FLAGS -L/System/Library/Frameworks/OpenGL.framework/Libraries -lGL -lGLU)
