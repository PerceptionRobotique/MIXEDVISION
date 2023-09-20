# Set SRC_subdir variable to all the files we want to parse during
# the build process. 
# Don't forget to update SRC_ALL variable if you add/remove a SRC_subdir 
# variable
#
# If you add/remove a directory, modify here

set(SRC_MGEOMETRY
		CPoint.cpp
		CModelStereo.cpp
		CModel.cpp
		COmni.cpp
		CParaboloid.cpp
		CPerspective.cpp
)

set(SRC_MIMGPROC
		CRing.cpp
		CCorner.cpp
)

set(SRC_MIO
		CXml.cpp
		CModelXml.cpp
		CModelStereoXml.cpp
		CPerspectiveXml.cpp
		COmniXml.cpp
)

set(SRC_MPOSE
		CPose.cpp
		CPoseStereo.cpp
		CPoseOmni.cpp
		CPoseParaboloid.cpp
		CPosePerspective.cpp
)

set(SRC_MCALIB
		CCalibrationStereo.cpp
		CMire.cpp
		CCalibrationModel.cpp
		CCalibrationOmni.cpp
		CCalibrationParaboloid.cpp
		CCalibrationPerspective.cpp
  )

SET(SRC_MINTERFACE
		CInterface.cpp
)

SET (SRC_ALL 
  ${SRC_MGEOMETRY}
  ${SRC_MIMGPROC}
  ${SRC_MIO}
  ${SRC_MPOSE}
  ${SRC_MCALIB}
  ${SRC_MINTERFACE}
  )
