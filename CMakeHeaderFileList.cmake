# Set HEADER_subdir variable to all the files we want to parse during
# the build process. 
# Don't forget to update HEADER_ALL variable if you add/remove a 
# HEADER_subdir variable
#
# If you add/remove a directory, modify here

set(HEADER_MIXEDVISION
		commun.h
)

set(HEADER_MGEOMETRY
		CPoint.h
		CModelStereo.h
		CModel.h
		COmni.h
		CParaboloid.h
		CPerspective.h
)

set(HEADER_MIMGPROC
		CRing.h
		CCorner.h
)

set(HEADER_MIO
		CXml.h
		CModelXml.h
		CModelStereoXml.h
		CPerspectiveXml.h
		COmniXml.h
)

set(HEADER_MPOSE
		CPose.h
		CPoseStereo.h
		CPoseOmni.h
		CPoseParaboloid.h
		CPosePerspective.h
)

set(HEADER_MCALIB
		CMire.h
		CCalibrationStereo.h
		CCalibrationModel.h
		CCalibrationOmni.h
		CCalibrationParaboloid.h
		CCalibrationPerspective.h
)

SET(HEADER_MINTERFACE
		CInterface.h
)

SET (HEADER_ALL 
  ${HEADER_MIXEDVISION}
  ${HEADER_MGEOMETRY}
  ${HEADER_MIMGPROC}
  ${HEADER_MIO}
  ${HEADER_MPOSE}
  ${HEADER_MCALIB}
  ${HEADER_MINTERFACE}
  )
