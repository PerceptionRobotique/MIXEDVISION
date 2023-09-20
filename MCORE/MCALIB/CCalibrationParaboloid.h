#ifndef AFX_CCalibrationParaboloid
#define AFX_CCalibrationParaboloid

#include <MIXEDVISION/CCalibrationModel.h>
#include <MIXEDVISION/CParaboloid.h>

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CCalibrationParaboloid : public CCalibrationModel
{
public: 
	CCalibrationParaboloid(CParaboloid *cam = NULL);
	~CCalibrationParaboloid();

	void computePoseJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsr);
	void computeIntrinsicJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsi);

	void updateCameraParameters(CModel *cam, vpColVector & Tc_cam);
	
	void setCamera(CModel *cam);
};


#endif
