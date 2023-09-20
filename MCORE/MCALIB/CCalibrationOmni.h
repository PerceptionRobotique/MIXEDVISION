#ifndef AFX_CCalibrationOmni
#define AFX_CCalibrationOmni

#include <MIXEDVISION/CCalibrationModel.h>
#include <MIXEDVISION/COmni.h>

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CCalibrationOmni : public CCalibrationModel
{
public: 
	CCalibrationOmni(COmni *cam = NULL);
	~CCalibrationOmni();

	void computePoseJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsr);
	void computeIntrinsicJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsi);

	void updateCameraParameters(CModel *cam, vpColVector & Tc_cam);

	void setCamera(CModel *cam);
	
};


#endif
