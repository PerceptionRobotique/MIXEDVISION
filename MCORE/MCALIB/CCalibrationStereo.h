#ifndef AFX_CCalibrationStereo
#define AFX_CCalibrationStereo

#include <MIXEDVISION/CModelStereo.h>
#include <MIXEDVISION/CCalibrationModel.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpColor.h>

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CCalibrationStereo
{
public:
	CModelStereo systeme;
	vpHomogeneousMatrix sMo; //Matrice de changement de repère objet -> systeme stereo

	CCalibrationModel **calibcams; //Pour le calibrage individuel de chaque camera
	unsigned int nbcams;
    std::vector<bool> *activeExtrinsicParameters;
    unsigned int *nbActiveExtrinsicParameters;

	unsigned int npt ;       //!< number of points used in calibration computation
	double residual ;		   //!< residual in pixel for camera model without distortion

	static double threshold;
	static unsigned int nbIterMax;
	static double gain; 
	static double mu;
	static double muRef;

	CCalibrationStereo(unsigned int nbcams = 0);
	~CCalibrationStereo();

	void init(unsigned int nbcams);

	void setCalibCam(unsigned int i, CCalibrationModel *_cam);

	static double getLambda();
	//!set the gain for the virtual visual servoing algorithm 
	static void setLambda(const double &lambda);
	static void setMu(const double &mu);
	
	double getResidual(void) const {return residual;}//!get the residual in pixels
	unsigned int get_npt() const {return npt;}//!get the number of points

    int setActiveExtrinsicParameters(unsigned int icam, bool act_tX = true, bool act_tY = true, bool act_tZ = true, bool act_rX = true, bool act_rY = true, bool act_rZ = true);
    unsigned int getNbActiveExtrinsicParameters(unsigned int icam);
    bool isActiveExtrinsicParameter(unsigned int icam, unsigned int k);
    int toTorseur6ddl(vpColVector & T6, unsigned int icam);
    
	static calibStats fullCalibVVSMulti(unsigned int nbPose, 
					CCalibrationStereo *table_cal, 
					CModelStereo & system,
					bool verbose = false);

	double computeStdDeviation();

};

#endif

