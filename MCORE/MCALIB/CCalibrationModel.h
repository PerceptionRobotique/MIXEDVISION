#ifndef AFX_CCalibrationModel
#define AFX_CCalibrationModel

#include <MIXEDVISION/CModel.h>
#include <MIXEDVISION/CPoint.h>

#include <vector>

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpList.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>
#include <visp/vpImage.h>
#include <visp/vpColor.h>

class 
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CCalibrationModel
{
public:
/*	//Variables privées
	typedef enum{
		CALIB_LAGRANGE,
		CALIB_VIRTUAL_VS,
		CALIB_VIRTUAL_VS_DIST,
		CALIB_LAGRANGE_VIRTUAL_VS,
		CALIB_LAGRANGE_VIRTUAL_VS_DIST,
	} vpCalibrationMethodType ;*/

	unsigned int npt ;       //!< number of points used in calibration computation
	double residual ;		   //!< residual in pixel for camera model without distortion

	vpList<CPoint> listP;
	CModel *cam;
	int nbparamintr; //sous entendu "estimés"

	vpHomogeneousMatrix cMo;

	static double threshold;
	static unsigned int nbIterMax;
	static double gain; 
	static double mu;
	static double muRef;
	
	CCalibrationModel(CModel *cam=NULL);
	virtual ~CCalibrationModel();

	void init(CModel *_cam);
    void updateCamera(CModel *_cam);
	void clearPoint() ;//! Suppress all the point in the array of point
	void addPoint(double X, double Y, double Z, double u, double v);//! Add a new point in this array
	void addPoint(CPoint &P);
	void setListPoints(vpList<CPoint> &lP);
	//!set the gain for the virtual visual servoing algorithm
	static double getLambda();
	//!set the gain for the virtual visual servoing algorithm 
	static void setLambda(const double &lambda);
	static void setMu(const double &mu);
	unsigned int get_nbparamintr() {return nbparamintr;}

	double getResidual(void) const {return residual;}//!get the residual in pixels
	unsigned int get_npt() const {return npt;}//!get the number of points

//	void computePose(CModel &cam, vpHomogeneousMatrix &cMo);

	void computePoseAndIntrinsicJacobiansForVVS(CPoint & P, CModel *cam, vpMatrix & Lsr, vpMatrix & Lsi);
	virtual void computePoseJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsr) = 0;
	virtual void computeIntrinsicJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsi) = 0;
	virtual void updateCameraParameters(CModel *cam, vpColVector & Tc_cam) = 0;
	virtual void setCamera(CModel *cam) = 0;


	void calibVVS( CModel *cam , vpHomogeneousMatrix &cMo,bool verbose = false);
	static int calibVVSMulti(unsigned int nbPose, unsigned int nbPosesValides, CCalibrationModel *table_cal, CModel *cam, bool verbose = false);
    static int calibUndistorsionsVVSMulti(unsigned int nbPose, CCalibrationModel *table_cal, CModel *cam, bool verbose = false);

	double computeStdDeviation();

    int getPrimitivesReproj(std::vector<SImagePoint> & ptsMire);
};
#endif
