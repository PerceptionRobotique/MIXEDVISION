
#ifndef AFX_CPosePersp
#define AFX_CPosePersp

#include <MIXEDVISION/CPose.h>
#include <MIXEDVISION/CPerspective.h>

#include <math.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpList.h>

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CPosePerspective : public CPose
{

public:

	CPosePerspective();
	~CPosePerspective();

	//! compute the pose using virtual visual servoing approach
	void computeJacobianForVVS(CPoint & P, CModel *cam, vpMatrix & Ls);

	//A decommenter et commenter poseInit (faire la même chose dans CPosePerspective.cpp)
	// pour utiliser la méthode de calcul de pose linéaire de Ameller
	//int initViewLines(CModel *cam, vpColVector *sP, int nbPts = -1);
	
	void poseInit(vpHomogeneousMatrix & cMo, CModel *cam=NULL);
} ;



#endif
