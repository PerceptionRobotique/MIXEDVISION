
/*!
  \file CPoseOmni.h
  \brief Tools for pose computation (pose from point only) with omnidirectional images
  \ingroup libpose

  \author Guillaume Caron

  \sa 
  \date   
*/

#ifndef CPOSEOMNI_HH
#define CPOSEOMNI_HH

#include <MIXEDVISION/CPose.h>
#include <MIXEDVISION/COmni.h>

#include <math.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpList.h>


/*!
  \class CPoseOmni
  \brief  class used for pose computation from N points (pose from point only) with omnidirectional images


  \author Guillaume Caron

  \sa 
  \date
*/

//#define ANCIENNEMETHODE

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CPoseOmni : public CPose
{

public:

	CPoseOmni();
	~CPoseOmni();
	
	int OutOfImage(int i, int j, int half, int rows, int cols);
	vpMatrix computeConic2Circle(vpColVector a);
	void computeConicFromNormal(vpColVector & N, vpColVector & a, COmni *cam);
	void computeNormalFromTwoPoints(vpColVector & Xs0, vpColVector & Xs, vpColVector & N);
/*
	void displayFrame(vpImage<unsigned char> &I,
							vpHomogeneousMatrix &cMo,
							CModel *cam,
							double size,
							vpColor col);
    void displayConic(vpImage<unsigned char>&I,
                      int i1,
                      int j1,
                      int i2,
                      int j2,
                      vpColVector a, //ellipseParameters dans l'espace métrique
                      CModel *cam,
                      vpColor color,
                      int l = 1);
    */
#ifdef ANCIENNEMETHODE
	void poseInit(vpHomogeneousMatrix & cMo, CModel *cam=NULL);

	//! Used in poseInit
	void genere_g(vpMatrix & costheta, vpMatrix & d, int nums[3], double *A);
#endif
	
	//! compute the pose using virtual visual servoing approach
	void computeJacobianForVVS(CPoint & P, CModel *cam, vpMatrix & Ls);
	int initViewLines(CModel *cam, vpColVector *sP, int nbPts = -1);
	
} ;



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
