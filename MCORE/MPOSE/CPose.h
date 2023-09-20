
/*!
  \file CPose.h
  \brief Tools for pose computation (pose from point only) with omnidirectional images
  \ingroup libpose

  \author Guillaume Caron

  \sa 
  \date   
*/

#ifndef CPOSE_HH
#define CPOSE_HH

#include <vector>
#include <math.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpList.h>
#include <visp/vpImage.h>
#include <visp/vpColor.h>
#include <MIXEDVISION/CModel.h>
#include <MIXEDVISION/CPoint.h>

/*!
  \class CPose
  \brief  class used for pose computation from N points (pose from point only) with omnidirectional images


  \author Guillaume Caron

  \sa 
  \date
*/


class 
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CPose
{
public:
/*  typedef enum
    {
      LAGRANGE         ,
      DEMENTHON        ,
      LOWE             ,
      LAGRANGE_LOWE    ,
      DEMENTHON_LOWE   ,
      VIRTUAL_VS       ,
      DEMENTHON_VIRTUAL_VS,
      LAGRANGE_VIRTUAL_VS
    } vpPoseMethodType;
*/
  int npt ;       //!< number of point used in pose computation
  vpList<CPoint> listP ;     //!< array of point (use here class CPoint)

  vpHomogeneousMatrix cMo;
  CModel *cam;

  double residual ;     //!< compute the residual in meter
  double lambda ;//!< parameters use for the virtual visual servoing approach
  double mu, muRef;
  int vvsIterMax ; //! define the maximum number of iteration in VVS

  //! constructor
  CPose(); //1 pour un capteur paracatadioptric, 0 pour une camera perspective
  //! destructor
  virtual ~CPose();

  void setLambda(double a);
  void setMu(double a);
  void setVvsIterMax(int nb);

  int get_npt();

  void init() ;
  //! suppress all the point in the array of point
  void clearPoint() ;
  //! Add a new point in this array
  void addPoint(const CPoint& P) ;
  //! Add a new point in this array
  void setListPoints(vpList<CPoint> & lP) ;

  void setCamera(CModel *);

  //! compute the pose using virtual visual servoing approach
  void poseVirtualVS(CModel *cam, vpHomogeneousMatrix & cMo);

  virtual void computeJacobianForVVS(CPoint & P, CModel *cam, vpMatrix & Ls) = 0;
  virtual void poseInit(vpHomogeneousMatrix & cMo, CModel *cam=NULL);
  virtual int initViewLines(CModel *cam, vpColVector *sP, int nbPts = -1) {return 1;}; // = 0 par la suite, quand tous les modèles utiliseront le même calcul linéaire de pose
  //Virer virtual si problème d'affichage du repère
  /*virtual*/ void getFrame(std::vector<SImagePoint> & ptsPose,
                  vpHomogeneousMatrix &cMo,
                  CModel *cam,
                  double size);

  //! compute the pose for a given method
//  virtual void computePose(vpPoseMethodType methode, vpHomogeneousMatrix &cMo) ;

} ;


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
