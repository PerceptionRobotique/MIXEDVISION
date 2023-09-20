
/*!
 \file CPose.h
 \brief Tools for pose computation (pose from point only) with omnidirectional images
 \ingroup libpose
 
 \author Guillaume Caron
 
 \sa
 \date
 */

#ifndef CPOSESTEREO_HH
#define CPOSESTEREO_HH

#include <math.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpList.h>
#include <visp/vpImage.h>
#include <visp/vpColor.h>
#include <MIXEDVISION/CPose.h>
#include <MIXEDVISION/CModelStereo.h>
#include <MIXEDVISION/CPoint.h>

/*!
 \class CPose
 \brief  class used for pose computation from N points (pose from point only) with omnidirectional images
 
 
 \author Guillaume Caron
 
 \sa
 \date
 */


class CPoseStereo
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
    CModelStereo systeme;
    vpHomogeneousMatrix cMo;
    CPose **poseCams;
    vpVelocityTwistMatrix *ciVc1;
    vpImage<unsigned char> **I;
    vpImage<unsigned char> **Mask;
    
    int npt ;       //!< number of point used in pose computation
    int *nptCam ;       //!< number of point used in pose computation
    vpList<CPoint> *listP ;     //!< array of point (use here class CPoint)
    
    
    double residual ;     //!< compute the residual in meter
    double lambda ;//!< parameters use for the virtual visual servoing approach
    double mu, muRef;
    int vvsIterMax ; //! define the maximum number of iteration in VVS
    
    //! constructor
    CPoseStereo(); //1 pour un capteur paracatadioptric, 0 pour une camera perspective
    //! destructor
    virtual ~CPoseStereo();
    
    void setLambda(double a);
    void setMu(double a);
    void setVvsIterMax(int nb);
    
    void init() ;
    //! suppress all the point in the array of point
    void clearPoint() ;
    void clearPoint(unsigned int icam);
    //! Add a new point in this array
    void addPoint(unsigned int icam, const CPoint& P) ;
    //! Add a new point in this array
    void setListPoints(unsigned int icam, vpList<CPoint> & lP) ;
    
    void init(unsigned int nbcams);
    void setPoseCam(unsigned int i, CPose *_pose);
    void setImageCam(unsigned int icam, vpImage<unsigned char> *_I, vpImage<unsigned char> *_Mask = NULL);
    void setSystem(CModelStereo & );
    
    //! compute the pose using virtual visual servoing approach
    poseStats poseVirtualVS(vpHomogeneousMatrix & cMo);
    
//    poseStats poseSyntheticApertureVVS(vpHomogeneousMatrix & cMo, unsigned int largMax = 150);
//    poseStats poseSyntheticApertureVVS_NV(vpHomogeneousMatrix & cMo, unsigned int largMax = 200);
    
} ;


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
