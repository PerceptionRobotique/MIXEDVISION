/****************************************************************************
 *
 * Copyright (C) 2009 M.I.S. All rights reserved.
 *
 * This software was developed at:
 * Laboratoire M.I.S. 
 * 7, rue du moulin neuf
 * 80000 Amiens
 * http://mis.u-picardie.fr
 *
 *
 * Authors:
 * ViSP Lagadic http://www.irisa.fr/lagadic
 * Guillaume Caron http://mis.u-picardie.fr/~g-caron/
 * Damien Eynard http://mis.u-picardie.fr/~eynard/
 *
 *****************************************************************************/

#ifndef AFX_CCalibrationPerspective
#define AFX_CCalibrationPerspective

#include <MIXEDVISION/CCalibrationModel.h>
#include <MIXEDVISION/CPerspective.h>

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CCalibrationPerspective : public CCalibrationModel
{
public:
  CCalibrationPerspective(CPerspective *cam = NULL);
  ~CCalibrationPerspective();

  //void calibLagrange( vpCameraParameters &cam , vpHomogeneousMatrix &cMo);
  void computePoseJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsr);
  void computeIntrinsicJacobianForVVS(CPoint &P, CModel *cam, vpMatrix & Lsi);

  void updateCameraParameters(CModel *cam, vpColVector & Tc_cam);

  void setCamera(CModel *cam);
	
};

#endif
