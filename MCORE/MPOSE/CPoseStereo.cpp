
/*!
  \file vpPoseOmniVirtualVisualServoing.cpp
  \brief Compute the pose using virtual visual servoing approach with omnidirectional images
*/

#include <MIXEDVISION/CPosePerspective.h>
#include <MIXEDVISION/CPoseOmni.h>
#include <MIXEDVISION/CPoseParaboloid.h>
#include <MIXEDVISION/CPoseStereo.h>

#include <visp/vpExponentialMap.h>

#include <visp/vpImageIo.h>

/*!
  \brief Compute the pose using virtual visual servoing approach on omnidirectional images

*/

CPoseStereo::CPoseStereo() : systeme(0)
{
    poseCams = NULL;
    listP = NULL;
    nptCam = NULL;
    ciVc1 = NULL;
    I = NULL;
    Mask = NULL;
    
	init();
}

CPoseStereo::~CPoseStereo()
{
    if(poseCams != NULL)
    {
        delete [] poseCams;
    }
    if(listP != NULL)
        delete [] listP;
    if(nptCam != NULL)
        delete [] nptCam;
    if(ciVc1 != NULL)
        delete [] ciVc1;
    if(I != NULL)
        delete [] I;
    if(Mask != NULL)
        delete [] Mask;
}

void CPoseStereo::setLambda(double a)
{
	lambda = a;
}

void CPoseStereo::setMu(double a)
{
	muRef = a;
}

void CPoseStereo::setVvsIterMax(int nb)
{
	vvsIterMax = nb;
}

void CPoseStereo::setSystem(CModelStereo & _systeme)
{
    systeme = _systeme;
    
    if(poseCams != NULL)
    {
        for(int icam = 0 ; icam < systeme.nbcams ; icam++)
            if(poseCams[icam] != NULL)
                delete poseCams[icam];
        delete [] poseCams;
    }
    poseCams = new CPose *[systeme.get_nbcams()];
    for(int icam = 0 ; icam < systeme.nbcams ; icam++)
        poseCams[icam] = NULL;

    if(listP != NULL)
        delete [] listP;
    listP = new vpList<CPoint>[systeme.get_nbcams()];
    if(nptCam != NULL)
        delete [] nptCam;
    nptCam = new int[systeme.get_nbcams()];
    if(ciVc1 != NULL)
        delete [] ciVc1;
    ciVc1 = new vpVelocityTwistMatrix[systeme.get_nbcams()];
    for(int icam = 0 ; icam < systeme.nbcams ; icam++)
        ciVc1[icam].buildFrom(systeme.ciMc1[icam]);
    
    if(I != NULL)
        delete [] I;
    I = new vpImage<unsigned char> *[systeme.get_nbcams()];
    
    if(Mask != NULL)
        delete [] Mask;
    Mask = new vpImage<unsigned char> *[systeme.get_nbcams()];
}

void CPoseStereo::setPoseCam(unsigned int i, CPose *_pose)
{
	if(poseCams[i] != NULL)
		npt -= poseCams[i]->get_npt();
	poseCams[i] = _pose;
	npt += poseCams[i]->get_npt();

    setListPoints(i, poseCams[i]->listP);
    //+ faire les addPoint ici
}

void CPoseStereo::setImageCam(unsigned int icam, vpImage<unsigned char> *_I, vpImage<unsigned char> *_Mask)
{
    I[icam] = _I;
    Mask[icam] = _Mask;
}

void CPoseStereo::init()
{
	clearPoint();

	lambda = 0.1 ;
	
	muRef = 0.01;

	vvsIterMax = 200 ;
}

void CPoseStereo::clearPoint()
{
	for(int icam = 0 ; icam < systeme.nbcams ; icam++)
        clearPoint(icam);
    npt = 0;
}

void CPoseStereo::clearPoint(unsigned int icam)
{
	listP[icam].kill() ;
	nptCam[icam] = 0 ;
}

void CPoseStereo::addPoint(unsigned int icam, const CPoint& newP)
{
	listP[icam] += newP;
	nptCam[icam]++;
}

void CPoseStereo::setListPoints(unsigned int icam, vpList<CPoint> & lP)
{
	clearPoint(icam);
	listP[icam] = lP;
    
	nptCam[icam] = lP.nbElements();
}

poseStats
CPoseStereo::poseVirtualVS(vpHomogeneousMatrix & cMo)
{
    poseStats resPose;
    //poseCams permet d'acceder a la liste des points et aux poses individuelles
    cMo = systeme.ciMc1[0].inverse() * poseCams[0]->cMo;
    if (sqrt(vpMath::sqr(cMo[0][3])+vpMath::sqr(cMo[1][3])+vpMath::sqr(cMo[2][3])) < 0.2)
    {
        poseCams[0]->poseInit(cMo, systeme.cam[0]);
        poseCams[0]->poseVirtualVS(systeme.cam[0], cMo);
        
        cMo = systeme.ciMc1[0].inverse() * cMo;
        for(int icam = 0 ; icam < systeme.nbcams ; icam++)
            poseCams[icam]->cMo = systeme.ciMc1[icam] * cMo;
    }
	try
	{
		double  residu_1 = 1e8 ;
		double r =1e8-1;

		int iter = 0 ;

		vpMatrix L(2*npt,6), Lp;
		vpColVector err(2*npt) ;
		vpColVector sd(2*npt),s(2*npt) ;

		vpMatrix H, Hs;
		vpMatrix Id; // matrice identite
		
		vpColVector v, gradient_du_cout;
		
		CPoint P;
        CPose ** poseObjects = new CPose *[systeme.get_nbcams()]; //pour eviter des reinit permanentes
		
		// create sd
		int k =0 ;
        for(int icam = 0 ; icam < systeme.nbcams ; icam++)
        {
            switch(systeme.cam[icam]->getType())
            {
            case Omni :
            case Fisheye :
                    poseObjects[icam] = new CPoseOmni();
                break;
            case Persp :
                    poseObjects[icam] = new CPosePerspective();
                break;
            case Paraboloid :
                    poseObjects[icam] = new CPoseParaboloid();
                break;
            default:
                std::cout << "CPoseStereo::poseVirtualVS : Type de camera inconnu" << std::endl;
                continue;
                break;
            }
            
            listP[icam].front() ;
            
            while (!listP[icam].outside())
            {
                P = listP[icam].value() ;
                sd[2*k] = P.get_x() ;
                sd[2*k+1] = P.get_y() ;
                listP[icam].next() ;
                k++ ;
            }
        }
		
		mu = muRef;
		vpMatrix Ls;
		vpHomogeneousMatrix cjMo;
        
		while((int)((residu_1 - r)*1e12) !=0)
		{
			L = 0;
			
			residu_1 = r ;

			// Compute the interaction matrix and the error
			int k =0 ;
            for(int icam = 0 ; icam < systeme.nbcams ; icam++)
            {
                listP[icam].front() ;
                cjMo = poseCams[icam]->cMo;//systeme.ciMc1[icam]*cMo;

                while (!listP[icam].outside())
                {
                    P = listP[icam].value() ;

                    P.changeFrame(cjMo) ;
                    systeme.cam[icam]->project3DImage(P);

                    s[2*k] = P.get_x();  /* point projected from cMo */
                    s[2*k+1] = P.get_y();

                    poseObjects[icam]->computeJacobianForVVS(P, systeme.cam[icam], Ls);
                    
                    //On multiplie Li par ciVc1
                    Ls = Ls * ciVc1[icam];

                    for(int i = 2*k, ii = 0 ; ii < 2 ; i++, ii++)
                        for(int j = 0 ; j < 6 ; j++)
                            L[i][j] = Ls[ii][j];

                    listP[icam].next() ;

                    k++ ;
                }
            }
			err = s - sd ;

			// compute the residual
			r = err.sumSquare() ;
			
			//std::cout << r << std::endl;
			
#ifdef GN			
			// compute the pseudo inverse of the interaction matrix
			L.pseudoInverse(Lp,1e-16) ;
			
			// compute the VVS control law
			v = -lambda*Lp*err ;
#else
#ifdef LM
			if(r < residu_1)
				mu /= 2.0;
			/*else
				if(r > residu_1)
					mu *= 2.0;*/
			
			Lp = L.t();
			
			gradient_du_cout = Lp * err;
			
			Hs = L.AtA();
			int nr = Hs.getRows();
			Id.eye(nr);
			for(int ind = 0 ; ind < nr ; ind++) Id[ind][ind] = Hs[ind][ind];
			
			H = (mu * Id + Hs).inverseByLU();//.pseudoInverse(); //
			
			v = -lambda * H * gradient_du_cout;
#endif
#endif
			// update the pose
			cMo = vpExponentialMap::direct(v).inverse()*cMo ;
            
			for(int icam = 0 ; icam < systeme.nbcams ; icam++)
                poseCams[icam]->cMo = systeme.ciMc1[icam] * cMo;
            
			if (iter++>vvsIterMax) break ;
		}
        
        resPose.nbIter = iter;
        
        for(int i=0;i<err.getRows();i++)
            err[i] = fabs(err[i]);
        resPose.moy = vpColVector::mean(err);
        
        //    vpColVector std(error.getRows());
        double std = 0;
        for(int i=0;i<err.getRows();i++)
            std += vpMath::sqr(err[i]-resPose.moy);
        resPose.std = sqrt(std / err.getRows());
	}
	catch(...)
	{
		vpERROR_TRACE(" ") ;
		throw ;
	}
	
    return resPose;
}
