#include <MIXEDVISION/CCalibrationModel.h>

#include <MIXEDVISION/CPose.h>

#include <visp/vpExponentialMap.h>

//#define AFFDETAILSTXT

double CCalibrationModel::threshold = 1e-8f;
unsigned int CCalibrationModel::nbIterMax = 300;
double CCalibrationModel::gain = 0.35;//0.35;
double CCalibrationModel::muRef = 0.01;
double CCalibrationModel::mu = CCalibrationModel::muRef;

CCalibrationModel::CCalibrationModel(CModel *_cam)
{
	init(_cam);
}

CCalibrationModel::~CCalibrationModel()
{
}

void CCalibrationModel::init(CModel *_cam)
{
	cam = _cam;
	clearPoint();
	residual = 0 ;
	nbparamintr = 4;//_cam->getNbActiveParameters();
	if(_cam != NULL)
		nbparamintr=cam->getNbActiveParameters();
	
}

void CCalibrationModel::updateCamera(CModel *_cam)
{
	if(_cam != NULL)
    {
        cam = _cam;
		nbparamintr=cam->getNbActiveParameters();
    }
}

void CCalibrationModel::clearPoint()
{
	listP.kill() ;
	npt = 0 ;
}

void CCalibrationModel::addPoint(double X, double Y, double Z, double u, double v)
{
	CPoint newP;
	newP.setObjetImage(X, Y, Z, u, v);
	listP += newP;
	npt++;
}

void CCalibrationModel::addPoint(CPoint& newP)
{
	listP += newP;
	npt++;
}

void CCalibrationModel::setListPoints(vpList<CPoint> & lP)
{
	clearPoint();
	listP = lP;
	npt = lP.nbElements();
}


/*!
 Compute the pose cMo
 \param cam : camera intrinsic parameters used for computation
 \param cMo : computed pose
 */
/*void
 CCalibrationModel::computePose(CModel &cam, vpHomogeneousMatrix &cMo)
 {
 // The vpPose class mainly contents a list of vpPoint (that is (X,Y,Z, x, y) )
 CPose pose ;
 //  the list of point is cleared (if that's not done before)
 pose.clearPoint() ;
 // we set the 3D points coordinates (in meter !) in the object/world frame
 
 listP.front();
 for (unsigned int i =0 ; i < npt ; i++)
 {
 CPoint P = listP.value();
 
 cam.pixelMeterConversion(P);
 pose.addPoint(P);
 }
 
 pose.poseInit(cMo);
 pose.poseVirtualVS(cMo);
 }
 */

/*!
 Compute and return the standard deviation expressed in pixel
 for pose matrix and camera intrinsic parameters for model without distortion.
 \param cMo : the matrix that defines the pose to be tested.
 \param cam : camera intrinsic parameters to be tested.
 \return the standard deviation by point of the error in pixel .
 */
double
CCalibrationModel::computeStdDeviation()
{
	double residual = 0 ;
	
	listP.front();
	
	for (unsigned int i =0 ; i < npt ; i++)
	{
		CPoint P = listP.value();
		//les coordonnees detectees dans l image
		double u = P.get_u(), v = P.get_v();
		//cible dans repere camera
		P.changeFrame(cMo);
		//projection de la cible dans le plan virtuel normalise metrique
		cam->project3DImage(P);
		//les coordonnees du point projete dans l image en pixels
		cam->meterPixelConversion(P);
		
		residual += sqrt(vpMath::sqr(P.get_u()-u) + vpMath::sqr(P.get_v()-v));
		
		listP.next();
	}
	this->residual = residual ;
	return residual/npt ;
}

int
CCalibrationModel::getPrimitivesReproj(std::vector<SImagePoint> & ptsMire)
{
  listP.front();
  ptsMire.clear();
  SImagePoint ptIm;
    
	for (unsigned int i =0 ; i < npt ; i++)
	{
		CPoint P = listP.value();
		//cible dans repere camera
		P.changeFrame(cMo);
		//projection de la cible dans le plan virtuel normalise metrique
		cam->project3DImage(P);
		//les coordonnees du point projete dans l image en pixels
		cam->meterPixelConversion(P);
		ptIm.u = P.get_u();
		ptIm.v = P.get_v();

		ptsMire.push_back(ptIm);
		listP.next();
	}
	
	return 0;
}

void CCalibrationModel::computePoseAndIntrinsicJacobiansForVVS(CPoint &P, CModel *cam, vpMatrix & Lsr, vpMatrix & Lsi)
{
    //Pose calibration Jacobian
    vpMatrix Ls;
    vpMatrix Li(2,2);
	computePoseJacobianForVVS(P, cam, Ls);
    //Li.resize(2,2);
	if(cam->distorsions)
	{
		double x = P.get_x(), y = P.get_y(), r2, distR, xy, x2, y2;
        xy = x*y;
        x2 = x*x;
        y2 = y*y;
		r2 = x2+y2;
		distR = 1;
		if(cam->activek[0])
			distR += cam->k[0]*r2;
		if(cam->activek[1])
			distR += cam->k[1]*r2*r2; //r4
		if(cam->activek[2])
			distR += cam->k[2]*r2*r2*r2; //r6
        
		double facCom = cam->k[0]+2*cam->k[1]*r2+3*cam->k[2]*r2*r2;
		
		Li[0][0] = cam->getau() * (distR + 2*( x2*(facCom) + cam->k[3]*y + 3*cam->k[4]*x ));
		Li[0][1] = cam->getau() * (2*( xy*(facCom) + cam->k[3]*x + cam->k[4]*y ));
		
		Li[1][0] = cam->getav() * (2*( xy*(facCom) + cam->k[3]*x + cam->k[4]*y )); //(cam->getav()/cam->getau())*Li[0][1];
		Li[1][1] = cam->getav() * (distR + 2*( y2*(facCom) + 3*cam->k[3]*y + cam->k[4]*x ));
        
        //std::cout << Li << std::endl;
	}
	else
	{
		Li[0][0] = cam->getau(); 
		Li[1][1] = cam->getav();
	}
	
	Lsr = Li*Ls;
    
    //intrinsic calibration Jacobian
	computeIntrinsicJacobianForVVS(P, cam, Lsi);

    if(cam->distorsions)
	{
        double x = P.get_x(), y = P.get_y();
        double au = cam->getau(), av = cam->getav();
        double r2 = x*x+y*y;
        int ind = Lsi.getCols();
        
        Lsi.resize(2,cam->getNbActiveParameters(), false);
        
        if(cam->activek[0])
        {
            Lsi[0][ind] = au*x*r2;
            Lsi[1][ind] = av*y*r2;
            ind++;
        }
        if(cam->activek[1])
        {
            Lsi[0][ind] = au*x*r2*r2;
            Lsi[1][ind] = av*y*r2*r2;
            ind++;
        }
        if(cam->activek[2])
        {
            Lsi[0][ind] = au*x*r2*r2*r2;
            Lsi[1][ind] = av*y*r2*r2*r2;
            ind++;
        }
        if(cam->activek[3])
        {
            Lsi[0][ind] = 2*au*x*y;
            Lsi[1][ind] = av*(r2+2*y*y);
            ind++;
        }
        if(cam->activek[4])
        {
            Lsi[0][ind] = au*(r2+2*x*x);
            Lsi[1][ind] = 2*av*x*y;
            ind++;
        }
    }
    //std::cout << Lsi << std::endl;
    //exit(12);
}

void
CCalibrationModel::calibVVS(
							CModel *cam,
							vpHomogeneousMatrix & cMo,
							bool verbose)
{
#ifdef AFFDETAILSTXT
	std::cout.precision(10);
#endif
    
	vpColVector P(2*npt);
	vpColVector Pd(2*npt);
	
	vpMatrix H, Hs;
	vpMatrix Id; // matrice identite
	
	vpColVector gradient_du_cout;
	
	//  double lambda = 0.1 ;
	unsigned int iter = 0 ;
	
	double  residu_1 = 1e12 ; //1e12
	double r =residu_1-1;
	
	mu = muRef;
	
	vpColVector Tc, Tc_v(6), Tc_cam ;
	CPoint Pt;
	vpMatrix L, Ls, Lsr, Lsi, Lp;
	vpColVector error ;

	while (vpMath::equal(residu_1,r,threshold) == false && iter < nbIterMax)
	{
		L.resize(0, 0);
		
		iter++ ;
		residu_1 = r ;
		
		r = 0 ;
		listP.front();
		for (int i=0 ; i < npt; i++)
		{
			Pt = listP.value();
			
			Pd[2*i]   = Pt.get_u();
			Pd[2*i+1] = Pt.get_v();
			
			Pt.changeFrame(cMo);
			cam->project3DImage(Pt);
			//On enregistre le résultat de la projection sans écraser le (u, v) à atteindre
			listP.modify(Pt);
			
			cam->meterPixelConversion(Pt);
			
			P[2*i]   =  Pt.get_u();
			P[2*i+1] =  Pt.get_v();
			
			listP.next();
		}
		
		listP.front();
		for (int i=0 ; i < npt; i++)
		{
			Pt = listP.value();
			computePoseAndIntrinsicJacobiansForVVS(Pt, cam, Lsr, Lsi);
			
			for(int j = 0 ; j < 2 ; j++)
			{
				r += vpMath::sqr(P[2*i+j]-Pd[2*i+j]);
			}
			
			Ls = vpMatrix::juxtaposeMatrices(Lsr, Lsi);
			L = vpMatrix::stackMatrices(L, Ls);
			
			listP.next();
		}    // end compute interaction matrix
		
		error = P-Pd ;
		//r = r/n_points ;
		
#ifdef GN			
		// compute the pseudo inverse of the interaction matrix
		L.pseudoInverse(Lp,1e-16) ;
		
		// compute the VVS control law
		Tc = -gain*Lp*error ;
#else
#ifdef LM
		if(r < residu_1)
			mu /= 2.0;
		/*else
			if(r > residu_1)
				mu *= 2.0;*/
		
		Lp = L.t();
		
		gradient_du_cout = Lp * error;
		
		Hs = L.AtA();
		int nr = Hs.getRows();
		Id.eye(nr);
		for(int ind = 0 ; ind < nr ; ind++) Id[ind][ind] = Hs[ind][ind];
		
		H = (mu * Id + Hs).inverseByLU();//.pseudoInverse(); //
		
		Tc = -gain * H * gradient_du_cout;
#endif
#endif
		
		//		vpMatrix Lp ;
		//		Lp = L.pseudoInverse(1e-10) ;
		
		//		vpColVector e ;
		//		e = Lp*error ;
		
		//		Tc = -gain*e ;
		
		for (int i=0 ; i <6 ; i++)
			Tc_v[i] = Tc[i] ;
		
		cMo = vpExponentialMap::direct(Tc_v).inverse()*cMo ;
		
		Tc_cam = Tc.rows(7, Tc.getRows());
		updateCameraParameters(cam, Tc_cam);
#ifdef AFFDETAILSTXT
		if (verbose)
		{
			std::cout <<  " std dev " << std::endl << r/npt << std::endl;
			std::cout << "  au = " << cam->getau() <<"\t av = "<< cam->getav() << std::endl ;
			std::cout << "  u0 = " << cam->getu0() <<"\t v0 = "<< cam->getv0() << std::endl ;
			
			if(cam->distorsions)
			{
				std::cout << "with distorsions parameters for undistorted to distorted transformation :" << std::endl ;
				if(cam->activek[0])
					std::cout << "  k1 = " << cam->getk1();
				if(cam->activek[1])
					std::cout <<"\t k2 = "<< cam->getk2();
				if(cam->activek[2])
					std::cout << "\t  k3 = " << cam->getk3();
				if(cam->activek[3])
					std::cout <<"\t k4 = "<< cam->getk4();
				if(cam->activek[4])
					std::cout << "\t  k5 = " << cam->getk5() << std::endl ;
			}
			
			std::cout << " cMo " << std::endl;
			cMo.print();
			std::cout << std::endl;
		}
#endif
		
	}
	if (iter == nbIterMax)
	{
		vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
		/*    throw(vpCalibrationException(vpCalibrationException::convergencyError,
		 "Maximum number of iterations reached")) ;*/
	}
	
	this->cMo   = cMo;
	this->residual = r;
	//  this->cam = cam;
#ifdef AFFDETAILSTXT
	if (verbose)
		std::cout <<  " std dev " << r/npt << std::endl;
	
	std::cout << "Nb iter : " << iter << std::endl;
#endif
}

int
CCalibrationModel::calibVVSMulti(
								 unsigned int nbPose,
								 unsigned int nbPoseValides,
								 CCalibrationModel *table_cal,
								 CModel *cam,
								 bool verbose
								 )
{
#ifdef AFFDETAILSTXT
	std::cout.precision(10);
#endif
    
	int nbPoint[256]; //number of points by image
	int nbPointTotal = 0; //total number of points
	
	unsigned int nbPose6 = 6*nbPoseValides;
	
	for (unsigned int i=0; i<nbPose ; i++)
	{
		nbPoint[i] = table_cal[i].npt;
		nbPointTotal += nbPoint[i];
	}
	
	//  std::cout << "nbPointTotal : " << nbPointTotal << std::endl;
	
	vpColVector P(2*nbPointTotal) ;
	vpColVector Pd(2*nbPointTotal) ;
	
	vpMatrix H, Hs;
	vpMatrix Id; // matrice identite
	
	vpColVector gradient_du_cout;
	
	vpColVector error(2*nbPointTotal) ;
	vpMatrix L(nbPointTotal*2,nbPose6+table_cal[0].get_nbparamintr()); //cam->getNbActiveParameters());
	vpMatrix Lp(nbPose6+table_cal[0].get_nbparamintr(), nbPointTotal*2);
	//  double lambda = 0.1 ;
	unsigned int iter = 0 ;
	
	double  residu_1 = 1e10 ;
	double r =residu_1-1;
	int curPoint;
	
	vpColVector Tc, Tc_v(nbPose6), Tc_cam ;
	
	mu = muRef;
	
	vpHomogeneousMatrix cMoTmp;
	vpMatrix Lsr, Lsi;
	CPoint Pt;
	vpColVector Tc_v_Tmp(6);
	
	while (vpMath::equal(residu_1,r,threshold) == false && iter < nbIterMax)
	{
		/*P=0;
		Pd=0;
		L=0;
		Lp.resize(0, 0);
		error.resize(0);
		gradient_du_cout.resize(0);
		H.resize(0, 0);
		Hs.resize(0, 0);
		Id.resize(0, 0);
		Tc.resize(0);
		Tc_cam.resize(0);
		Tc_v= 0;
		Tc_v_Tmp = 0;
		Lsr.resize(0,0);
		Lsi.resize(0,0);*/
		
		iter++ ;
		residu_1 = r ;
		
		r = 0. ;
		curPoint = 0 ; //current point indice
		for (unsigned int p=0; p<nbPose ; p++)
		{
			cMoTmp = table_cal[p].cMo;
			
			//std::cout << "cMoTmp :" << cMoTmp << std::endl;
			
			table_cal[p].listP.front();
			for (int i=0 ; i < nbPoint[p]; i++)
			{
				unsigned int curPoint2 = 2*curPoint;
				Pt = table_cal[p].listP.value();
				
				Pd[curPoint2]   = Pt.get_u();
				Pd[curPoint2+1] = Pt.get_v();
				
				Pt.changeFrame(cMoTmp);
				cam->project3DImage(Pt);
				//On enregistre le résultat de la projection sans écraser le (u, v) a atteindre
				table_cal[p].listP.modify(Pt);
				
				cam->meterPixelConversion(Pt);
				
				P[curPoint2]   =  Pt.get_u();
				P[curPoint2+1] =  Pt.get_v();
				
				//r += ((vpMath::sqr(P[curPoint2]-Pd[curPoint2]) + vpMath::sqr(P[curPoint2+1]-Pd[curPoint2+1]))) ;
				
				table_cal[p].listP.next();
				curPoint++;
			}
		}
		
		curPoint = 0 ; //current point indice
        unsigned int q = 0;
		for (unsigned int p=0; p<nbPose ; p++)
		{
			table_cal[p].listP.front();
			
			for (int i=0 ; i < nbPoint[p]; i++)
			{
				Pt = table_cal[p].listP.value();
				table_cal[p].computePoseAndIntrinsicJacobiansForVVS(Pt, cam, Lsr, Lsi);
                
				for(int j = 0 ; j < 2 ; j++)
				{
					r += vpMath::sqr(P[2*curPoint+j]-Pd[2*curPoint+j]);
				}
				
				//copie des deux (ou 4) lignes du jacobien du point P
				for(unsigned int curPoint2 = 2*curPoint, m = 0; m < 2 ; curPoint2++, m++)
				{
					//On récupère la partie correspondant à du/dr
					for(int j = q, k = 0 ; k < 6 ; j++, k++)
						L[curPoint2][j] = Lsr[m][k];
					
					//On récupère la partie correspondant à du/di
					for(int j = nbPose6, k = 0 ; k < table_cal[p].get_nbparamintr() ; j++, k++)
                    //for(int j = nbPose6, k = 0 ; k < cam->getNbActiveParameters() ; j++, k++)
						L[curPoint2][j] = Lsi[m][k];
				}
				
				curPoint++;
				table_cal[p].listP.next();
			}    // end interaction
            if(nbPoint[p] != 0)
                q += 6;
		}
		
		error = P-Pd;
        
        //std::cout << L << std::endl;
		
		//r = r/nbPointTotal ;
#ifdef GN			
		// compute the pseudo inverse of the interaction matrix
		L.pseudoInverse(Lp,1e-16) ;
		
		// compute the VVS control law
		Tc = -gain*Lp*error ;
#else
#ifdef LM
		if(r < residu_1)
			mu /= 2.0;
		/*else
			if(r > residu_1)
				mu *= 2.0;*/
		
		Lp = L.t();
		
		gradient_du_cout = Lp * error;
		
		Hs = L.AtA();
        //std::cout << Hs << std::endl;
		int nr = Hs.getRows();
		Id.eye(nr);
		for(int ind = 0 ; ind < nr ; ind++) Id[ind][ind] = Hs[ind][ind];
		
        //std::cout << Hs << std::endl;
        
		H = (mu * Id + Hs).inverseByLU();//.pseudoInverse(); //
		
		Tc = -gain * H * gradient_du_cout;
#endif
#endif
        //std::cout << sqrt(Tc.sumSquare()) << std::endl;
		
		//Lp = L.pseudoInverse(1e-10) ;
		
		//vpColVector e ;
		//e = Lp*error ;
		
		//Tc = -e*gain ;
		
		Tc_v =0 ;
		for (unsigned int i = 0 ; i < nbPose6 ; i++)
			Tc_v[i] = Tc[i] ;
		
		Tc_cam = Tc.rows(nbPose6+1, Tc.getRows());
		//std::cout << "Tcam : " << Tc_cam.t() << std::endl;
		table_cal[0].updateCameraParameters(cam, Tc_cam);
		
		//Mise à jout de chaque pose
		for (unsigned int p = 0 ; p < nbPoseValides ; p++)
		{ 
			//vpColVector Tc_v_Tmp(6) ;
			
			for (unsigned int i = 0 ; i < 6 ; i++)
				Tc_v_Tmp[i] = Tc_v[6*p + i];
			
			table_cal[p].cMo = vpExponentialMap::direct(Tc_v_Tmp,1).inverse()
			* table_cal[p].cMo;
			
		}
#ifdef AFFDETAILSTXT
		if(iter == 1)
			std::cout <<  " residu initial " << sqrt(r/(2*nbPointTotal)) << std::endl;

		if (verbose)
		{
			std::cout <<  " calibOmniVVSMulti " << std::endl;
			std::cout << "Tcam : " << Tc_cam.t() << std::endl;
			std::cout <<  " residu " << r << std::endl;
			std::cout <<  " std dev " << r/nbPointTotal << std::endl;
		}
#endif
	}// fin VVS
    
    if(cam->getNbActiveDistorsionParameters())
        calibUndistorsionsVVSMulti(nbPose, table_cal, cam, verbose);

#ifdef AFFDETAILSTXT
    std::cout <<  " residu final " << sqrt(r/(2*nbPointTotal)) << std::endl;
    
    if (iter == nbIterMax)
	{
		vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
		std::cout << cam;
		
		/*    throw(vpCalibrationException(vpCalibrationException::convergencyError,
		 "Maximum number of iterations reached")) ;*/
	}
#endif
    
	for (unsigned int p = 0 ; p < nbPose ; p++)
	{
		table_cal[p].cam = cam;
	}
    
#ifdef AFFDETAILSTXT
	if (verbose)
	{
		std::cout <<  " calibOmniVVSMulti " << r << std::endl;
		std::cout <<  " std dev " << r/nbPointTotal << std::endl;
		
		std::cout << "Nb iter : " << iter << std::endl;
		
		for(int i=0;i<error.getRows();i++)
			error[i] = fabs(error[i]);
		double moy = vpColVector::mean(error);
		std::cout <<  " mean pix err " << moy << std::endl;
		
		//    vpColVector std(error.getRows());
		double std = 0;
		for(int i=0;i<error.getRows();i++)	
			std += vpMath::sqr(error[i]-moy);
		std /= error.getRows();
		std::cout <<  " pix std dev " << sqrt(std) << std::endl;
		
		int borne = (Lp.getRows()<Lp.getCols())?Lp.getRows():Lp.getCols();
		vpColVector A(borne);
		for(int i = 0 ; i < borne ; i++)
			A[i] = 3.0*sqrt(fabs(Lp[i][i]))*std;
		std::cout << "std px py" << A[6] << " " << A[7] << std::endl;
		std::cout << "std u0 v0" << A[8] << " " << A[9] << std::endl;
		std::cout << "std xi" << A[10] << std::endl;
	}
#endif
	
	return iter;
}

int
CCalibrationModel::calibUndistorsionsVVSMulti(
                                              unsigned int nbPose,
                                              CCalibrationModel *table_cal,
                                              CModel *cam,
                                              bool verbose
                                              )
{
#ifdef AFFDETAILSTXT
	std::cout.precision(10);
#endif
    
	int nbPoint[256]; //number of points by image
	int nbPointTotal = 0; //total number of points
	
	for (unsigned int i=0; i<nbPose ; i++)
	{
		nbPoint[i] = table_cal[i].npt;
		nbPointTotal += nbPoint[i];
	}
	
	//  std::cout << "nbPointTotal : " << nbPointTotal << std::endl;
	
	vpColVector P(2*nbPointTotal) ;
	vpColVector Pd(2*nbPointTotal) ;
	
	vpMatrix H, Hs;
	vpMatrix Id; // matrice identite
	
	vpColVector gradient_du_cout;
	
	vpColVector error ;
	vpMatrix L(nbPointTotal*2,cam->getNbActiveDistorsionParameters());//table_cal[0].get_nbparamintr()); //cam->getNbActiveParameters());
	vpMatrix Lp ;
	//  double lambda = 0.1 ;
	unsigned int iter = 0 ;
	
	double  residu_1 = 1e10 ;
	double r =residu_1-1;
	int curPoint;
	
	vpColVector Tc;
	
	mu = muRef;
	
	vpHomogeneousMatrix cMo;
	vpMatrix Lsr, Lsi(2,cam->getNbActiveDistorsionParameters());
	CPoint Pt;
    double xd, yd, rd2, xdyd;
	
	while (vpMath::equal(residu_1,r,threshold) == false && iter < nbIterMax)
	{
		/*P=0;
         Pd=0;
         L=0;
         Lp.resize(0, 0);
         error.resize(0);
         gradient_du_cout.resize(0);
         H.resize(0, 0);
         Hs.resize(0, 0);
         Id.resize(0, 0);
         Tc.resize(0);
         Tc_cam.resize(0);
         Tc_v= 0;
         Tc_v_Tmp = 0;
         Lsr.resize(0,0);
         Lsi.resize(0,0);*/
		
		iter++ ;
		residu_1 = r ;
		
		r = 0. ;
		curPoint = 0 ; //current point indice
		for (unsigned int p=0; p<nbPose ; p++)
		{
			cMo = table_cal[p].cMo;
			
			//std::cout << "cMoTmp :" << cMoTmp << std::endl;
			
			table_cal[p].listP.front();
			for (int i=0 ; i < nbPoint[p]; i++)
			{
				unsigned int curPoint2 = 2*curPoint;
				Pt = table_cal[p].listP.value();
				
				Pt.changeFrame(cMo);
				cam->project3DImage(Pt);
                
                //std::cout << Pt.get_x() << " " << Pt.get_y() << " ";
                
				Pd[curPoint2]   = Pt.get_x();
				Pd[curPoint2+1] = Pt.get_y();
                
				//On enregistre le résultat de la projection sans écraser le (u, v) a atteindre
				//table_cal[p].listP.modify(Pt);
				
				cam->pixelMeterConversion(Pt);
                //std::cout << Pt.get_x() << " " << Pt.get_y() << std::endl;
				P[curPoint2]   =  Pt.get_x();
				P[curPoint2+1] =  Pt.get_y();
				
				//r += ((vpMath::sqr(P[curPoint2]-Pd[curPoint2]) + vpMath::sqr(P[curPoint2+1]-Pd[curPoint2+1]))) ;
				
				table_cal[p].listP.next();
				curPoint++;
			}
		}
        //exit(12);
		
		curPoint = 0 ; //current point indice
		for (unsigned int p=0; p<nbPose ; p++)
		{
			table_cal[p].listP.front();
			
			for (int i=0 ; i < nbPoint[p]; i++)
			{
				Pt = table_cal[p].listP.value();
                
                xd = (Pt.get_u() - cam->getu0())/cam->getau();
                yd = (Pt.get_v() - cam->getv0())/cam->getav();
                xdyd = xd*yd;
                rd2 = xd*xd+yd*yd;
				
                int ind = 0;
                if(cam->activek[0])
                {
                    Lsi[0][ind] = xd*rd2;
                    Lsi[1][ind] = yd*rd2;
                    ind++;
                }
                if(cam->activek[1])
                {
                    Lsi[0][ind] = xd*rd2*rd2;
                    Lsi[1][ind] = yd*rd2*rd2;
                    ind++;
                }
                if(cam->activek[2])
                {
                    Lsi[0][ind] = xd*rd2*rd2*rd2;
                    Lsi[1][ind] = yd*rd2*rd2*rd2;
                    ind++;
                }
                if(cam->activek[3])
                {
                    Lsi[0][ind] = 2*xdyd;
                    Lsi[1][ind] = xd*xd+3*yd*yd;
                    ind++;
                }
                if(cam->activek[4])
                {
                    Lsi[0][ind] = 3*xd*xd+yd*yd;
                    Lsi[1][ind] = 2*xdyd;
                    ind++;
                }
                
				for(int j = 0 ; j < 2 ; j++)
				{
					r += vpMath::sqr(P[2*curPoint+j]-Pd[2*curPoint+j]);
				}
				
				//copie des deux (ou 4) lignes du jacobien du point P
				for(unsigned int curPoint2 = 2*curPoint, m = 0; m < 2 ; curPoint2++, m++)
				{
					//On récupère la partie correspondant à dx/dik
					for(int j = 0 ; j < cam->getNbActiveDistorsionParameters() ; j++)
						L[curPoint2][j] = Lsi[m][j];
				}
				
				curPoint++;
				table_cal[p].listP.next();
			}    // end interaction
		}
		
		error = P-Pd;
        
        //std::cout << L << std::endl;
		
		//r = r/nbPointTotal ;
#ifdef GN
		// compute the pseudo inverse of the interaction matrix
		L.pseudoInverse(Lp,1e-16) ;
		
		// compute the VVS control law
		Tc = -gain*Lp*error ;
#else
#ifdef LM
		if(r < residu_1)
			mu /= 2.0;
		/*else
         if(r > residu_1)
         mu *= 2.0;*/
		
		Lp = L.t();
		
		gradient_du_cout = Lp * error;
		
		Hs = L.AtA();
        //std::cout << Hs << std::endl;
		int nr = Hs.getRows();
		Id.eye(nr);
		for(int ind = 0 ; ind < nr ; ind++) Id[ind][ind] = Hs[ind][ind];
		
        //std::cout << Hs << std::endl;
        
		H = (mu * Id + Hs).inverseByLU();//.pseudoInverse(); //
		
		Tc = -gain * H * gradient_du_cout;
#endif
#endif
        //std::cout << sqrt(Tc.sumSquare()) << std::endl;
		
		//std::cout << "Tc : " << Tc.t() << std::endl;
        double ik[5], ind = 0;
        for(int i = 0 ; i < 5 ; i++)
        {
            if(cam->activek[i])
            {
                ik[i] = Tc[ind];
                ind++;
            }
            else
                ik[i] = 0.0;
        }
        
        cam->setUndistorsionParameters(
                                       cam->getik1() + ik[0],
                                       cam->getik2() + ik[1],
                                       cam->getik3() + ik[2],
                                       cam->getik4() + ik[3],
                                       cam->getik5() + ik[4]);
#ifdef AFFDETAILSTXT
		if(iter == 1)
			std::cout <<  " residu initial " << sqrt(r/(2*nbPointTotal)) << std::endl;
		
		if (verbose)
		{
			std::cout <<  " calibDistorsionVVSMulti " << std::endl;
			std::cout <<  " residu " << r << std::endl;
			std::cout <<  " std dev " << r/nbPointTotal << std::endl;
		}
#endif
		
	}// fin VVS
    
#ifdef AFFDETAILSTXT
    std::cout <<  " residu final " << sqrt(r/(2*nbPointTotal)) << std::endl;
	
    if (iter == nbIterMax)
	{
		vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
		std::cout << cam;
		
		/*    throw(vpCalibrationException(vpCalibrationException::convergencyError,
		 "Maximum number of iterations reached")) ;*/
	}
    
	if (verbose)
	{
		std::cout <<  " calibDistorsionsVVSMulti " << r << std::endl;
		std::cout <<  " std dev " << r/nbPointTotal << std::endl;
		
		std::cout << "Nb iter : " << iter << std::endl;
		
		for(int i=0;i<error.getRows();i++)
			error[i] = fabs(error[i]);
		double moy = vpColVector::mean(error);
		std::cout <<  " mean pix err " << moy << std::endl;
		
		//    vpColVector std(error.getRows());
		double std = 0;
		for(int i=0;i<error.getRows();i++)	
			std += vpMath::sqr(error[i]-moy);
		std /= error.getRows();
		std::cout <<  " pix std dev " << sqrt(std) << std::endl;
	}
#endif
    
	return iter;
}
