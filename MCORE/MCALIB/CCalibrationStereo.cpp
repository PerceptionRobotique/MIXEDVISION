#include <MIXEDVISION/CCalibrationStereo.h>

#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpColVector.h>

//#define AFFDETAILSTXT

double CCalibrationStereo::threshold = 1e-8f;
unsigned int CCalibrationStereo::nbIterMax = 300;
double CCalibrationStereo::gain = 0.35; //0.25
double CCalibrationStereo::muRef = 0.02;
double CCalibrationStereo::mu = CCalibrationStereo::muRef;

CCalibrationStereo::CCalibrationStereo(unsigned int _nbcams) : systeme(_nbcams)
{
	init(_nbcams);
}

void CCalibrationStereo::init(unsigned int _nbcams)
{
	nbcams = _nbcams;
	if(nbcams)
	{
		systeme.init(nbcams);
		calibcams = new CCalibrationModel *[nbcams];
        activeExtrinsicParameters = new std::vector<bool>[nbcams];
        nbActiveExtrinsicParameters = new unsigned int[nbcams];
		for(int icam = 0 ; icam < nbcams ; icam++)
        {
			calibcams[icam] = NULL;
            activeExtrinsicParameters[icam].reserve(6);
            for(int j = 0 ; j < 6 ; j++)
                activeExtrinsicParameters[icam][j] = true;
            nbActiveExtrinsicParameters[icam] = 6;
        }
	}
	npt = residual = 0;
}

CCalibrationStereo::~CCalibrationStereo()
{
	if(nbcams)
	{
		/*
		 //déjà libéré ailleurs par CInterface
		 for(int icam = 0 ; icam < nbcams ; icam++)
			if(calibcams[icam] != NULL)
				delete calibcams[icam];
		 */
		if(calibcams != NULL)
			delete [] calibcams;
        if(activeExtrinsicParameters != NULL)
            delete [] activeExtrinsicParameters;
        if(nbActiveExtrinsicParameters != NULL)
            delete [] nbActiveExtrinsicParameters;
        
	}
}

void CCalibrationStereo::setCalibCam(unsigned int i, CCalibrationModel *_cam)
{
	if(calibcams[i] != NULL)
		npt -= calibcams[i]->get_npt();
	calibcams[i] = _cam;
	npt += calibcams[i]->get_npt();
}

double CCalibrationStereo::computeStdDeviation()
{
	double residu = 0, npt = 0;
	for(int icam = 0 ; icam < systeme.nbcams ; icam++)
	{
		npt += calibcams[icam]->get_npt();
		calibcams[icam]->computeStdDeviation();
		residu += calibcams[icam]->getResidual();
#ifdef AFFDETAILSTXT
		std::cout << "npt : " << npt << " residu : " << residu << std::endl;
#endif
	}
	this->residual = residu;
	//this->npt = npt;
	return residu/npt;
}

int CCalibrationStereo::setActiveExtrinsicParameters(unsigned int icam, bool act_tX, bool act_tY, bool act_tZ, bool act_rX, bool act_rY, bool act_rZ)
{
    nbActiveExtrinsicParameters[icam] = 0;
    if((activeExtrinsicParameters[icam][0] = act_tX))
        nbActiveExtrinsicParameters[icam]++;
    if((activeExtrinsicParameters[icam][1] = act_tY))
        nbActiveExtrinsicParameters[icam]++;
    if((activeExtrinsicParameters[icam][2] = act_tZ))
        nbActiveExtrinsicParameters[icam]++;
    if((activeExtrinsicParameters[icam][3] = act_rX))
        nbActiveExtrinsicParameters[icam]++;
    if((activeExtrinsicParameters[icam][4] = act_rY))
        nbActiveExtrinsicParameters[icam]++;
    if((activeExtrinsicParameters[icam][5] = act_rZ))
        nbActiveExtrinsicParameters[icam]++;
    
    return 0;
}

int CCalibrationStereo::toTorseur6ddl(vpColVector & T6, unsigned int icam)
{
    vpColVector T = T6;
    T6.resize(6);
    int k = 0;
    for (int j = 0 ; j < 6 ; j++)
    {
        if(activeExtrinsicParameters[icam][j])
        {
            T6[j] = T[k];
            k++;
        }
        else
            T6[j] = 0.0;
    }
    
    return 0;
}

bool CCalibrationStereo::isActiveExtrinsicParameter(unsigned int icam, unsigned int k)
{
    if(k < 6)
        return activeExtrinsicParameters[icam][k];
    return true;
}

unsigned int CCalibrationStereo::getNbActiveExtrinsicParameters(unsigned int icam)
{
    return nbActiveExtrinsicParameters[icam];
}

calibStats CCalibrationStereo::fullCalibVVSMulti(unsigned int nbPose, 
										   CCalibrationStereo *table_cal, 
										   CModelStereo & systeme,
										   bool verbose)
{
#ifdef AFFDETAILSTXT
	std::cout.precision(10);
#endif
    
	int **nbPoint = NULL; //number of points by image
	int nbPointTotal = 0; //total number of points
	
	unsigned int nbPose6 = 6*nbPose, nbPoseRel, nbPoseRelDeb, icam;
	int *nbcolsintr = new int[systeme.nbcams];

#ifdef AFFDETAILSTXT
    for (icam = 0 ; icam < systeme.nbcams ; icam++)
    {
		//std::cout << "c" << icam+1 << "Mc1 : " << std::endl;
		systeme.ciMc1[icam].print();
		std::cout << std::endl;
    }
#endif
	
	vpVelocityTwistMatrix *ciVc1 = new vpVelocityTwistMatrix[systeme.nbcams]; //Matrices de changement de repere du torseur cinematique
	ciVc1[0].eye();//(6,6);
	for(icam = 1 ; icam < systeme.nbcams ; icam++)
		ciVc1[icam].buildFrom(systeme.ciMc1[icam]);
	
	nbPoint = new int *[systeme.nbcams];
	for(int i = 0 ; i < systeme.nbcams ; i++)
		nbPoint[i] = new int[256];
	
	for (unsigned int p=0; p < nbPose ; p++)
    {
        std::cout << "pose " << p << " ";
		for(icam = 0 ; icam < systeme.nbcams ; icam++)
		{
			nbPoint[icam][p] = table_cal[p].calibcams[icam]->get_npt();
            std::cout << nbPoint[icam][p] << " ";
			nbPointTotal += nbPoint[icam][p];
		}
        std::cout << std::endl;
    }
#ifdef AFFDETAILSTXT
	std::cout << "nbPointTotal : " << nbPointTotal << std::endl;
#endif
	
	nbcolsintr[0] = 0;
	for(icam = 1 ; icam < systeme.nbcams ; icam++)
	{
		nbcolsintr[icam] = nbcolsintr[icam-1]+table_cal[0].calibcams[icam-1]->get_nbparamintr();
        //systeme.cam[icam-1]->getNbActiveParameters();
	}
	
	vpMatrix H, Hs;
	vpMatrix Id; // matrice identite
	
	vpColVector gradient_du_cout;
	
	vpColVector P(2*nbPointTotal) ;
	vpColVector Pd(2*nbPointTotal) ;
	vpColVector error ;
	//  double lambda = 0.1 ;
	unsigned int iter = 0 ;
	
	double  residu_1 = 1e12 ;
	double r =1e12-1;
	int curPoint;
	vpList<CPoint> **listPviac1 = new vpList<CPoint> *[nbPose];
	
	for(unsigned int i = 0 ; i < nbPose ; i++)
		listPviac1[i] = new vpList<CPoint>[systeme.nbcams];
	
	unsigned int nbcolsL = nbPose6;//+6*(systeme.nbcams-1);
    nbPoseRel = 0;
    for(icam = 1 ; icam < systeme.nbcams ; icam++)
        nbPoseRel += table_cal[0].getNbActiveExtrinsicParameters(icam);
    nbcolsL += nbPoseRel;
	for(icam = 0 ; icam < systeme.nbcams ; icam++)
		nbcolsL += table_cal[0].calibcams[icam]->get_nbparamintr();
        //systeme.cam[icam]->getNbActiveParameters();
	
	vpMatrix L(nbPointTotal*2,nbcolsL), Lsr, LsrciVc1, Lsi, Lsrc1ci, Lp;
	
	mu = muRef;
	
	vpHomogeneousMatrix ciMoTmp, ciMc1c1Mo;
	CModel *cam;
	CPoint Pt;
	
	vpColVector Tc_v(6);
	
	while (vpMath::equal(residu_1,r,threshold) == false && iter < nbIterMax)
	{
        /*for(icam = 0 ; icam < systeme.nbcams ; icam++)
        {
            std::cout << "debut boucle" << std::endl;
            std::cout << systeme.ciMc1[icam] << std::endl;
            std::cout << systeme.cam[icam]->getau() << " " << systeme.cam[icam]->getav() << " " << systeme.cam[icam]->getu0() << " " << systeme.cam[icam]->getv0() << " ";
            if(icam == 0)
                std::cout << ((COmni *)(systeme.cam[icam]))->getXi();
            std::cout << std::endl;
        }*/
        
		L = 0;
		
		iter++ ;
		residu_1 = r ;
		
		r = 0 ;
		curPoint = 0 ; //current point indice
		for (unsigned int p=0; p<nbPose ; p++)
		{
			for(icam = 0 ; icam < systeme.nbcams ; icam++)
			{
				ciMoTmp = table_cal[p].calibcams[icam]->cMo;//, ciMc1c1Mo;                
                //std::cout << "ciMoTmp : " << std::endl << ciMoTmp << std::endl;
				cam = systeme.cam[icam];
				
				ciMc1c1Mo = systeme.ciMc1[icam] * table_cal[p].calibcams[0]->cMo;
                //std::cout << "ciMc1c1Mo : " << std::endl << ciMc1c1Mo << std::endl;
				table_cal[p].calibcams[icam]->listP.front();
				
				listPviac1[p][icam].kill();
				listPviac1[p][icam].front();
				
				for (int i=0 ; i < nbPoint[icam][p]; i++)
				{
					unsigned int curPoint2 = 2*curPoint;
					Pt = table_cal[p].calibcams[icam]->listP.value();
					
					Pd[curPoint2]   = Pt.get_u();
					Pd[curPoint2+1] = Pt.get_v();
					
					Pt.changeFrame(ciMoTmp);
					cam->project3DImage(Pt);
					//On enregistre le résultat de la projection sans écraser le (u, v) a atteindre
					table_cal[p].calibcams[icam]->listP.modify(Pt);
					
					cam->meterPixelConversion(Pt);
					
					P[curPoint2]   =  Pt.get_u();
					P[curPoint2+1] =  Pt.get_v();
					
					//Point dans le repere camera i en passant par le repere camera 1 et en utilisant ciMc1
					Pt.changeFrame(ciMc1c1Mo);
					cam->project3DImage(Pt);
					listPviac1[p][icam].addRight(Pt);
					
					//r += sqrt(((vpMath::sqr(P[curPoint2]-Pd[curPoint2]) + vpMath::sqr(P[curPoint2+1]-Pd[curPoint2+1]))));
					
					table_cal[p].calibcams[icam]->listP.next();
					curPoint++;
				}
			}
		}
		
        //std::cout << "Pd : " << Pd.t() << std::endl << std::endl;
        //std::cout << "P : " << P.t() << std::endl;

		curPoint = 0 ; //current point indice
		for (unsigned int p=0; p<nbPose ; p++)
		{
			unsigned int q = 6*p;
            nbPoseRelDeb = 0;
			for(icam = 0 ; icam < systeme.nbcams ; icam++)
			{
				cam = systeme.cam[icam];
				
				table_cal[p].calibcams[icam]->listP.front();
				listPviac1[p][icam].front();
				
				for (int i=0 ; i < nbPoint[icam][p]; i++)
				{
					// On recupere d'abord les jacobiens ds/dr (que l'on multiplie par ciVc1) et ds/di
					Pt = table_cal[p].calibcams[icam]->listP.value();
					table_cal[p].calibcams[icam]->computePoseAndIntrinsicJacobiansForVVS(Pt, cam, Lsr, Lsi);
                    
                    //std::cout << "Lsr : " << Lsr << std::endl;
                    //std::cout << "Lsi : " << Lsi << std::endl;
					
					for(int j = 0 ; j < 2 ; j++)
					{
						r += vpMath::sqr(P[2*curPoint+j]-Pd[2*curPoint+j]);
					}
					
					if(icam > 0) //On multiplie Lsr par ciVc1
						LsrciVc1 = Lsr * ciVc1[icam];
					else
						LsrciVc1 = Lsr;
					
					// On recupere enfin le jacobien ds/drc1ci si i>1
					if(icam > 0)
					{
						Pt = listPviac1[p][icam].value();
                        vpMatrix Lstmp;
                        vpMatrix Li(2,2);
						table_cal[p].calibcams[icam]->computePoseJacobianForVVS(Pt, cam, Lstmp);

                        if(cam->distorsions)
                        {
                            double x = Pt.get_x(), y = Pt.get_y(), r2, distR, xy, x2, y2;
                            xy = x * y;
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
                        
                        Lsrc1ci = Li*Lstmp;
                        
                        //std::cout << "Lsrc1ci : " << Lsrc1ci << std::endl << std::endl;
					}
					
					//copie des deux lignes du jacobien du point P
					for(unsigned int m = 0, curPoint2 = 2*curPoint ; m < 2 ; curPoint2++, m++)
					{
						int k, j;
						//On récupère la partie correspondant à ds/dr
						for(k = 0, j = q ; k < 6 ; j++, k++)
							L[curPoint2][j] = LsrciVc1[m][k];
						
						if(icam > 0)
						{
							//On récupère la partie correspondant à ds/drc1ci
							for(k = 0, j = nbPose6+nbPoseRelDeb ; k < 6 ; k++)
                            {
                                if(table_cal[0].isActiveExtrinsicParameter(icam, k))
                                {
                                   L[curPoint2][j] = Lsrc1ci[m][k];
                                   j++;
                                }
                            }
						}
						
						//On récupère la partie correspondant à ds/di
						for(k = 0, j = nbPose6+nbPoseRel+nbcolsintr[icam] ; 
                            //k < systeme.cam[icam]->getNbActiveParameters() ; j++, k++) 
							k < table_cal[p].calibcams[icam]->get_nbparamintr() ; j++, k++) 
							L[curPoint2][j] = Lsi[m][k];
					}
					
					curPoint++;
					
					table_cal[p].calibcams[icam]->listP.next();
					listPviac1[p][icam].next();
				}    // end interaction
                if(icam > 0)
                    nbPoseRelDeb += table_cal[0].getNbActiveExtrinsicParameters(icam);
			}
		}
		error = P-Pd ;
		//r = r/nbPointTotal ;
		
		//std::cout << Lp.getCols() << " | " << error.getRows() << std::endl;
		
       // std::cout << L << std::endl;
        
		vpColVector Tc, Tc_vc1ci(6), Tc_cam;
		
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
		
        //std::cout << Hs << std::endl;
        
		H = (mu * Id + Hs).inverseByLU();//.pseudoInverse(); //
		
		Tc = -gain * H * gradient_du_cout;
#endif
#endif
		//vpMatrix Lp ;
		//Lp = L.pseudoInverse(1e-10) ;
		
		//		vpColVector e ;
		//		e = Lp*error ;
		
		//		Tc = -e*gain ;
		
		//Mise a jour des poses relatives
		int i, k;
		i = nbPose6;
		for(icam = 1 ; icam < systeme.nbcams ; icam++)
		{
            //std::cout << "ciVc1["<<icam<<"] : " << std::endl << ciVc1[icam] << std::endl;
			// On prend le torseur de la pose relative mir1 - mir2
            Tc_vc1ci.resize(table_cal[0].getNbActiveExtrinsicParameters(icam));
			for (k = 0 ; k < table_cal[0].getNbActiveExtrinsicParameters(icam) ; k++, i++)
				Tc_vc1ci[k] = Tc[i] ;
            
            //std::cout << Tc_vc1ci.t() << std::endl;
            table_cal[0].toTorseur6ddl(Tc_vc1ci, icam);
            //std::cout << Tc_vc1ci.t() << std::endl;
            
            systeme.ciMc1[icam] = vpExponentialMap::direct(Tc_vc1ci).inverse()*systeme.ciMc1[icam];
            ciVc1[icam].buildFrom(systeme.ciMc1[icam]);
		}
		
		//mise a jour des parametres des cameras
		i = nbPose6+nbPoseRel;//+6*(systeme.nbcams-1);
		k = i;
		for(icam = 0 ; icam < systeme.nbcams ; icam++)
		{
			k += table_cal[0].calibcams[icam]->get_nbparamintr();
            //systeme.cam[icam]->getNbActiveParameters();
			//std::cout << "Taille : " << Tc.getRows() << " i : " << i+1 << " k : " << k << std::endl;
			Tc_cam = Tc.rows(i+1, k);
			//std::cout << "cam update : " << Tc_cam.t() << std::endl;
			table_cal[0].calibcams[icam]->updateCameraParameters(systeme.cam[icam], Tc_cam);
			i = k;
		}
		//Mise à jour de chaque pose
		for (unsigned int p = 0 ; p < nbPose ; p++)
		{ 
			for (unsigned int i = 0 ; i < 6 ; i++)
				Tc_v[i] = Tc[6*p + i];
			
            //std::cout << "pose update : " << std::endl << vpExponentialMap::direct(Tc_v,1).inverse() << std::endl;
            
			table_cal[p].calibcams[0]->cMo = vpExponentialMap::direct(Tc_v,1).inverse()
			* table_cal[p].calibcams[0]->cMo;
			
			for(icam = 1 ; icam < systeme.nbcams ; icam++)
				table_cal[p].calibcams[icam]->cMo = systeme.ciMc1[icam] * table_cal[p].calibcams[0]->cMo;
		}
#ifdef AFFDETAILSTXT
		if (verbose)
		{
			std::cout <<  " calibStereoVVSMulti " << std::endl;
			std::cout <<  " residu " << r << std::endl;
			//      	std::cout <<  " std dev " << sqrt(r/nbPointTotal) << std::endl;
			std::cout <<  " std dev " << r/nbPointTotal << std::endl;
			for(icam = 1 ; icam < systeme.nbcams ; icam++)
			 {
			 //		std::cout << *(systeme.cam[icam]);
			 std::cout << " c" << icam+1 << "Mc1 : " << std::endl;
			 systeme.ciMc1[icam].print();
			 }
            std::cout <<  std::endl;
		}
#endif
		
	}
	if (iter == nbIterMax)
	{
		vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)",nbIterMax);
		/*    throw(vpCalibrationException(vpCalibrationException::convergencyError,
		 "Maximum number of iterations reached")) ;*/
	}
	for (unsigned int p = 0 ; p < nbPose ; p++)
	{
		for(icam = 0 ; icam < systeme.nbcams ; icam++)
		{
			table_cal[p].systeme.setCamera(icam, systeme.cam[icam]);
			table_cal[p].systeme.setciMc1(icam, systeme.ciMc1[icam]);
		}
	}
	
	calibStats resCalib;
	
	resCalib.nbIter = iter;

	
	//Scaramuzza style
	double moySca = 0.;
	for(int i=0;i<error.getRows();i+=2)
		moySca += sqrt(error[i]*error[i] + error[i+1]*error[i+1]);
	std::cout << "OCamCalib-style average error [pixels]: " << (moySca/(error.getRows()*0.5)) << std::endl;
	

	for(int i=0;i<error.getRows();i++)
		error[i] = fabs(error[i]);
	resCalib.moy = vpColVector::mean(error);
	
	//    vpColVector std(error.getRows());
	double std = 0;
	for(int i=0;i<error.getRows();i++)	
		std += vpMath::sqr(error[i]-resCalib.moy);
	resCalib.std = sqrt(std / error.getRows());

#ifdef AFFDETAILSTXT
	if (verbose)
	{
		std::cout <<  " calibStereoVVSMulti " << r << std::endl;
		//    std::cout <<  " std dev " << sqrt(r/nbPointTotal) << std::endl;
		std::cout <<  " std dev " << r/nbPointTotal << std::endl;
		for(icam = 1 ; icam < systeme.nbcams ; icam++)
		{
			//		std::cout << *(systeme.cam[icam]);
			std::cout << " c" << icam+1 << "Mc1 : " << std::endl;
			systeme.ciMc1[icam].print();
		}

		std::cout << "Nb iter : " << resCalib.nbIter << std::endl;
		std::cout <<  " mean pix err " << resCalib.moy << std::endl;
		std::cout <<  " pix std dev " << resCalib.std << std::endl;
	}
#endif
	
	for(unsigned int i = 0 ; i < systeme.nbcams ; i++)
		delete [] nbPoint[i];
	delete [] nbPoint;
	delete [] ciVc1;
	
	for(unsigned int i = 0 ; i < nbPose ; i++)
		delete [] listPviac1[i];
	delete [] listPviac1;
	delete [] nbcolsintr;
	
	return resCalib;
}

