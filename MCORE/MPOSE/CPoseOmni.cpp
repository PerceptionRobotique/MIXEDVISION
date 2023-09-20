
/*!
  \file CPoseOmni.cpp
  \brief Compute the pose using virtual visual servoing approach with omnidirectional images
*/

#include <MIXEDVISION/CPoseOmni.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

/*!
  \brief Compute the pose using virtual visual servoing approach on omnidirectional images

*/

CPoseOmni::CPoseOmni()
{
}

CPoseOmni::~CPoseOmni()
{
}

/*void
CPoseOmni::displayFrame(vpImage<unsigned char> &I,
					vpHomogeneousMatrix &cMo,
					CModel *_cam,
					double size,
					vpColor col)
{
	COmni *cam = (COmni *)_cam;

	vpColVector Xs0(3), Xs(3), N(3), a(5);
	// used by display
	CPoint o; o.setWorldCoordinates ( 0.0,0.0,0.0 ) ;
	CPoint x; x.setWorldCoordinates ( size,0.0,0.0 ) ;
	CPoint y; y.setWorldCoordinates ( 0.0,size,0.0 ) ;
	CPoint z; z.setWorldCoordinates ( 0.0,0.0,size ) ;
	
	o.changeFrame(cMo) ;
	cam->project3DSphere(o, Xs0[0], Xs0[1], Xs0[2]);
	cam->project3DImage(o);
	cam->meterPixelConversion(o);
	
	x.changeFrame(cMo) ;
	cam->project3DSphere(x, Xs[0], Xs[1], Xs[2]);
	computeNormalFromTwoPoints(Xs0, Xs, N);
	computeConicFromNormal(N, a, cam);
	
	cam->project3DImage(x);
	cam->meterPixelConversion(x);
	if ( col == vpColor::none )
		displayConic(I,
				 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
				 vpMath::round ( x.get_v() ), vpMath::round ( x.get_u() ),
				 a, //ellipseParameters dans l'espace métrique
				 cam,
				 vpColor::red);
	else 
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( x.get_v() ), vpMath::round ( x.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 col);
	
	y.changeFrame(cMo) ;
	cam->project3DSphere(y, Xs[0], Xs[1], Xs[2]);
	computeNormalFromTwoPoints(Xs0, Xs, N);
	computeConicFromNormal(N, a, cam);
	
	cam->project3DImage(y) ;
	cam->meterPixelConversion(y);
	if ( col == vpColor::none )
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( y.get_v() ), vpMath::round ( y.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 vpColor::green);
	else 
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( y.get_v() ), vpMath::round ( y.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 col);
	
	z.changeFrame(cMo) ;
	cam->project3DSphere(z, Xs[0], Xs[1], Xs[2]);
	computeNormalFromTwoPoints(Xs0, Xs, N);
	computeConicFromNormal(N, a, cam);
	cam->project3DImage(z) ;
	cam->meterPixelConversion(z);
	if ( col == vpColor::none )
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( z.get_v() ), vpMath::round ( z.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 vpColor::blue);
	else 
		displayConic(I,
					 vpMath::round ( o.get_v() ), vpMath::round ( o.get_u() ),
					 vpMath::round ( z.get_v() ), vpMath::round ( z.get_u() ),
					 a, //ellipseParameters dans l'espace métrique
					 cam,
					 col);

}*/

void 
CPoseOmni::computeNormalFromTwoPoints(vpColVector & Xs0, vpColVector & Xs, vpColVector & N)
{
	N = vpColVector::cross(Xs0, Xs);
}

void 
CPoseOmni::computeConicFromNormal(vpColVector & N, vpColVector & a, COmni *cam)
{	
	double xi = cam->getXi(), xi2;
	xi2 = xi*xi;
	
	a.resize(5);
	
	a[0] = (N[0]*N[0]/(N[2]*N[2]))*(1-xi2) - xi2;
	a[1] = (N[1]*N[1]/(N[2]*N[2]))*(1-xi2) - xi2;
	a[2] = (N[0]*N[1]/(N[2]*N[2]))*(1-xi2);
	a[3] = N[0]/N[2];
	a[4] = N[1]/N[2];
}

/*void
CPoseOmni::displayConic(vpImage<unsigned char>&I,
						  int i1,
						  int j1,
						  int i2,
						  int j2,
						  vpColVector a, //ellipseParameters dans l'espace métrique
						  CModel *cam,
						  vpColor color,
						  int l)
{
	vpColVector Pt(3), PtHat(3), PtStop(3), PtStopHat(3), PtStep(3), PtStepHat(3), axe(2), PtLast(3), PtPrecLast(3);
	CPoint CPt;
	double alpha[3], difAgl;
	vpMatrix invP(3,3);
	int rows = I.getRows() ;
	int cols = I.getCols() ;
	
	CPt.setPixUV(j1, i1);
	cam->pixelMeterConversion(CPt);
	Pt[0] = CPt.get_x();
	Pt[1] = CPt.get_y();
	Pt[2] = 1.0;
	
	CPt.setPixUV(j2, i2);
	cam->pixelMeterConversion(CPt);
	PtStop[0] = CPt.get_x();
	PtStop[1] = CPt.get_y();
	PtStop[2] = 1.0;
	
	axe[0] = j2 - j1;
	axe[1] = i2 - i1;
	axe.normalize();

	CPt.setPixUV(j1+axe[0], i1+axe[1]);
	cam->pixelMeterConversion(CPt);
	PtStep[0] = CPt.get_x();
	PtStep[1] = CPt.get_y();
	PtStep[2] = 1.0;
	
	invP = computeConic2Circle(a);
	
	//On passe le premier point de la conique sur le cercle unitaire
	PtHat = invP * Pt;
	for(int j = 0 ; j < 2 ; j++)
		PtHat[j] /= PtHat[2];
	PtHat[2] = 1.0;
	//Calcul de sa coordonnée angulaire dans le cercle
	alpha[0] = atan2(PtHat[1], PtHat[0]);
	if(alpha[0] < 0.0)
		alpha[0] += 2.0*M_PI;
	
	//On passe le dernier point de la conique sur le cercle unitaire
	PtStopHat = invP * PtStop;
	for(int j = 0 ; j < 2 ; j++)
		PtStopHat[j] /= PtStopHat[2];
	PtStopHat[2] = 1.0;
	//Calcul de sa coordonnée angulaire dans le cercle
	alpha[1] = atan2(PtStopHat[1], PtStopHat[0]);
	if(alpha[1] < 0.0)
		alpha[1] += 2.0*M_PI;
	
	//On passe le point step de la conique sur le cercle unitaire
	PtStepHat = invP * PtStep;
	for(int j = 0 ; j < 2 ; j++)
		PtStepHat[j] /= PtStepHat[2];
	PtStepHat[2] = 1.0;
	//Calcul de sa coordonnée angulaire dans le cercle
	alpha[2] = atan2(PtStepHat[1], PtStepHat[0]);
	if(alpha[2] < 0.0)
		alpha[2] += 2.0*M_PI;
	
	//Calcul de la longueur d'un échantillon
	difAgl = vpMath::abs(alpha[2] - alpha[0]);
	if(difAgl > M_PI)
		difAgl = 2.0*M_PI - difAgl;
	
	// Choose starting point
	double alphaS = alpha[0], n_sample;
	
	//définissons le nombre d'échantillons
	n_sample = vpMath::abs(alpha[1] - alpha[0]);
	if(n_sample > M_PI)
	{
		n_sample = 2.0*M_PI-n_sample;
		difAgl *= (alpha[0]<alpha[1])?(-1):1;
	}
	else
		difAgl *= (alpha[0]<alpha[1])?1:(-1);
	
	n_sample /= vpMath::abs(difAgl);
	
	vpMatrix P = invP.inverseByLU();	

	vpMatrix Rot(3,3);
	Rot[0][0] = cos(difAgl); 	Rot[0][1] = -sin(difAgl);
	Rot[1][0] = sin(difAgl);	Rot[1][1] = cos(difAgl);
	Rot[2][2] = 1.0;
	
	for(int i = 0 ; i <= n_sample ; i++)
	{
		Pt = P * PtHat;
		for(int j = 0 ; j < 2 ; j++)
			Pt[j] /= Pt[2];
		Pt[2] = 1.0;

		CPt.set_x(Pt[0]);
		CPt.set_y(Pt[1]);
		cam->meterPixelConversion(CPt);
		Pt[0] = CPt.get_u();
		Pt[1] = CPt.get_v();
		
		// If point is in the image, draw it
		if(!OutOfImage(vpMath::round(Pt[1]), vpMath::round(Pt[0]), 0, rows, cols))
			vpDisplay::displayPoint(I, vpMath::round(Pt[1]), vpMath::round(Pt[0]), color);

		if (i%6 == 0) 
			PtPrecLast = PtLast;
		
		PtLast = Pt;
		
		PtHat = Rot * PtHat;
		
		alphaS += difAgl;
		
	}
	
	vpDisplay::displayArrow ( I,
							 vpMath::round ( PtPrecLast[1] ), vpMath::round ( PtPrecLast[0] ),
							 vpMath::round ( PtLast[1] ), vpMath::round ( PtLast[0] ),
							 color, 6, 4 ) ;
}*/

vpMatrix
CPoseOmni::computeConic2Circle(vpColVector a)
{
	vpMatrix C(3,3), Rp(3,3), R(3,3), abc(3,3);
	vpMatrix invP(3,3);
	C[0][0] = a[0]; 	C[0][1] = a[2]; 	C[0][2] = a[3];
	C[1][0] = a[2]; 	C[1][1] = a[1]; 	C[1][2] = a[4];
	C[2][0] = a[3]; 	C[2][1] = a[4]; 	C[2][2] = 1.0;
	vpColVector EigVals(3);
	vpMatrix EigVects(3,3);

//	//Ok si GSL
	C.eigenValues(EigVals, EigVects);
	//classement des valeurs et vecteurs propres par ordre croissant
	for(int i = 0 ; i < 3 ; i++)
		for(int j = i+1 ; j < 3 ; j ++)
		{
			if(EigVals[j] < EigVals[i])
			{
				double tmp = EigVals[j];
				EigVals[j] = EigVals[i];
				EigVals[i] = tmp;
				
				for(int k = 0 ; k < 3 ; k++)
				{
					tmp = EigVects[k][j];
					EigVects[k][j] = EigVects[k][i];
					EigVects[k][i] = tmp;
				}
			}
		}
	
/*	std::cout << "EigVals : " << EigVals.t() << std::endl;
	std::cout << "EigVects : " << EigVects << std::endl;

	CvMat CMat = cvMat(3, 3, CV_64F, C.data), 
			EigValsMat = cvMat(3, 1, CV_64F, EigVals.data), 
			EigVectsMat = cvMat(3, 3, CV_64F, EigVects.data);
	cvEigenVV(&CMat, &EigVectsMat, &EigValsMat, DBL_EPSILON);
	EigVects = EigVects.t();
	//classement des valeurs et vecteurs propres par ordre croissant
	for(int i = 0 ; i < 3 ; i++)
		for(int j = i+1 ; j < 3 ; j ++)
		{
			if(EigVals[j] < EigVals[i])
			{
				double tmp = EigVals[j];
				EigVals[j] = EigVals[i];
				EigVals[i] = tmp;
				
				for(int k = 0 ; k < 3 ; k++)
				{
					tmp = EigVects[k][j];
					EigVects[k][j] = EigVects[k][i];
					EigVects[k][i] = tmp;
				}
			}
		}	
		
	std::cout << "EigVals : " << EigVals.t() << std::endl;
	std::cout << "EigVects : " << EigVects << std::endl;
	*/
	
	Rp = EigVects;
	
	if( (vpMath::sign(EigVals[1]) == vpMath::sign(EigVals[2])) && (vpMath::sign(EigVals[0]) != vpMath::sign(EigVals[1])))
	{//a est la valeur particulière
		vpMatrix transfRp(3,3);
		transfRp[0][2] = 1.0; transfRp[1][1] = -1.0; transfRp[2][0] = 1.0;
		R = transfRp*Rp;
		for(int i = 0; i < 3 ; i++)
			abc[i][i] = 1.0/sqrt(vpMath::abs(EigVals[2-i]));
	}
	else if( (vpMath::sign(EigVals[0]) == vpMath::sign(EigVals[2])) && (vpMath::sign(EigVals[1]) != vpMath::sign(EigVals[0])))
	{//b est la valeur particulière
		vpMatrix transfRp(3,3);
		transfRp[0][0] = -1.0; transfRp[1][2] = 1.0; transfRp[2][1] = 1.0;
		R = transfRp*Rp;
		abc[0][0] = 1.0/sqrt(vpMath::abs(EigVals[0]));
		abc[1][1] = 1.0/sqrt(vpMath::abs(EigVals[2]));
		abc[2][2] = 1.0/sqrt(vpMath::abs(EigVals[1]));
	}
	else if( (vpMath::sign(EigVals[0]) == vpMath::sign(EigVals[1])) && (vpMath::sign(EigVals[2]) != vpMath::sign(EigVals[0])))
	{//c est la valeur particulière
		R = Rp;
		for(int i = 0; i < 3 ; i++)
			abc[i][i] = 1.0/sqrt(vpMath::abs(EigVals[i]));
	}
	else
    {
#ifdef AFFDETAILSTXT
		std::cout << "ce n est pas une conique" << std::endl;
#endif
    }
	
	invP = R * abc;
	invP = invP.inverseByLU();
	return invP;
}

int
CPoseOmni::OutOfImage(int i, int j, int half, int rows, int cols)
{
	return (! ((i> half+2) &&
			   (i< rows -(half+2)) &&
			   (j>half+2) &&
			   (j<cols-(half+2)))
			) ;
}

void CPoseOmni::computeJacobianForVVS(CPoint & P, CModel *cam, vpMatrix & Ls)
{
	double x = P.get_x(), y = P.get_y();
	double Z = P.get_Z(), Xi = ((COmni *)cam)->getXi() ;

	double X = P.get_X(), Y = P.get_Y();
	double rho = sqrt(X*X+Y*Y+Z*Z);

	Ls.resize(2,6);

	Ls[0][0] = -( (rho*Z + Xi*Z*Z)/(rho*vpMath::sqr(Z + Xi*rho)) + Xi*y*y/rho) ;
	Ls[0][1] = Xi*x*y/rho;
	Ls[0][2] = x * ((rho+Xi*Z) / (rho*(Z + Xi*rho)));
	Ls[0][3] = x*y;
	Ls[0][4] = -( x*x + (Z*Z + Z*Xi*rho)/(vpMath::sqr(Z + Xi*rho)) );
	Ls[0][5] = y;

	Ls[1][0] = Xi*x*y/rho;
	Ls[1][1] = -( (rho*Z + Xi*Z*Z)/(rho*vpMath::sqr(Z + Xi*rho)) + Xi*x*x/rho );
	Ls[1][2] = y * ( (rho+Xi*Z)/(rho*(Z + Xi*rho)) );
	Ls[1][3] = ( (Z*Z + Z*Xi*rho)/(vpMath::sqr(Z + Xi*rho)) ) + y*y;
	Ls[1][4] = -x*y;
	Ls[1][5] = -x;
}

int CPoseOmni::initViewLines(CModel *cam, vpColVector *sP, int nbPts)
{
	int i = 0, nbP;// = listP.nbElement();
	CPoint P;
    
    if(nbPts == -1)
        nbP = listP.nbElement();
    else
        nbP = nbPts;
    
	listP.front();
	while (i < nbP)
	{
		P = listP.value() ;
		
		sP[i] = vpColVector(3);

		((COmni *)cam)->projectImageSphere(P, sP[i][0], sP[i][1], sP[i][2]);
		
		listP.next() ;
		i++;
	}
	
	return 0;
}

#ifdef ANCIENNEMETHODE

void CPoseOmni::poseInit(vpHomogeneousMatrix & cMo, CModel *cam)
{
//	cMo.eye(4);
	static int nums[4][3][3] = { { {0, 1, 2}, {0, 1, 3}, {0, 2, 3} },
				     { {1, 0, 2}, {1, 0, 3}, {1, 2, 3} },
				     { {2, 0, 1}, {2, 0, 3}, {2, 1, 3} },
				     { {3, 0, 1}, {3, 0, 2}, {3, 1, 2} } };
	static int tabi[] = {4, 4, 4, 4, 3, 3, 2};
	static int tabj[] = {2, 1, 0, 0, 1, 0, 0};
	static int tabk[] = {3, 3, 3, 2, 2, 2, 1};
	static int tabl[] = {3, 2, 1, 2, 2, 2, 1};

/*	for(int i=0;i<4;i++)
		for(int j=0;j<3;j++)
			std::cout << nums[i][j][0] << " " << nums[i][j][1] <<  " " << nums[i][j][2] << std::endl;*/

	int i, j;
	CPoint P, Pbis;
	vpList<CPoint> lP, lPbis;
	vpColVector sP[4];
	double fact = 0.0;

	/*// Pré-conditionnement de données ==> normalisation des coordonnées des points objet
	listP.front();
	i = 0;
	while (i < 4)
	{
		P = listP.value();
		fact += sqrt(vpMath::sqr(P.oP[0]) + vpMath::sqr(P.oP[1]) + vpMath::sqr(P.oP[2]));
		//fact += sqrt(vpMath::sqr(P.p[0]) + vpMath::sqr(P.p[1]));
		listP.next();
		i++;
	}
	fact /= 4.0;
	fact = 1.0 / fact;

	std::cout << "fact : " << fact << std::endl;*/
	fact = 1000.0;

	lP.kill();
	lP.front();

	listP.front();
	i = 0;
//	std::cout << "nb pts : " << listP.nbElements() << std::endl;
	while (i < 4)
	{
		P = listP.value() ;

		sP[i] = vpColVector(3);
		//projectInverseOmni(P.p[0], P.p[1], sP[i][0], sP[i][1], sP[i][2], Xi);
		((COmni *)cam)->projectImageSphere(P, sP[i][0], sP[i][1], sP[i][2]);
		P.oP *= fact; // pour conditionnement du problème

		P.oP[3] = 1.0;

		lP.addRight(P);

		listP.next() ;
		i++;
	}

	lPbis.kill();
	lPbis = lP;
	vpMatrix costheta(4,4), distance(4,4);

	lP.front();
	i = 0;
	while(i < 3)
	{
		P = lP.value();
		lPbis.front();
		lPbis.suppress();
		j = i+1;
		while(j < 4)
		{
			Pbis = lPbis.value();
			
			costheta[i][j] = costheta[j][i] = sP[i][0]*sP[j][0] + sP[i][1]*sP[j][1] + sP[i][2]*sP[j][2];
			vpColVector oP = P.oP-Pbis.oP;
			oP.resize(3,false);
			distance[i][j] = distance[j][i] = sqrt( oP.sumSquare() );

			lPbis.next();
			j++;
		}
		lP.next();
		i++;
	}

	//std::cout << costheta << std::endl;

	vpMatrix A(3,5);

	// Pour chaque point, on recherche sa distance au centre de projection
	double x[4] = {0.0,0.0,0.0,0.0};
	for(i = 0 ; i < 4 ; i++)
	{
		for(j = 0 ; j < 3 ; j++)
		{
			genere_g(costheta, distance, nums[i][j], A[j]);
		}

		//std::cout << A << std::endl;
		
		vpColVector S(5), Sb;
		vpMatrix V(5,5), Vb, U;
		//La méthode svd() ne gère que les matrices de taille MxN avec M>=N alors biaisons en ajoutant des zeros...
		U = A;
		U.resize(5,5,false); //Des zéros sont ajoutés uniquement aux nouvelles cellules
		U.svd(S,V);

		vpMatrix B(7,3);
		for(int ind = 0 ; ind < 7 ; ind++)
		{
			B[ind][0] = V[tabi[ind]][3]*V[tabj[ind]][3] - V[tabk[ind]][3]*V[tabl[ind]][3];
			B[ind][1] = V[tabi[ind]][3]*V[tabj[ind]][4] + V[tabi[ind]][4]*V[tabj[ind]][3] 
					- (V[tabk[ind]][3]*V[tabl[ind]][4] + V[tabk[ind]][4]*V[tabl[ind]][3]);
			B[ind][2] = V[tabi[ind]][4]*V[tabj[ind]][4] - V[tabk[ind]][4]*V[tabl[ind]][4];
		}

		B.svd(Sb, Vb);

		vpColVector y = Vb.column(3), t, v4, v5;

		double LambdaSurRho = 0.0, rho, lambda, xp = 0.0;
		int nbVals = 0;

		if(y[1] != 0.0)
		{
			LambdaSurRho = y[0]/y[1];
			nbVals++;
		}
		if(y[2] != 0.0)
		{
			LambdaSurRho += y[1]/y[2];
			nbVals++;
		}
		if(nbVals != 0.0)
			LambdaSurRho /= nbVals;
		else
			LambdaSurRho = 1.0;

		rho = 1.0 / (LambdaSurRho*V[0][3]+V[0][4]);
		lambda = LambdaSurRho * rho;

		v4 = V.column(4); v5 = V.column(5);
		t = lambda*v4 + rho*v5;

		nbVals = 0;
		if(t[0] != 0)
		{
			xp = t[1]/t[0];
			nbVals++;
		}
		if(t[1] != 0)
		{
			xp += t[2]/t[1];
			nbVals++;
		}
		if(t[2] != 0)
		{
			xp += t[3]/t[2];
			nbVals++;
		}
		/*if(t[3] != 0)
		{
			xp += t[4]/t[3];
			nbVals++;
		}*/
		//std::cout << t[0] << " " << t[1] << " " << t[2] << " " << t[3] << " " << t[4] << std::endl;
		//std::cout << t[1]/t[0] << " " << t[2]/t[1] << " " << t[3]/t[2] << " " << t[4]/t[3] << std::endl;
		if(nbVals != 0.0)
		{
			xp /= nbVals;
			//std::cout << "xp : " << xp << std::endl;
			x[i] = sqrt(fabs(xp));
		}
		else
			x[i] = 0.0;
	}

	//std::cout << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << std::endl;
	vpColVector cP[4], oP[4];
	// Calcul des 4 points dans le repère caméra
	lP.front();
	i = 0;
	while (i < 4)
	{
		P = lP.value() ;

		P.cP = x[i] * sP[i];
		//lP.modify(P);
		cP[i].resize(3);
		cP[i] = P.cP;
		cP[i].resize(3,false);

		oP[i].resize(4);
		oP[i] = P.oP;	
		oP[i].resize(3,false);

		lP.next();
		i++;
	}

	//Calcul de la matrice de transformation du repère objet/monde au repère caméra cMo
	//Calcul des 4 bases différentes possibles avec 4 points dans les deux repères pour calculer la rotation d'abord
	vpMatrix cBase(3,12), oBase(3,12);
	vpColVector X, Y, Z;
	int ix, iy, iz;
	//Repère caméra
	//Premiere base
	X = cP[1]-cP[0]; Y = cP[2]-cP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = 0 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
		{ cBase[i][ix] = X[i]; cBase[i][iy] = Y[i]; cBase[i][iz] = Z[i]; }

	//Deuxieme base
	X = cP[2]-cP[1]; Y = cP[3]-cP[1];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
		{ cBase[i][ix] = X[i]; cBase[i][iy] = Y[i]; cBase[i][iz] = Z[i]; }
		
	//Troisieme base
	X = cP[1]-cP[0]; Y = cP[3]-cP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
		{ cBase[i][ix] = X[i]; cBase[i][iy] = Y[i]; cBase[i][iz] = Z[i]; }

	//Quatrieme base
	X = cP[2]-cP[0]; Y = cP[3]-cP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
		{ cBase[i][ix] = X[i]; cBase[i][iy] = Y[i]; cBase[i][iz] = Z[i]; }

	//Repère objet/monde
	//Premiere base
	X = oP[1]-oP[0]; Y = oP[2]-oP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = 0 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
		{ oBase[i][ix] = X[i]; oBase[i][iy] = Y[i]; oBase[i][iz] = Z[i]; }

	//Deuxieme base
	X = oP[2]-oP[1]; Y = oP[3]-oP[1];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
		{ oBase[i][ix] = X[i]; oBase[i][iy] = Y[i]; oBase[i][iz] = Z[i]; }
		
	//Troisieme base
	X = oP[1]-oP[0]; Y = oP[3]-oP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
		{ oBase[i][ix] = X[i]; oBase[i][iy] = Y[i]; oBase[i][iz] = Z[i]; }

	//Quatrieme base
	X = oP[2]-oP[0]; Y = oP[3]-oP[0];
	Z = vpColVector::cross(X, Y);
	Y = vpColVector::cross(Z, X);
	X.normalize(); Y.normalize(); Z.normalize();
	ix = iz + 1 ; iy = ix + 1 ; iz = iy + 1;
	for(i=0 ; i < 3 ; i++)
		{ oBase[i][ix] = X[i]; oBase[i][iy] = Y[i]; oBase[i][iz] = Z[i]; }

	vpMatrix R;
	R = cBase * ( oBase.t() * (oBase * oBase.t()).inverseByLU());

	vpColVector S;
	vpMatrix U, V;

	U = R;
	U.svd(S, V);
	//elimination des facteurs d'échelle introduits par du bruit
	R = U * V.t();

	vpColVector rcP[4], mrcP(3), mcP(3), t;

	for(i = 0 ; i < 4 ; i++)
	{
		oP[i] /= fact;
		rcP[i] = R * oP[i];
		mrcP += rcP[i];
		cP[i] /= fact;
		mcP += cP[i];
	}
	mrcP *= 0.25;
	mcP *= 0.25;

	t = mcP-mrcP;
	//t /= fact; // / 1000.0; // On reprend la bonne échelle (cf. conditionnement)
	vpTranslationVector cto(t[0], t[1], t[2]);
	vpRotationMatrix cRo;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			cRo[i][j]=R[i][j];
	cMo.buildFrom(cto, cRo);

	/*vpPoseVector r(cMo);
	r.print();*/
}

void CPoseOmni::genere_g(vpMatrix & costheta, vpMatrix & d, int nums[3], double *A)
{
	int i = nums[0]+1, j = nums[1]+1, k = nums[2]+1;

	A[0] = -0.12e2 * pow(d[k - 1][j - 1], 0.2e1) * pow(d[(int) i - 1][j - 1], 0.4e1) * pow(d[(int) i - 1][k - 1], 0.2e1) + 0.16e2 * pow(d[(int) i - 1][k - 1], 0.4e1) * pow((costheta[k - 1][j - 1]), 0.4e1) * pow(d[(int) i - 1][j - 1], 0.4e1) + 0.4e1 * pow(d[(int) i - 1][k - 1], 0.6e1) * pow(d[(int) i - 1][j - 1], 0.2e1) - 0.4e1 * pow(d[(int) i - 1][k - 1], 0.6e1) * pow(d[k - 1][j - 1], 0.2e1) + 0.6e1 * pow(d[k - 1][j - 1], 0.4e1) * pow(d[(int) i - 1][j - 1], 0.4e1) - 0.4e1 * pow(d[k - 1][j - 1], 0.6e1) * pow(d[(int) i - 1][j - 1], 0.2e1) - 0.4e1 * pow(d[k - 1][j - 1], 0.2e1) * pow(d[(int) i - 1][j - 1], 0.6e1) + pow(d[(int) i - 1][j - 1], 0.8e1) + pow(d[(int) i - 1][k - 1], 0.8e1) + pow(d[k - 1][j - 1], 0.8e1) - 0.4e1 * pow(d[(int) i - 1][k - 1], 0.2e1) * pow(d[k - 1][j - 1], 0.6e1) + 0.6e1 * pow(d[(int) i - 1][k - 1], 0.4e1) * pow(d[k - 1][j - 1], 0.4e1) + 0.4e1 * pow(d[(int) i - 1][j - 1], 0.6e1) * pow(d[(int) i - 1][k - 1], 0.2e1) + 0.6e1 * pow(d[(int) i - 1][k - 1], 0.4e1) * pow(d[(int) i - 1][j - 1], 0.4e1) + 0.12e2 * pow(d[k - 1][j - 1], 0.4e1) * pow(d[(int) i - 1][j - 1], 0.2e1) * pow(d[(int) i - 1][k - 1], 0.2e1) - 0.12e2 * pow(d[k - 1][j - 1], 0.2e1) * pow(d[(int) i - 1][j - 1], 0.2e1) * pow(d[(int) i - 1][k - 1], 0.4e1) - 0.8e1 * pow(d[(int) i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][j - 1]), 0.2e1) * pow(d[(int) i - 1][j - 1], 0.6e1) - 0.8e1 * pow(d[(int) i - 1][k - 1], 0.6e1) * pow((costheta[k - 1][j - 1]), 0.2e1) * pow(d[(int) i - 1][j - 1], 0.2e1) - 0.16e2 * pow(d[(int) i - 1][k - 1], 0.4e1) * pow((costheta[k - 1][j - 1]), 0.2e1) * pow(d[(int) i - 1][j - 1], 0.4e1) - 0.8e1 * pow(d[(int) i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][j - 1]), 0.2e1) * pow(d[(int) i - 1][j - 1], 0.2e1) * pow(d[k - 1][j - 1], 0.4e1) + 0.16e2 * pow(d[(int) i - 1][k - 1], 0.4e1) * pow(d[k - 1][j - 1], 0.2e1) * pow((costheta[k - 1][j - 1]), 0.2e1) * pow(d[(int) i - 1][j - 1], 0.2e1) + 0.16e2 * pow(d[k - 1][j - 1], 0.2e1) * pow(d[(int) i - 1][j - 1], 0.4e1) * pow(d[(int) i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][j - 1]), 0.2e1);

	A[1] = -0.32e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[k - 1][(int) j - 1]), 0.4e1) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.48e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.8e1 * pow(d[i - 1][(int) j - 1], 0.6e1) - 0.8e1 * pow(d[i - 1][k - 1], 0.6e1) + 0.8e1 * pow(d[k - 1][(int) j - 1], 0.6e1) + 0.24e2 * pow(d[i - 1][(int) j - 1], 0.4e1) * pow(d[k - 1][(int) j - 1], 0.2e1) - 0.24e2 * pow(d[i - 1][(int) j - 1], 0.4e1) * pow(d[i - 1][k - 1], 0.2e1) - 0.24e2 * pow(d[k - 1][(int) j - 1], 0.4e1) * pow(d[i - 1][k - 1], 0.2e1) + 0.24e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.4e1) - 0.24e2 * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.4e1) - 0.24e2 * pow(d[k - 1][(int) j - 1], 0.4e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.8e1 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) + 0.8e1 * pow(d[i - 1][k - 1], 0.6e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) - 0.8e1 * pow(d[k - 1][(int) j - 1], 0.6e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.8e1 * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.6e1) + 0.8e1 * pow(d[i - 1][k - 1], 0.6e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) - 0.8e1 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.6e1) + 0.8e1 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.6e1) + 0.8e1 * pow(d[i - 1][k - 1], 0.6e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) + 0.8e1 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.6e1) - 0.8e1 * pow(d[i - 1][k - 1], 0.2e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.4e1) - 0.64e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.32e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) * pow(d[k - 1][(int) j - 1], 0.2e1) - 0.32e2 * pow((costheta[k - 1][(int) j - 1]), 0.4e1) * pow(d[i - 1][(int) j - 1], 0.4e1) * pow(d[i - 1][k - 1], 0.2e1) + 0.24e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.8e1 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) + 0.16e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.16e2 * pow(d[i - 1][k - 1], 0.4e1) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) - 0.16e2 * pow(d[i - 1][k - 1], 0.6e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.16e2 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) + 0.8e1 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.24e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) * pow(d[k - 1][(int) j - 1], 0.2e1) + 0.16e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) * pow(d[i - 1][k - 1], 0.2e1) + 0.16e2 * pow(d[k - 1][(int) j - 1], 0.4e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.8e1 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) + 0.56e2 * pow(d[i - 1][(int) j - 1], 0.4e1) * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) - 0.16e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) - 0.16e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.6e1) - 0.8e1 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) + 0.24e2 * pow(d[k - 1][(int) j - 1], 0.4e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) - 0.24e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][k - 1], 0.4e1) + 0.8e1 * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) + 0.8e1 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) + 0.56e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.8e1 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.4e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.16e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.32e2 * (costheta[i - 1][k - 1]) * pow((costheta[k - 1][(int) j - 1]), 0.3e1) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.32e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.40e2 * pow(d[i - 1][k - 1], 0.4e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.32e2 * (costheta[i - 1][k - 1]) * pow((costheta[k - 1][(int) j - 1]), 0.3e1) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][(int) j - 1], 0.4e1) * pow(d[i - 1][k - 1], 0.2e1) + 0.8e1 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][(int) j - 1], 0.6e1) - 0.16e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) - 0.16e2 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) - 0.16e2 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) - 0.32e2 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.48e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) - 0.8e1 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.4e1) + 0.32e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) - 0.40e2 * pow(d[i - 1][k - 1], 0.2e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][(int) j - 1], 0.4e1) + 0.32e2 * pow(d[i - 1][k - 1], 0.4e1) * (costheta[i - 1][k - 1]) * pow((costheta[k - 1][(int) j - 1]), 0.3e1) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][(int) j - 1], 0.2e1);

	A[2] = -0.40e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.48e2 * pow(d[i - 1][k - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.48e2 * pow(d[i - 1][k - 1], 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) - 0.32e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) - 0.32e2 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * pow((costheta[i - 1][(int) j - 1]), 0.3e1) * pow(d[k - 1][(int) j - 1], 0.4e1) - 0.40e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) - 0.48e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.24e2 * pow(d[k - 1][(int) j - 1], 0.4e1) + 0.16e2 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][(int) j - 1], 0.4e1) + 0.24e2 * pow(d[i - 1][k - 1], 0.4e1) + 0.24e2 * pow(d[i - 1][(int) j - 1], 0.4e1) - 0.8e1 * pow(d[i - 1][(int) j - 1], 0.4e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.16e2 * pow(d[k - 1][(int) j - 1], 0.4e1) * pow((costheta[i - 1][(int) j - 1]), 0.4e1) + 0.16e2 * pow((costheta[i - 1][k - 1]), 0.4e1) * pow(d[i - 1][(int) j - 1], 0.4e1) - 0.40e2 * pow(d[i - 1][(int) j - 1], 0.4e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) + 0.48e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) + 0.16e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) - 0.32e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.4e1) * pow(d[i - 1][k - 1], 0.2e1) + 0.80e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][k - 1]), 0.2e1) + 0.48e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.48e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) - 0.32e2 * pow((costheta[i - 1][k - 1]), 0.4e1) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.16e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) + 0.48e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.64e2 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.4e1) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.16e2 * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) + 0.16e2 * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.4e1) + 0.48e2 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) - 0.48e2 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.16e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) + 0.16e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.48e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) + 0.80e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.48e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.112e3 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.48e2 * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) - 0.32e2 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * pow((costheta[i - 1][(int) j - 1]), 0.3e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) + 0.64e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.3e1) * pow(d[i - 1][k - 1], 0.2e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) - 0.64e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) + 0.160e3 * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) - 0.40e2 * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][k - 1], 0.4e1) + 0.16e2 * pow((costheta[i - 1][k - 1]), 0.4e1) * pow(d[k - 1][(int) j - 1], 0.4e1) + 0.32e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.32e2 * (costheta[i - 1][k - 1]) * pow((costheta[k - 1][(int) j - 1]), 0.3e1) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.32e2 * pow((costheta[i - 1][k - 1]), 0.3e1) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.4e1) - 0.40e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) - 0.40e2 * pow(d[k - 1][(int) j - 1], 0.4e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) - 0.8e1 * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.4e1) - 0.8e1 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[i - 1][k - 1]), 0.2e1) + 0.16e2 * pow((costheta[k - 1][(int) j - 1]), 0.4e1) * pow(d[i - 1][(int) j - 1], 0.4e1) + 0.16e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[k - 1][(int) j - 1]), 0.4e1) + 0.16e2 * pow(d[i - 1][k - 1], 0.4e1) * pow((costheta[i - 1][(int) j - 1]), 0.4e1) - 0.32e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) - 0.64e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.32e2 * pow(d[i - 1][k - 1], 0.4e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * pow((costheta[i - 1][(int) j - 1]), 0.3e1) - 0.32e2 * pow(d[i - 1][k - 1], 0.4e1) * (costheta[i - 1][k - 1]) * pow((costheta[k - 1][(int) j - 1]), 0.3e1) * (costheta[i - 1][(int) j - 1]) + 0.16e2 * pow(d[i - 1][k - 1], 0.4e1) * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) + 0.64e2 * pow((costheta[i - 1][k - 1]), 0.3e1) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.32e2 * pow((costheta[i - 1][k - 1]), 0.3e1) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][k - 1], 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) - 0.32e2 * pow((costheta[i - 1][k - 1]), 0.3e1) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) + 0.32e2 * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) - 0.32e2 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * pow((costheta[i - 1][(int) j - 1]), 0.3e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) - 0.32e2 * (costheta[i - 1][k - 1]) * pow((costheta[k - 1][(int) j - 1]), 0.3e1) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][(int) j - 1], 0.4e1) - 0.64e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) - 0.128e3 * pow(d[i - 1][k - 1], 0.2e1) * (costheta[i - 1][k - 1]) * pow((costheta[k - 1][(int) j - 1]), 0.3e1) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][(int) j - 1], 0.2e1) + 0.16e2 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.4e1) + 0.64e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) - 0.64e2 * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[i - 1][k - 1]), 0.2e1) + 0.64e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.64e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow(d[i - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) - 0.32e2 * pow((costheta[i - 1][k - 1]), 0.3e1) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) * pow(d[i - 1][(int) j - 1], 0.4e1) - 0.32e2 * (costheta[i - 1][k - 1]) * pow((costheta[k - 1][(int) j - 1]), 0.3e1) * (costheta[i - 1][(int) j - 1]) * pow(d[k - 1][(int) j - 1], 0.2e1) * pow(d[i - 1][k - 1], 0.2e1);

	A[3] = 0.64e2 * pow(d[i - 1][j - 1], 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) - 0.64e2 * pow(d[(int) k - 1][j - 1], 0.2e1) * pow((costheta[i - 1][(int) k - 1]), 0.2e1) + 0.32e2 * pow(d[i - 1][j - 1], 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) - 0.32e2 * pow((costheta[(int) k - 1][j - 1]), 0.4e1) * pow(d[i - 1][(int) k - 1], 0.2e1) - 0.64e2 * pow(d[(int) k - 1][j - 1], 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) - 0.32e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * pow((costheta[i - 1][j - 1]), 0.4e1) + 0.32e2 * pow(d[(int) k - 1][j - 1], 0.2e1) * pow((costheta[i - 1][j - 1]), 0.4e1) + 0.64e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) - 0.32e2 * pow((costheta[i - 1][(int) k - 1]), 0.4e1) * pow(d[i - 1][j - 1], 0.2e1) + 0.32e2 * pow((costheta[i - 1][(int) k - 1]), 0.4e1) * pow(d[(int) k - 1][j - 1], 0.2e1) - 0.32e2 * pow((costheta[(int) k - 1][j - 1]), 0.4e1) * pow(d[i - 1][j - 1], 0.2e1) + 0.32e2 * (costheta[i - 1][(int) k - 1]) * pow((costheta[(int) k - 1][j - 1]), 0.3e1) * (costheta[i - 1][j - 1]) * pow(d[(int) k - 1][j - 1], 0.2e1) - 0.128e3 * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) * pow(d[(int) k - 1][j - 1], 0.2e1) + 0.96e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * (costheta[i - 1][(int) k - 1]) * (costheta[(int) k - 1][j - 1]) * pow((costheta[i - 1][j - 1]), 0.3e1) + 0.32e2 * pow(d[(int) k - 1][j - 1], 0.2e1) * (costheta[i - 1][(int) k - 1]) * (costheta[(int) k - 1][j - 1]) * (costheta[i - 1][j - 1]) - 0.96e2 * pow(d[i - 1][j - 1], 0.2e1) * (costheta[i - 1][(int) k - 1]) * (costheta[(int) k - 1][j - 1]) * (costheta[i - 1][j - 1]) - 0.64e2 * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) * pow(d[i - 1][j - 1], 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) + 0.128e3 * pow((costheta[i - 1][(int) k - 1]), 0.3e1) * (costheta[(int) k - 1][j - 1]) * pow((costheta[i - 1][j - 1]), 0.3e1) * pow(d[(int) k - 1][j - 1], 0.2e1) + 0.32e2 * pow(d[(int) k - 1][j - 1], 0.2e1) + 0.64e2 * pow((costheta[(int) k - 1][j - 1]), 0.2e1) * pow(d[i - 1][(int) k - 1], 0.2e1) - 0.64e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) - 0.32e2 * (costheta[i - 1][(int) k - 1]) * (costheta[(int) k - 1][j - 1]) * pow((costheta[i - 1][j - 1]), 0.3e1) * pow(d[(int) k - 1][j - 1], 0.2e1) + 0.32e2 * (costheta[i - 1][(int) k - 1]) * (costheta[(int) k - 1][j - 1]) * pow((costheta[i - 1][j - 1]), 0.3e1) * pow(d[i - 1][j - 1], 0.2e1) - 0.32e2 * pow((costheta[i - 1][(int) k - 1]), 0.3e1) * (costheta[(int) k - 1][j - 1]) * (costheta[i - 1][j - 1]) * pow(d[(int) k - 1][j - 1], 0.2e1) - 0.32e2 * pow(d[i - 1][j - 1], 0.2e1) - 0.32e2 * pow(d[i - 1][(int) k - 1], 0.2e1) - 0.32e2 * pow(d[(int) k - 1][j - 1], 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) + 0.32e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * pow((costheta[i - 1][(int) k - 1]), 0.2e1) + 0.64e2 * pow(d[i - 1][j - 1], 0.2e1) * pow((costheta[i - 1][(int) k - 1]), 0.2e1) - 0.64e2 * pow((costheta[i - 1][(int) k - 1]), 0.4e1) * pow(d[(int) k - 1][j - 1], 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) - 0.32e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) - 0.64e2 * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) * pow(d[i - 1][j - 1], 0.2e1) + 0.128e3 * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) * pow(d[(int) k - 1][j - 1], 0.2e1) - 0.32e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) - 0.64e2 * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[i - 1][j - 1]), 0.4e1) * pow(d[(int) k - 1][j - 1], 0.2e1) + 0.32e2 * pow(d[(int) k - 1][j - 1], 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) - 0.32e2 * pow((costheta[(int) k - 1][j - 1]), 0.2e1) * pow(d[i - 1][j - 1], 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) - 0.64e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) + 0.96e2 * pow((costheta[i - 1][(int) k - 1]), 0.3e1) * (costheta[(int) k - 1][j - 1]) * (costheta[i - 1][j - 1]) * pow(d[i - 1][j - 1], 0.2e1) + 0.32e2 * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[(int) k - 1][j - 1]), 0.2e1) * pow(d[(int) k - 1][j - 1], 0.2e1) + 0.32e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * pow((costheta[i - 1][(int) k - 1]), 0.3e1) * (costheta[(int) k - 1][j - 1]) * (costheta[i - 1][j - 1]) - 0.96e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * (costheta[i - 1][(int) k - 1]) * (costheta[(int) k - 1][j - 1]) * (costheta[i - 1][j - 1]) + 0.96e2 * (costheta[i - 1][(int) k - 1]) * pow((costheta[(int) k - 1][j - 1]), 0.3e1) * (costheta[i - 1][j - 1]) * pow(d[i - 1][j - 1], 0.2e1) - 0.32e2 * pow((costheta[i - 1][(int) k - 1]), 0.2e1) * pow((costheta[i - 1][j - 1]), 0.2e1) * pow(d[i - 1][j - 1], 0.2e1) + 0.96e2 * pow(d[i - 1][(int) k - 1], 0.2e1) * (costheta[i - 1][(int) k - 1]) * pow((costheta[(int) k - 1][j - 1]), 0.3e1) * (costheta[i - 1][j - 1]);

	A[4] = 0.16e2 * pow((costheta[i - 1][k - 1]), 0.4e1) + 0.16e2 * pow((costheta[i - 1][(int) j - 1]), 0.4e1) - 0.32e2 * pow((costheta[k - 1][(int) j - 1]), 0.2e1) - 0.32e2 * pow((costheta[i - 1][(int) j - 1]), 0.2e1) - 0.32e2 * pow((costheta[i - 1][k - 1]), 0.2e1) + 0.16e2 * pow((costheta[k - 1][(int) j - 1]), 0.4e1) + 0.16e2 + 0.32e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.32e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) + 0.32e2 * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) + 0.64e2 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) - 0.64e2 * (costheta[i - 1][k - 1]) * pow((costheta[k - 1][(int) j - 1]), 0.3e1) * (costheta[i - 1][(int) j - 1]) - 0.64e2 * pow((costheta[i - 1][k - 1]), 0.3e1) * (costheta[k - 1][(int) j - 1]) * (costheta[i - 1][(int) j - 1]) + 0.64e2 * pow((costheta[i - 1][k - 1]), 0.2e1) * pow((costheta[k - 1][(int) j - 1]), 0.2e1) * pow((costheta[i - 1][(int) j - 1]), 0.2e1) - 0.64e2 * (costheta[i - 1][k - 1]) * (costheta[k - 1][(int) j - 1]) * pow((costheta[i - 1][(int) j - 1]), 0.3e1);

}


#endif
