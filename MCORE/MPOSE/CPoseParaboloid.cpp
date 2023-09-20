
/*!
  \file CPoseParaboloid.cpp
  \brief Compute the pose using virtual visual servoing approach with omnidirectional (Paraboloid) images
*/

#include <MIXEDVISION/CPoseParaboloid.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

//#define AFFDETAILSTXT

/*!
  \brief Compute the pose using virtual visual servoing approach on omnidirectional images

*/

CPoseParaboloid::CPoseParaboloid()
{
}

CPoseParaboloid::~CPoseParaboloid()
{
}

void CPoseParaboloid::computeJacobianForVVS(CPoint & P, CModel *cam, vpMatrix & Ls)
{
	double x = P.get_x(), y = P.get_y(), z;
	double Z = P.get_Z(), h = ((CParaboloid *)cam)->geth();

	double X = P.get_X(), Y = P.get_Y();
	double rho = sqrt(X*X+Y*Y+Z*Z);

	/*z = h*Z / (rho+Z);

	Ls.resize(2,6);

	Ls[0][0] = -( (y*y+z*z)/(rho*h) + z/(rho+Z) ) ;
	Ls[0][1] = x*y/(h*rho);
	Ls[0][2] = x / rho;
	Ls[0][3] = x*y/h;
	Ls[0][4] = -( (x*x+z*z)/h + rho*z/(rho+Z) );
	Ls[0][5] = y;

	Ls[1][0] = x*y/(h*rho);
	Ls[1][1] = -( (x*x + z*z)/(h*rho) + z/(rho+Z) );
	Ls[1][2] = y / rho;
	Ls[1][3] = (y*y+z*z)/h + rho*z/(rho+Z);
	Ls[1][4] = -x*y/h;
	Ls[1][5] = -x;*/
	
	/*z = h*Z / (rho-Z);
	 
	 Ls.resize(2,6);
	 
	 Ls[0][0] = -((y*y+z*z)/(h*rho) + z/(rho-Z)) ;
	 Ls[0][1] = x*y/(h*rho);
	 Ls[0][2] = -x / rho;
	 Ls[0][3] = -x*y/h;
	 Ls[0][4] = -Z*(y*y+z*z+x*x)/(h*rho) + (x*x+z*z)/h;
	 Ls[0][5] = -(Y/rho) * ( z*rho/(rho-Z) - (x*x+y*y+z*z)/h );
	 
	 Ls[1][0] = x*y/(h*rho);
	 Ls[1][1] = -(x*x + z*z)/(h*rho) + z/(rho-Z);
	 Ls[1][2] = -y / rho;
	 Ls[1][3] = Z*(x*x+y*y+z*z)/(h*rho) - (y*y+z*z)/h;
	 Ls[1][4] = x*y/h;
	 Ls[1][5] = (X/rho) * (z*rho/(rho-Z) - (x*x+y*y+z*z)/h);
	 */
	
	z = h*Z/(rho+Z);
	
	Ls.resize(2,6);
	
	Ls[0][0] = -( (y*y+z*z)/(h*rho) + z/(rho+Z) );
	Ls[0][1] = x*y/(h*rho);
	Ls[0][2] = x / rho;
	Ls[0][3] = x*y/h;
	Ls[0][4] = -( (x*x+z*z)/h + (rho*z)/(rho+Z) );
	Ls[0][5] = y;
	
	Ls[1][0] = x*y/(h*rho);
	Ls[1][1] = -( (x*x + z*z)/(h*rho) + z/(rho+Z) );
	Ls[1][2] = y / rho;
	Ls[1][3] = ( (y*y+z*z)/h + (rho*z)/(rho+Z) );
	Ls[1][4] = -x*y/h;
	Ls[1][5] = -x;
}

int CPoseParaboloid::initViewLines(CModel *cam, vpColVector *sP, int nbPts)
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
		
		((CParaboloid *)cam)->projectImageMiroir(P, sP[i][0], sP[i][1], sP[i][2]);
		sP[i].normalize();
		
		listP.next() ;
		i++;
	}
	
	return 0;
}

/*void
CPoseParaboloid::displayFrame(vpImage<unsigned char> &I,
						vpHomogeneousMatrix &cMo,
						CModel *_cam,
						double size,
						vpColor col)
{
	CParaboloid *cam = (CParaboloid *)_cam;
	
	vpColVector Xs0(3), Xs(3), N(3), a(5);
	// used by display
	CPoint o; o.setWorldCoordinates ( 0.0,0.0,0.0 ) ;
	CPoint x; x.setWorldCoordinates ( size,0.0,0.0 ) ;
	CPoint y; y.setWorldCoordinates ( 0.0,size,0.0 ) ;
	CPoint z; z.setWorldCoordinates ( 0.0,0.0,size ) ;
	
	o.changeFrame(cMo) ;
	cam->project3DMiroir(o, Xs0[0], Xs0[1], Xs0[2]);
	Xs0.normalize();
	cam->project3DImage(o);
	cam->meterPixelConversion(o);
	
	x.changeFrame(cMo) ;
	cam->project3DMiroir(x, Xs[0], Xs[1], Xs[2]);
	Xs.normalize();
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
	cam->project3DMiroir(y, Xs[0], Xs[1], Xs[2]);
	Xs.normalize();
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
	cam->project3DMiroir(z, Xs[0], Xs[1], Xs[2]);
	Xs.normalize();
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
CPoseParaboloid::computeNormalFromTwoPoints(vpColVector & Xs0, vpColVector & Xs, vpColVector & N)
{
	N = vpColVector::cross(Xs0, Xs);
}

void 
CPoseParaboloid::computeConicFromNormal(vpColVector & N, vpColVector & a, CParaboloid *cam)
{		
	a.resize(5);
	
	a[0] = -1;
	a[1] = -1;
	a[2] = 0;
	a[3] = N[0]/N[2];
	a[4] = N[1]/N[2];
}

/*void
CPoseParaboloid::displayConic(vpImage<unsigned char>&I,
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
CPoseParaboloid::computeConic2Circle(vpColVector a)
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
	/*CvMat CMat = cvMat(3, 3, CV_64F, C.data), 
	EigValsMat = cvMat(3, 1, CV_64F, EigVals.data), 
	EigVectsMat = cvMat(3, 3, CV_64F, EigVects.data);
	cvEigenVV(&CMat, &EigVectsMat, &EigValsMat, DBL_EPSILON);
	EigVects = EigVects.t();*/
	
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
CPoseParaboloid::OutOfImage(int i, int j, int half, int rows, int cols)
{
	return (! ((i> half+2) &&
			   (i< rows -(half+2)) &&
			   (j>half+2) &&
			   (j<cols-(half+2)))
			) ;
}
