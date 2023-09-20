
/*!
  \file vpPoseOmniVirtualVisualServoing.cpp
  \brief Compute the pose using virtual visual servoing approach with omnidirectional images
*/

#include <MIXEDVISION/CPose.h>

#include <visp/vpExponentialMap.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>
/*!
  \brief Compute the pose using virtual visual servoing approach on omnidirectional images

*/

CPose::CPose()
{
	init();
}

CPose::~CPose()
{
}

void CPose::setLambda(double a)
{
	lambda = a;
}

void CPose::setMu(double a)
{
	muRef = a;
}

void CPose::setVvsIterMax(int nb)
{
	vvsIterMax = nb;
}

void CPose::setCamera(CModel *_cam)
{
    if(_cam != NULL)
        cam = _cam;
}

int CPose::get_npt()
{
	return npt;
}

void CPose::init()
{
	clearPoint();

	lambda = 0.1 ;
	
	muRef = 0.01;

	vvsIterMax = 200 ;
}

void CPose::clearPoint()
{
	listP.kill() ;
	npt = 0 ;
}

void CPose::addPoint(const CPoint& newP)
{
	listP += newP;
	npt++;
}

void CPose::setListPoints(vpList<CPoint> & lP)
{
	clearPoint();
	listP = lP;
	npt = lP.nbElements();
}

//Pour affichage
//Attention : a specialiser pour les differentes projections
void
CPose::getFrame(std::vector<SImagePoint> & ptsPose,
                vpHomogeneousMatrix &cMo,
                CModel *cam,
                double size)
{
    SImagePoint ptIm;
    ptsPose.clear();
	
	// used by display
	CPoint o; o.setWorldCoordinates ( 0.0,0.0,0.0 ) ;
	CPoint x; x.setWorldCoordinates ( size,0.0,0.0 ) ;
	CPoint y; y.setWorldCoordinates ( 0.0,size,0.0 ) ;
	CPoint z; z.setWorldCoordinates ( 0.0,0.0,size ) ;
    
	o.changeFrame(cMo) ;
    cam->project3DImage(o) ;
	cam->meterPixelConversion(o);
	x.changeFrame(cMo) ;
    cam->project3DImage(x) ;
	cam->meterPixelConversion(x);
	y.changeFrame(cMo) ;
    cam->project3DImage(y) ;
	cam->meterPixelConversion(y);
	z.changeFrame(cMo) ;
    cam->project3DImage(z) ;
	cam->meterPixelConversion(z);
    
    ptIm.u = o.get_u();
    ptIm.v = o.get_v();
    ptsPose.push_back(ptIm);
    
    ptIm.u = x.get_u();
    ptIm.v = x.get_v();
    ptsPose.push_back(ptIm);
    
    ptIm.u = y.get_u();
    ptIm.v = y.get_v();
    ptsPose.push_back(ptIm);
    
    ptIm.u = z.get_u();
    ptIm.v = z.get_v();
    ptsPose.push_back(ptIm);
}

void
CPose::poseVirtualVS(CModel *cam, vpHomogeneousMatrix & cMo)
{
	try
	{
		double  residu_1 = 1e8 ;
		double r =1e8-1;

		int iter = 0 ;

		int nb = listP.nbElement() ;
		vpMatrix L(2*nb,6), Lp;
		vpColVector err(2*nb) ;
		vpColVector sd(2*nb),s(2*nb) ;

		vpMatrix H, Hs;
		vpMatrix Id; // matrice identite
		
		vpColVector v, gradient_du_cout;
		
		listP.front() ;
		vpList<CPoint> lP ;

		CPoint P;
		
		// create sd
		int k =0 ;
		while (!listP.outside())
		{
			P = listP.value() ;
			sd[2*k] = P.get_x() ;
			sd[2*k+1] = P.get_y() ;
			lP += P ;
			listP.next() ;
			k++ ;
		}
		
		mu = muRef;
		vpMatrix Ls;
		
		while((int)((residu_1 - r)*1e12) !=0)
		{
			L = 0;
			
			residu_1 = r ;

			// Compute the interaction matrix and the error
			int k =0 ;
			lP.front() ;
			while (!lP.outside())
			{
				P = lP.value() ;

				P.changeFrame(cMo) ;
				cam->project3DImage(P) ;

				lP.modify(P); //Sauvegarde

				s[2*k] = P.get_x();  /* point projected from cMo */
				s[2*k+1] = P.get_y();

				computeJacobianForVVS(P, cam, Ls);

				for(int i = 2*k, ii = 0 ; ii < 2 ; i++, ii++)
					for(int j = 0 ; j < 6 ; j++)
						L[i][j] = Ls[ii][j];

				lP.next() ;

				k++ ;
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
			if (iter++>vvsIterMax) break ;
		}
		lP.kill() ;
	}
	catch(...)
	{
		vpERROR_TRACE(" ") ;
		throw ;
	}

}

void CPose::poseInit(vpHomogeneousMatrix & cMo, CModel *cam)
{	
	int i, j;
	CPoint P, Pbis;
	vpList<CPoint> lP;
	vpColVector sP[4];
	
	lP.kill();
	lP += listP;
	
	if(initViewLines(cam, sP, 4))
	{
		cMo.eye();
		return;
	}
	
	vpMatrix c(4,4), sqrDistance(4,4);
	
	listP.front();
	i = 0;
	while(i < 3)
	{
		P = listP.value();
		lP.front();
		lP.suppress();
		j = i+1;
		while(j < 4)
		{
			Pbis = lP.value();
			
			c[i][j] = c[j][i] = -2.0*(sP[i][0]*sP[j][0] + sP[i][1]*sP[j][1] + sP[i][2]*sP[j][2]);
			vpColVector oP = P.oP-Pbis.oP;
			oP.resize(3,false);
			sqrDistance[i][j] = sqrDistance[j][i] = oP.sumSquare();
			
			lP.next();
			j++;
		}
		listP.next();
		i++;
	}
	
	vpMatrix A(24,24);
	
	A[0][0] = 1.0; A[0][4] = c[0][1]; A[0][7] = 1.0;      A[0][20] = sqrDistance[0][1];
	A[1][1] = 1.0; A[1][4] = 1.0;     A[1][7] = c[0][1];  A[1][21] = sqrDistance[0][1];
	A[2][5] = 1.0; A[2][8] = 1.0;     A[2][16] = c[0][1]; A[2][22] = sqrDistance[0][1];
	A[3][6] = 1.0; A[3][9] = 1.0;     A[3][17] = c[0][1]; A[3][23] = sqrDistance[0][1];
	
	A[4][0] = 1.0; A[4][5] = c[0][2];  A[4][11] = 1.0;      A[4][20] = sqrDistance[0][2];
	A[5][4] = 1.0; A[5][10] = 1.0;     A[5][16] = c[0][2];  A[5][21] = sqrDistance[0][2];
	A[6][2] = 1.0; A[6][5] = 1.0;      A[6][11] = c[0][2];  A[6][22] = sqrDistance[0][2];
	A[7][6] = 1.0; A[7][12] = 1.0;     A[7][18] = c[0][2];  A[7][23] = sqrDistance[0][2];
	
	A[8][0] = 1.0;  A[8][6] = c[0][3];    A[8][15] = 1.0;       A[8][20] = sqrDistance[0][3];
	A[9][4] = 1.0;  A[9][13] = 1.0;       A[9][17] = c[0][3];   A[9][21] = sqrDistance[0][3];
	A[10][5] = 1.0; A[10][14] = 1.0;      A[10][18] = c[0][3];  A[10][22] = sqrDistance[0][3];
	A[11][3] = 1.0; A[11][6] = 1.0;       A[11][15] = c[0][3];  A[11][23] = sqrDistance[0][3];
	
	A[12][7] = 1.0;  A[12][11] = 1.0;      A[12][16] = c[1][2];   A[12][20] = sqrDistance[1][2];
	A[13][1] = 1.0;  A[13][8] = c[1][2];   A[13][10] = 1.0;       A[13][21] = sqrDistance[1][2];
	A[14][2] = 1.0;  A[14][8] = 1.0;       A[14][10] = c[1][2];   A[14][22] = sqrDistance[1][2];
	A[15][9] = 1.0;  A[15][12] = 1.0;      A[15][19] = c[1][2];   A[15][23] = sqrDistance[1][2];
	
	A[16][7] = 1.0;  A[16][15] = 1.0;     A[16][17] = c[1][3];   A[16][20] = sqrDistance[1][3];
	A[17][1] = 1.0;  A[17][9] = c[1][3];  A[17][13] = 1.0;       A[17][21] = sqrDistance[1][3];
	A[18][8] = 1.0;  A[18][14] = 1.0;     A[18][19] = c[1][3];   A[18][22] = sqrDistance[1][3];
	A[19][3] = 1.0;  A[19][9] = 1.0;      A[19][13] = c[1][3];   A[19][23] = sqrDistance[1][3];
	
	A[20][11] = 1.0;    A[20][15] = 1.0; A[20][17] = c[2][3];   A[20][20] = sqrDistance[2][3];
	A[21][9] = c[2][3]; A[21][10] = 1.0; A[21][13] = 1.0;       A[21][21] = sqrDistance[2][3];
	A[22][2] = 1.0;     A[22][14] = 1.0; A[22][19] = c[2][3];   A[22][22] = sqrDistance[2][3];
	A[23][3] = 1.0;     A[23][12] = 1.0; A[23][13] = c[2][3];   A[23][23] = sqrDistance[2][3];
	
	vpColVector Sa(A.getRows());
	vpMatrix Va(A.getRows(), A.getCols());
	
#ifdef VISP_HAVE_GSL
	A.svd(Sa, Va);
#else
	//on utilise la svd de OpenCV
	CvMat AMat=cvMat(A.getRows(), A.getCols(), CV_64F, A.data), 
		  SaMat=cvMat(Sa.getRows(), 1, CV_64F, Sa.data), 
		  VaMat=cvMat(Va.getRows(), Va.getCols(), CV_64F, Va.data);
	
	cvSVD(&AMat, &SaMat, NULL, &VaMat, CV_SVD_MODIFY_A);
#endif
	
	//std::cout << "Sa: " << Sa << std::endl;
	//exit(9);
	
	// Pour chaque point, on recherche sa distance au centre de projection
	double x[4] = {0.0,0.0,0.0,0.0};
	
	for(i = 0 ; i < 4 ; i++)
		x[i] = sqrt(fabs(Va[i][23] / Va[20+i][23]));
	
	//std::cout << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << std::endl;
	
	vpColVector cP[4], oP[4];
	// Calcul des 4 points dans le repère caméra
	listP.front();
	i = 0;
	while (i < 4)
	{
		P = listP.value() ;
		
		P.cP = x[i] * sP[i];
		//lP.modify(P);
		cP[i].resize(4);
		cP[i] = P.cP;
		cP[i].resize(3,false);
		
		oP[i].resize(4);
		oP[i] = P.oP;	
		oP[i].resize(3,false);
		
		listP.next();
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
	
	vpColVector S(R.getRows());
	vpMatrix U(R.getRows(), R.getCols()), V(R.getRows(), R.getCols());
	
#ifdef VISP_HAVE_GSL
	U = R;
	U.svd(S, V);
#else
	//on utilise la svd de OpenCV
	CvMat RMat=cvMat(R.getRows(), R.getCols(), CV_64F, R.data), 
	UMat=cvMat(U.getRows(), U.getCols(), CV_64F, U.data),
	SMat=cvMat(S.getRows(), 1, CV_64F, S.data), 
	VMat=cvMat(V.getRows(), V.getCols(), CV_64F, V.data);
	
	cvSVD(&RMat, &SMat, &UMat, &VMat, CV_SVD_MODIFY_A);
#endif
	//elimination des facteurs d'échelle introduits par du bruit
	R = U * V.t();
	
	vpColVector rcP[4], mrcP(3), mcP(3), t;
	for(i = 0 ; i < 4 ; i++)
	{
		rcP[i] = R * oP[i];
		mrcP += rcP[i];
		mcP += cP[i];
	}
	mrcP *= 0.25;
	mcP *= 0.25;
	
	t = mcP-mrcP;
	
	vpTranslationVector cto(t[0], t[1], t[2]);
	vpRotationMatrix cRo;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			cRo[i][j]=R[i][j];
	cMo.buildFrom(cto, cRo);
	
	/*vpPoseVector r(cMo);
	 r.print();*/
}
