#include <MIXEDVISION/CPosePerspective.h>

CPosePerspective::CPosePerspective()
{
}

CPosePerspective::~CPosePerspective()
{
}

void CPosePerspective::computeJacobianForVVS(CPoint & P, CModel *cam, vpMatrix & Ls)
{
	double x = P.get_x(), y = P.get_y();
	double Z = P.get_Z(), iZ;
	
	iZ = 1.0/Z;
	
	Ls.resize(2,6);
	
	Ls[0][0] = -iZ;
	Ls[0][1] = 0;
	Ls[0][2] = x * iZ;
	Ls[0][3] = x * y;
	Ls[0][4] = -(1.0+x*x);
	Ls[0][5] = y;
	
	Ls[1][0] = 0;
	Ls[1][1] = -iZ;
	Ls[1][2] = y * iZ;
	Ls[1][3] = (1+y*y);
	Ls[1][4] = -x * y;
	Ls[1][5] = -x;
}
/*
int CPosePerspective::initViewLines(CModel *cam, vpColVector *sP)
{
	int i = 0, nbP = listP.nbElement();
	CPoint P;
	double inrm;
	
	listP.front();
	while (i < nbP)
	{
		P = listP.value() ;
		
		sP[i] = vpColVector(3);
		
		//((COmni *)cam)->projectImageSphere(P, sP[i][0], sP[i][1], sP[i][2]);
		inrm = 1.0/sqrt(P.get_x()*P.get_x() + P.get_y()*P.get_y() + 1.0);
		sP[i][0] = P.get_x()*inrm;
		sP[i][1] = P.get_y()*inrm;
		sP[i][2] = inrm;
		
		listP.next() ;
		i++;
	}
	
	return 0;
}
*/

static void calculTranslation(vpMatrix &a, vpMatrix &b, int nl, int nc1,int nc3, vpColVector &x1, vpColVector &x2)
{
    int i,j;

    vpMatrix ct(3,nl) ;
    for (i=0 ; i < 3 ; i++)
    {
		for (j=0 ; j < nl ; j++)
		ct[i][j] = b[j][i+nc3] ;
    }

    vpMatrix c ;
    c = ct.t() ;

    vpMatrix ctc ;
    ctc = ct*c ;

    vpMatrix ctc1 ; // (C^T C)^(-1)
    ctc1 = ctc.inverseByLU() ;

    vpMatrix cta ;
    vpMatrix ctb ;
    cta = ct*a ;  // C^T A	
    ctb = ct*b ;  // C^T B	

    vpColVector X2(nc3)  ;
    vpMatrix CTB(nc1,nc3) ;
    for (i=0 ; i < nc1 ; i++)
    {
		for (j=0 ; j < nc3 ; j++)
		CTB[i][j] = ctb[i][j] ;
    }

    for (j=0 ; j < nc3 ; j++) X2[j] = x2[j] ;

    vpColVector sv ;       // C^T A X1 + C^T B X2)
    sv = cta*x1 + CTB*X2 ;// C^T A X1 + C^T B X2)

    vpColVector X3 ; // X3 = - (C^T C )^{-1} C^T (A X1 + B X2) 
    X3 = -ctc1*sv ;

    for (i=0 ; i < nc1 ; i++) x2[i+nc3] = X3[i] ;
}

static void lagrange (vpMatrix &a, vpMatrix &b, vpColVector &x1, vpColVector &x2)
{
    int i,imin;

    vpMatrix ata ; // A^T A
    ata = a.t()*a ;
    vpMatrix btb ; // B^T B
    btb = b.t()*b ;

    vpMatrix bta ;  // B^T A
    bta = b.t()*a ;

    vpMatrix btb1 ;  // (B^T B)^(-1)

    if (b.getRows() >= b.getCols()) btb1 = btb.inverseByLU() ;
    else btb1 = btb.pseudoInverse();

    vpMatrix r ;  // (B^T B)^(-1) B^T A
    r = btb1*bta ;

    vpMatrix e ;  //   - A^T B (B^T B)^(-1) B^T A
    e = - (a.t()*b) *r ;

    e += ata ; // calcul E = A^T A - A^T B (B^T B)^(-1) B^T A

    //   vpColVector sv ;
    //    vpMatrix v ;
    e.svd(x1,ata) ;// destructif sur e
    // calcul du vecteur propre de E correspondant a la valeur propre min.

    // calcul de SVmax	
    imin = 0;
    // FC : Pourquoi calculer SVmax ??????
    //     double  svm = 0.0;
    //    for (i=0;i<x1.getRows();i++)
    //    {
    //      if (x1[i] > svm) { svm = x1[i]; imin = i; }
    //    }
    //    svm *= EPS;	// pour le rang	

    for (i=0;i<x1.getRows();i++) if (x1[i] < x1[imin]) imin = i;

    for (i=0;i<x1.getRows();i++) x1[i] = ata[i][imin];

    x2 = - (r*x1) ; // X_2 = - (B^T B)^(-1) B^T A X_1
}



void CPosePerspective::poseInit(vpHomogeneousMatrix & cMo, CModel *cam)
{
	double s;
    int i;

    int k=0;
    int nl=npt*2;


    vpMatrix a(nl,3)  ;
    vpMatrix b(nl,6);
    CPoint P ;
    listP.front() ;
    i=0 ;
    while (!listP.outside())
    {
      P= listP.value() ;
      a[k][0]   = -P.get_oX();
      a[k][1]   = 0.0;
      a[k][2]   = P.get_oX()*P.get_x();

      a[k+1][0] = 0.0;
      a[k+1][1] = -P.get_oX();
      a[k+1][2] = P.get_oX()*P.get_y();

      b[k][0]   = -P.get_oY();
      b[k][1]   = 0.0;
      b[k][2]   = P.get_oY()*P.get_x();
      b[k][3]   =  -1.0;
      b[k][4]   =  0.0;
      b[k][5]   =  P.get_x();

      b[k+1][0] =  0.0;
      b[k+1][1] = -P.get_oY();
      b[k+1][2] =  P.get_oY()*P.get_y();
      b[k+1][3] =  0.0;
      b[k+1][4] = -1.0;
      b[k+1][5] =  P.get_y();

      k += 2;
      listP.next() ;
    }
    vpColVector X1(3) ;
    vpColVector X2(6) ;

    lagrange(a,b,X1,X2);

    if (X2[5] < 0.0)
    {		// car Zo > 0	
      for (i=0;i<3;i++) X1[i] = -X1[i];
      for (i=0;i<6;i++) X2[i] = -X2[i];
    }
    s = 0.0;
    for (i=0;i<3;i++) {s += (X1[i]*X2[i]);}
    for (i=0;i<3;i++)  {X2[i] -= (s*X1[i]);} // X1^T X2 = 0	

    s = 0.0;
    for (i=0;i<3;i++)  {s += (X2[i]*X2[i]);}

    if (s<1e-10)
    {
      vpERROR_TRACE( "in vpCalculPose::PosePlan(...) division par zero ") ;
      throw(vpException(vpException::divideByZeroError,
			"division by zero  ")) ;
    }

    s = 1.0/sqrt(s);
    for (i=0;i<3;i++)  {X2[i] *= s;}		// X2^T X2 = 1	


    calculTranslation (a, b, nl, 3, 3, X1, X2) ;

    // if (err != OK)
    {
      // std::cout << "in (vpCalculPose_plan.cc)CalculTranslation returns " ;
      // PrintError(err) ;
      //    return err ;
    }

    cMo[0][2] = (X1[1]*X2[2])-(X1[2]*X2[1]);
    cMo[1][2] = (X1[2]*X2[0])-(X1[0]*X2[2]);
    cMo[2][2] = (X1[0]*X2[1])-(X1[1]*X2[0]);

    for (i=0;i<3;i++)
    { // calcul de la matrice de passage	
      cMo[i][0] = X1[i];
      cMo[i][1] = X2[i];
      cMo[i][3] = X2[i+3];
    }
}
