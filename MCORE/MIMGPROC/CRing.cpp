/*
  \file vpDot.cpp
  \brief Track a white dot
*/

#include <MIXEDVISION/CRing.h>

#include <visp/vpDisplay.h>
#include <visp/vpColor.h>

// exception handling
#include <visp/vpTrackingException.h>

//#include <cv.h>
#include <opencv2/opencv.hpp>

#include <gsl/gsl_eigen.h>

//#define AFFDETAILSTXT

//Attention : à remettre
//#include "../lapack/clapack.h"

CRing::CRing()
{
}

CRing::CRing(const CRing& c) : vpDot2()
{
  *this = c ;
}


CRing::~CRing()
{
}

CRing& CRing::operator =(const CRing& twinDot)
{
	*((vpDot2 *)this) = (vpDot2)twinDot;
	/*
	//de vpDot2
	setCog(twinDot.getCog()); //cog = twinDot.cog;

	setWidth(twinDot.getWidth()); //width    = twinDot.width;
	setHeight(twinDot.getHeight()); //height   = twinDot.height;
	setArea(twinDot.getArea()); //surface  = twinDot.surface;
	setMeanGrayLevel(twinDot.getMeanGrayLevel()); //mean_gray_level = twinDot.mean_gray_level;
	setMinGrayLevel(twinDot.getMinGrayLevel()); //gray_level_min = twinDot.gray_level_min;
	setMaxGrayLevel(twinDot.getMaxGrayWidth()); //gray_level_max = twinDot.gray_level_max;
	grayLevelPrecision = twinDot.grayLevelPrecision;
	gamma = twinDot.gamma; ;
	sizePrecision = twinDot.sizePrecision;
	ellipsoidShapePrecision = twinDot.ellipsoidShapePrecision ;
	maxSizeSearchDistancePrecision = twinDot.maxSizeSearchDistancePrecision;
	area = twinDot.area;

	m00 = twinDot.m00;
	m01 = twinDot.m01;
	m11 = twinDot.m11;
	m10 = twinDot.m10;
	m02 = twinDot.m02;
	m20 = twinDot.m20;

	bbox_u_min = twinDot.bbox_u_min;
	bbox_u_max = twinDot.bbox_u_max;
	bbox_v_min = twinDot.bbox_v_min;
	bbox_v_max = twinDot.bbox_v_max;

	firstBorder_u = twinDot.firstBorder_u;
	firstBorder_v = twinDot.firstBorder_v;

	compute_moment = twinDot.compute_moment;
	graphics = twinDot.graphics;

	direction_list = twinDot.direction_list;
	ip_edges_list =  twinDot.ip_edges_list;
	*/

	//nouveau pour CRing
	ecog_ufloat = twinDot.ecog_ufloat;
	ecog_vfloat = twinDot.ecog_vfloat ;

	prc_ufloat = twinDot.prc_ufloat;
	prc_vfloat = twinDot.prc_vfloat;

	ewidth = twinDot.ewidth;
	eheight = twinDot.eheight;
	esurface = twinDot.esurface;
	egray_level_min = twinDot.egray_level_min;
	egray_level_max = twinDot.egray_level_max;
	emean_gray_level = twinDot.emean_gray_level;
	egamma = twinDot.egamma;

	edirection_list = twinDot.edirection_list;
	eu_list = twinDot.eu_list;
	ev_list = twinDot.ev_list;

	ebbox_u_min = twinDot.ebbox_u_min;
	ebbox_u_max = twinDot.ebbox_u_max;
	ebbox_v_min = twinDot.ebbox_v_min;
	ebbox_v_max = twinDot.ebbox_v_max;

	efirstBorder_u = twinDot.efirstBorder_u;
	efirstBorder_v = twinDot.efirstBorder_v;

	em00 = twinDot.em00;
	em10 = twinDot.em10;
	em01 = twinDot.em01;
	em11 = twinDot.em11;
	em20 = twinDot.em20;
	em02 = twinDot.em02;

	return *this; 
};

void CRing::initTracking(vpImage<unsigned char>& I,unsigned int size)
{
	bool trackOK = true;
	// Track inner dot
	vpDot2::initTracking(I,size);
#ifdef AFFDETAILSTXT
	std::cout << "vpDot2 internal cog : " << getCog().get_u() << " " << getCog().get_v() << std::endl;
#endif
	// Track outer dot
	track(I);
	if(!trackOK)
	{
#ifdef AFFDETAILSTXT
	    vpTRACE( "Ring not found !");
#endif
	    return;
	}
#ifdef AFFDETAILSTXT
	std::cout << "internal cog : " << getCog().get_u() << " " << getCog().get_v() << std::endl;
	std::cout << "external cog : " << ecog_ufloat << " " << ecog_vfloat << std::endl;
#endif

	// Retrieve real projected center point
	vpMatrix A1, A2; // Conic matrices of the two concentric circles
#ifdef AFFDETAILSTXT
	std::cout << "points contour cercle interne" << std::endl;
#endif
    std::list<vpImagePoint> my_ip_edges_list;
    getEdges(my_ip_edges_list);
	computeConicParameters(my_ip_edges_list, A1); //conique du cercle interne
#ifdef AFFDETAILSTXT
	std::cout << "points contour cercle externe" << std::endl;
#endif
	computeConicParameters(eu_list, ev_list, A2); //conique du cercle externe

/*	std::cout << "A1 : " << A1;
	std::cout << "A2 : " << A2;*/

	vpColVector ICP1, ICP2, phi;
	retrieveICPs(A1, A2, ICP1, ICP2, phi);
//	vpDisplay::displayCross(I, 3608, 1317, 10, vpColor::green);

/*	vpDisplay::displayCross(I, (unsigned int)ICP1[1],(unsigned int)ICP1[0], 10, vpColor::green);
	vpDisplay::displayCross(I, (unsigned int)ICP2[1],(unsigned int)ICP2[0], 10, vpColor::blue);
	vpDisplay::displayLine_uv(I, 451, 156, -52, 152, vpColor::blue);*/
	/*vpDisplay::displayCross(I, (unsigned int)phi[1],(unsigned int)phi[0], 10, vpColor::green);
	vpDisplay::flush(I);
	vpDisplay::getClick(I);*/

	prc_ufloat = phi[0];
	prc_vfloat = phi[1];

//	std::cout << "real centre point : " << prc_ufloat << " " << prc_vfloat << std::endl;
}

void CRing::initTracking(vpImage<unsigned char>& I, unsigned int u, unsigned int v,
             unsigned int size)
{
	bool trackOK = true;
	vpDot2::initTracking(I,vpImagePoint(v,u),size);
	trackOK = track(I);
	if(trackOK)
	{
		// Retrieve real projected center point
		vpMatrix A1, A2; // Conic matrices of the two concentric circles
        std::list<vpImagePoint> my_ip_edges_list;
        getEdges(my_ip_edges_list);
		computeConicParameters(my_ip_edges_list, A1); //conique du cercle interne
		computeConicParameters(eu_list, ev_list, A2); //conique du cercle externe

		vpColVector ICP1, ICP2, phi;
		retrieveICPs(A1, A2, ICP1, ICP2, phi);

		prc_ufloat = phi[0];
		prc_vfloat = phi[1];
	}
}

void CRing::initTracking(vpImage<unsigned char>& I, unsigned int u, unsigned int v,
	    unsigned int gray_level_min, unsigned int gray_level_max,
		unsigned int size)
{
	bool trackOK = true;
	vpDot2::initTracking(I,vpImagePoint(v,u),gray_level_min, gray_level_max, size);

	trackOK = track(I);

	if(trackOK)
	{
		// Retrieve real projected center point
		vpMatrix A1, A2; // Conic matrices of the two concentric circles
        std::list<vpImagePoint> my_ip_edges_list;
        getEdges(my_ip_edges_list);
		computeConicParameters(my_ip_edges_list, A1); //conique du cercle interne
		computeConicParameters(eu_list, ev_list, A2); //conique du cercle externe

		vpColVector ICP1, ICP2, phi;
		retrieveICPs(A1, A2, ICP1, ICP2, phi);

		prc_ufloat = phi[0];
		prc_vfloat = phi[1];
	}
}

bool CRing::track(vpImage<unsigned char> &I)
{
  bool ret=true;
#ifdef AFFDETAILSTXT
  std::cout << "track ring ! " << std::endl;
#endif

//  vpDot2::track(I);

  // A ce point, le tracking est fait pour le dot central
  unsigned u = this->getFirstBorder_u()+2; //Position à l'extérieur du dot blanc //+5
  unsigned v = this->getFirstBorder_v();

  //setArea(I);//, u-ewidth/2, v-eheight/2, ewidth, eheight);

  ecog_ufloat = (double) u ; 
  ecog_vfloat = (double) v ;

  double Ip = pow((double)I[v][u]/255.0,1/getGamma());

  if(Ip - (1 - getGrayLevelPrecision())<0){
    egray_level_min = 0 ;
  }
  else{
    egray_level_min = (unsigned int) (255*pow(Ip - (1 - getGrayLevelPrecision()),getGamma()));
    if (egray_level_min > 255)
      egray_level_min = 255;
  }
  egray_level_max = (unsigned int) (255*pow(Ip + (1 - getGrayLevelPrecision()),getGamma()));
  if (egray_level_max > 255)
    egray_level_max = 255;

//std::cout << "gray levels : " << (double)I[v][u] << " " << egray_level_min << " " << egray_level_max << " " << grayLevelPrecision << std::endl;

	double 	Sgray_level_min=getGrayLevelMin(),
		Sgray_level_max=getGrayLevelMax(),
		Swidth = getWidth(), Sheight = getHeight();


    setGrayLevelMin(egray_level_min);
    setGrayLevelMax(egray_level_max);
	setWidth(ewidth); setHeight(eheight);

  //changer gray_level_min, gray_level_max
  ret = computeExternalParameters(I);


	egray_level_min=getGrayLevelMin();
	egray_level_max=getGrayLevelMax();
	ewidth = getWidth(); eheight = getHeight();

    setGrayLevelMin(Sgray_level_min);
    setGrayLevelMax(Sgray_level_max);
	setWidth(Swidth); setHeight(Sheight);
  return ret;
}

bool CRing::track(vpImage<unsigned char> &I, double &_u, double &_v)
{
  bool ret=true;
#ifdef AFFDETAILSTXT
  std::cout << "track ring 2 ! " << std::endl;
#endif
//  vpDot2::track(I,_u,_v);

  // A ce point, le tracking est fait pour le dot central
  unsigned u = this->getFirstBorder_u()+2; //Position à l'extérieur du dot blanc
  unsigned v = this->getFirstBorder_v();

  ecog_ufloat = (double) u ; 
  ecog_vfloat = (double) v ;

  double Ip = pow((double)I[v][u]/255.0,1/getGamma());

  if(Ip - (1 - getGrayLevelPrecision())<0){
    egray_level_min = 0 ;
  }
  else{
    egray_level_min = (unsigned int) (255*pow(Ip - (1 - getGrayLevelPrecision()),getGamma()));
    if (egray_level_min > 255)
      egray_level_min = 255;
  }
  egray_level_max = (unsigned int) (255*pow(Ip + (1 - getGrayLevelPrecision()),getGamma()));
  if (egray_level_max > 255)
    egray_level_max = 255;

	double 	Sgray_level_min=getGrayLevelMin(),
		Sgray_level_max=getGrayLevelMax(),
		Swidth = getWidth(), Sheight =getHeight();

    setGrayLevelMin(egray_level_min);
	setGrayLevelMax(egray_level_max);
	setWidth(ewidth); setHeight(eheight);

  ret=computeExternalParameters(I);

	egray_level_min=getGrayLevelMin();
	egray_level_max=getGrayLevelMax();
	ewidth = getWidth(); eheight = getHeight();

	setGrayLevelMin(Sgray_level_min);
	setGrayLevelMax(Sgray_level_max);
	setWidth(Swidth); setHeight(Sheight);

  return ret;
}


/*!

  Compute all the parameters of the dot (center, width, height, surface,
  inertia moments...).

  This is done the followin way:

  - First, we check the point (_u, _v) passed in has the right level in the
    image

  - Then we cross the tracked entity from left to right until we reach it's
    border.

  - We follow this border until we come back to the first point or we get to
    border of the image. Each time we update variables used to compute the
    dot parameters

  \param I : The image we are working with.

  \param _u : A pixel coordinate inside the dot.

  \param _v : A pixel coordinate inside the dot.

  \return false : If a dot can't be found around pixel coordinates given as
  parameter

  \return true : If a dot was found.

  \sa getFirstBorder_u(), getFirstBorder_v()

*/
bool CRing::computeExternalParameters(const vpImage<unsigned char> &I,
			       const double &_u,
			       const double &_v)
{
#ifdef AFFDETAILSTXT
  std::cout << "track ring ! " << std::endl;
#endif

  edirection_list.kill();
  eu_list.kill();
  ev_list.kill();

  double est_u = _u; // estimated
  double est_v = _v;

  // if u has default value, set it to the actual center value
  if( est_u == -1.0 )
  {
    est_u = getFirstBorder_u()+1;
  }

  // if v has default value, set it to the actual center value
  if( est_v == -1.0 )
  {
    est_v = getFirstBorder_v();
  }

    vpTRACE( "source pixel (%d, %d) coordinates",
		(int) est_u, (int) est_v) ;

  // if the estimated position of the dot is out of the image, not need to continue,
  // return an error tracking
  if( !isInArea( (int) est_u, (int) est_v ) )
  {
    vpTRACE( "Initial pixel coordinates (%d, %d) for dot tracking are not in the area",
		(int) est_u, (int) est_v) ;
    return false;
  }

  ebbox_u_min = I.getWidth();
  ebbox_u_max = 0;
  ebbox_v_min = I.getHeight();
  ebbox_v_max = 0;

  if( !isInArea( (int) est_u, (int) est_v ) )
  {
#ifdef AFFDETAILSTXT
	std::cout << "oups " << std::endl;
#endif
  }

  if( I[(int) est_v][(int) est_u] >= getGrayLevelMin() &&  I[(int) est_v][(int) est_u] <= getGrayLevelMax())
  {
#ifdef AFFDETAILSTXT
	std::cout << "youpi " << std::endl;
#endif
  }
  // if the first point doesn't have the right level then there's no point to
  // continue.
  if( !hasGoodLevel( I, (unsigned int) est_u, (unsigned int) est_v ) )
  {
    vpTRACE( "Can't find a dot from pixel (%d, %d) coordinates",
		(int) est_u, (int) est_v) ;
    return false;
  }
{
  int border_u = est_u;
  int border_v = est_v;
  double epsilon =0.001;
  while( hasGoodLevel( I, border_u+1, border_v ) &&
    border_u < area.getRight()/*I.getWidth()*/ )  {
    // if the width of this dot was initialised and we already crossed the dot
    // on more than the max possible width, no need to continue, return an
    // error tracking
    if( getWidth() > 0 && ( border_u - est_u ) > getWidth()/(getMaxSizeSearchDistancePrecision()+epsilon) ) {
#ifdef AFFDETAILSTXT
      vpTRACE("The found dot (%d, %d, %d) has a greater width than the required one", (int)est_u, (int)est_v, border_u);
#endif
      break;
    }
    border_u++;
  }
#ifdef AFFDETAILSTXT
  std::cout << "border_u : " << border_u << std::endl;
#endif
}
  // find the border
  if(!findFirstBorder(I, (unsigned int) est_u, (unsigned int) est_v,
		      this->efirstBorder_u, this->efirstBorder_v)) {

    vpTRACE( "Can't find first border (%d, %d) coordinates",
		(int) est_u, (int) est_v) ;
    return false;
  }

  unsigned int dir = 6;

  // Determine the first element of the Freeman chain
  computeFreemanChainElement(I, this->efirstBorder_u, this->efirstBorder_v, dir);
  unsigned int firstDir = dir;

  // if we are now out of the image, return an error tracking
  if( !isInArea( this->efirstBorder_u, this->efirstBorder_v ) )
  {
    vpTRACE( "Border pixel coordinates (%d, %d) of the dot are not in the area",
		  this->efirstBorder_u, this->efirstBorder_v);
    return false;
  }

  // store the new direction and dot border coordinates.
  edirection_list.addRight( dir );
  eu_list.addRight( this->efirstBorder_u );
  ev_list.addRight( this->efirstBorder_v );

  int border_u = this->efirstBorder_u;
  int border_v = this->efirstBorder_v;

//   vpTRACE("-----------------------------------------");
//   vpTRACE("first border_u: %d border_v: %d dir: %d",
// 	this->firstBorder_u, this->firstBorder_v,firstDir);
  int du, dv;
  float dS, dMu, dMv, dMuv, dMu2, dMv2;
  em00 = 0.0;
  em10 = 0.0;
  em01 = 0.0;
  em11 = 0.0;
  em20 = 0.0;
  em02 = 0.0;
  // while we didn't come back to the first point, follow the border
  do {
    // if it was asked, show the border
   /* if (graphics) {
      vpDisplay::displayPoint(I, vpImagePoint(border_v, border_u), vpColor::red) ;
      //vpDisplay::flush(I);
    }
    */
    // Determine the increments for the parameters
    computeFreemanParameters(border_u, border_v, dir, du, dv,
			     dS, // surface
			     dMu, dMv, // first order moments
			     dMuv, dMu2, dMv2); // second order moment

    // Update the parameters
    border_u += du; // Next position on the border
    border_v += dv;
    em00 += dS; // enclosed area
    em10 += dMu; // First order moment along v axis
    em01 += dMv; // First order moment along u axis
    if (compute_moment) {
      em11 += dMuv; // Second order moment
      em20 += dMu2; // Second order moment along v axis
      em02 += dMv2; // Second order moment along u axis
    }
    // if we are now out of the image, return an error tracking
    if( !isInArea( border_u, border_v ) )  {

      vpTRACE( "Dot (%d, %d) is not in the area", border_u, border_v);
      // Can Occur on a single pixel dot located on the top border
      return false;
    }

    // store the new direction and dot border coordinates.

    edirection_list.addRight( dir );
    eu_list.addRight( border_u );
    ev_list.addRight( border_v );

    // vpDisplay::getClick(I);

    // update the extreme point of the dot.
    if( border_v < bbox_v_min ) bbox_v_min = border_v;
    if( border_v > bbox_v_max ) bbox_v_max = border_v;
    if( border_u < bbox_u_min ) bbox_u_min = border_u;
    if( border_u > bbox_u_max ) bbox_u_max = border_u;

    // move around the tracked entity by following the border.
    if (computeFreemanChainElement(I, border_u, border_v, dir) == false) {
      vpTRACE( "Can't compute Freeman chain for dot (%d, %d)",
		    border_u, border_v);
      return false;
    }

//     vpTRACE("border_u: %d border_v: %d dir: %d", border_u, border_v, dir);

  }
  while( (this->efirstBorder_u != border_u
	  || this->efirstBorder_v != border_v
	  || firstDir != dir) &&
	 isInArea( border_u, border_v ) );


/*  vpDisplay::flush(I);
  vpDisplay::getClick(I);*/

  // if the surface is one or zero , the center of gravity wasn't properly
  // detected. Return an error tracking.
  if( em00 == 0 || em00 == 1 )
  {
    vpTRACE( "The center of gravity of the dot wasn't properly detected");
    return false;
  }
  else // compute the center
  {
    // this magic formula gives the coordinates of the center of gravity
    double tmpCenter_u = em10 / em00;
    double tmpCenter_v = em01 / em00;


    // check the center is in the image... never know...
//     if( !hasGoodLevel( I, (unsigned int)tmpCenter_u,
// 		       (unsigned int)tmpCenter_v ) )
//     {
//       vpTRACE( "The center of gravity of the dot (%g, %g) has not a good in level", tmpCenter_u, tmpCenter_v);
//       return false;
//     }

    ecog_ufloat = tmpCenter_u;
    ecog_vfloat = tmpCenter_v;
  }

  ewidth   = ebbox_u_max - ebbox_u_min + 1;
  eheight  = ebbox_v_max - ebbox_v_min + 1;
  esurface = em00;
/*
  computeMeanGrayLevel(I);
  //sauvegarder les infos du dot interne
  //les remplacer par celles du dot externe
  //appeler de nouveau computeMeanGrayLevel(I) qui s'exécutera cette fois-ci pour le dot globale
  //Procéder par différence pondérée pour trouver le ndg moyen de l'anneau
*/
  return true;
}

/*void
CRing::display(vpImage<unsigned char>& I,vpColor colint,vpColor colext,vpColor colcross)
{
    vpList<vpImagePoint> my_ip_edges_list;
    getEdges(my_ip_edges_list);
    
 my_ip_edges_list.front();

  while(!(my_ip_edges_list.outside())){
	  vpImagePoint ip = my_ip_edges_list.value();
    vpDisplay::displayPoint(I,ip,colint);
    my_ip_edges_list.next();
  }

  eu_list.front();
  ev_list.front();
  while(!(eu_list.outside())){
    vpDisplay::displayPoint(I,ev_list.value(),eu_list.value(),colext);
    eu_list.next();
    ev_list.next();
  }

  vpDisplay::displayCross(I,(unsigned int)get_v(),(unsigned int)get_u(),
                          10,colcross);

}*/

/*
Fonction calculant l'équation de conique à partir des points passés en paramètre par SVD
! à sûrement placer dans une autre classe avec d'autres outils "coniques"
*/

void 
CRing::computeConicParameters(std::list<vpImagePoint> ip_list, vpMatrix & A)
{
		//std::list<vpImagePoint> ip_list
    std::list<vpImagePoint> my_ip_edges_list;
    std::list<vpImagePoint>::iterator it_my_ip_edges_list;
    getEdges(my_ip_edges_list);
	vpMatrix D(my_ip_edges_list.size(), 6), V(6,(my_ip_edges_list.size()<6)?my_ip_edges_list.size():6);
	vpColVector S(6), Z(6);
	unsigned int i;
	double u, v;

	// Construction d'un système DZ=0

	it_my_ip_edges_list = my_ip_edges_list.begin();
	i = 0;
	while(it_my_ip_edges_list != my_ip_edges_list.end())
	{
		vpImagePoint ip = *it_my_ip_edges_list;
		u = ip.get_u();
		v = ip.get_v();

//		std::cout << u << "   " << v << ";";

		D[i][0] = u*u;
		D[i][1] = u*v;
		D[i][2] = v*v;
		D[i][3] = u;
		D[i][4] = v;
		D[i][5] = 1.0;

		i++;
		it_my_ip_edges_list++;

	}
//	std::cout << std::endl;
	//Maintenant on traite D pour obtenir Z
#ifdef VISP_HAVE_GSL
	D.svd(S, V);
#else
	//on utilise la svd de OpenCV
	cv::Mat DMat=cv::Mat(D.getRows(), D.getCols(), CV_64F, D.data),
	SMat=cv::Mat(S.getRows(), 1, CV_64F, S.data), 
	VMat=cv::Mat(V.getRows(), V.getCols(), CV_64F, V.data),
	UMat;
	
	cv::SVD::compute(DMat, SMat, UMat, VMat, cv::SVD::MODIFY_A);
#endif

/*	//find max
	double minVal=S[0];
	unsigned int indVal = 0;
	for(i=1;i<6;i++)	
		if(S[i] < minVal)
		{
			indVal = i;
			minVal = S[i];
		}
	std::cout << "ind Val : " << indVal << std::endl;*/

	for(i=0;i<6;i++)
		Z[i] = V[i][5];
	
	//Création de la matrice conique à partir du vecteur des paramètres de cette même conique
	A.resize(3,3);
	A[0][0] = Z[0]; 	A[0][1] = Z[1]/2.0; 	A[0][2] = Z[3]/2.0;
	A[1][0] = Z[1]/2.0; 	A[1][1] = Z[2]; 	A[1][2] = Z[4]/2.0;
	A[2][0] = Z[3]/2.0; 	A[2][1] = Z[4]/2.0; 	A[2][2] = Z[5];
	//normalisation
	A /= A[2][2];
}


void 
CRing::computeConicParameters(vpList<unsigned int> u_list, vpList<unsigned int> v_list, vpMatrix & A)
{
    std::list<vpImagePoint> my_ip_edges_list;
    getEdges(my_ip_edges_list);
	vpMatrix D(u_list.nbElements(), 6), V(6,(my_ip_edges_list.size()<6)?my_ip_edges_list.size():6);
	vpColVector S(6), Z(6);
	unsigned int i;
	double u, v;
	
	// Construction d'un système DZ=0
	
	u_list.front();
	v_list.front();
	i = 0;
	while(!(u_list.outside()))
	{
		u = u_list.value();
		v = v_list.value();
		
		D[i][0] = u*u;
		D[i][1] = u*v;
		D[i][2] = v*v;
		D[i][3] = u;
		D[i][4] = v;
		D[i][5] = 1.0;
		
		i++;
		u_list.next();
		v_list.next();
	}
	//Maintenant on traite D pour obtenir Z
#ifdef VISP_HAVE_GSL
	D.svd(S, V);
#else
	//on utilise la svd de OpenCV
	cv::Mat DMat=cv::Mat(D.getRows(), D.getCols(), CV_64F, D.data),
	SMat=cv::Mat(S.getRows(), 1, CV_64F, S.data), 
	VMat=cv::Mat(V.getRows(), V.getCols(), CV_64F, V.data),
	UMat;
	
	cv::SVD::compute(DMat, SMat, UMat, VMat, cv::SVD::MODIFY_A);
#endif
	
	/*	//find max
	 double minVal=S[0];
	 unsigned int indVal = 0;
	 for(i=1;i<6;i++)	
	 if(S[i] < minVal)
	 {
	 indVal = i;
	 minVal = S[i];
	 }
	 std::cout << "ind Val : " << indVal << std::endl;*/
	
	for(i=0;i<6;i++)
		Z[i] = V[i][5];
	
	//Création de la matrice conique à partir du vecteur des paramètres de cette même conique
	A.resize(3,3);
	A[0][0] = Z[0]; 	A[0][1] = Z[1]/2.0; 	A[0][2] = Z[3]/2.0;
	A[1][0] = Z[1]/2.0; 	A[1][1] = Z[2]; 	A[1][2] = Z[4]/2.0;
	A[2][0] = Z[3]/2.0; 	A[2][1] = Z[4]/2.0; 	A[2][2] = Z[5];
	//normalisation
	A /= A[2][2];
}


void 
CRing::retrieveICPs(vpMatrix &A1, vpMatrix &A2, vpColVector &ICP1, vpColVector &ICP2, vpColVector &phi)
{
	// Calcul des valeurs propres généralisées sur A1 et A2
	vpMatrix Ep = A1*A2.inverseByLU(), Delta2, V, U, S2x2, evector(Ep.getCols(), Ep.getRows());
	vpColVector E(Ep.getRows()), Ei(Ep.getRows()), Ec(Ep.getRows()), S, I(2), J(2);
	
	eigenValuesNonSym(Ep, E, evector);
#ifdef AFFDETAILSTXT
	std::cout << E.t() << std::endl << std::endl << evector << std::endl << std::endl;
#endif
	
	char jobvl = 'V', jobvr = 'N';
	int n = Ep.getRows();
	double *a = Ep.data;
	int lda = Ep.getRows();
	double *wr = E.data, *vr = evector.data, *wi = Ei.data;
	int ldvl = 1, ldvr = Ep.getCols();
	double work[1024];
	int lwork = 1024;
	int info;
	E = 0;
	evector = 0;
//Attention : à remettre
//	dgeev_(&jobvl, &jobvr, &n, a, &lda, wr, wi, vr, &ldvr, NULL, &ldvl, work, &lwork, &info);
#ifdef AFFDETAILSTXT
	std::cout << E.t() << std::endl << std::endl << evector << std::endl << std::endl;
#endif
	//exit(10);
	
	// Recherche de l'index de la valeur propre la plus grande/la plus éloignée du médian
	double median_E = vpColVector::median(E);
	Ec[0] = E[0] - median_E;

	double valMax = Ec[0];
	int indMax = 0;
	for(int i = 1 ; i < Ec.getRows() ; i++)
	{
		Ec[i] = E[i] - median_E;
		if(fabs(Ec[i]) > valMax)
		{
			valMax = fabs(Ec[i]);
			indMax = i;
		}
	}

	// calcul du Delta2 
	Delta2 = E[indMax]*A1.inverseByLU() - A2.inverseByLU();

	vpColVector LInf(3);
	LInf[2]=1.0;
	phi=Delta2*LInf;
	phi /= phi[2];
	
/*
	//SVD puis ICPs
	U = Delta2;
	U.svd(S, V);

	S.resize(2, false);
	S[0] = sqrt(S[0]); S[1] = sqrt(S[1]);
	vpMatrix::createDiagonalMatrix(S, S2x2);
	U.resize(3, 2, false);

	//On utilise S2x2 directement car sqrt est déjà fait
	I[0] = I[1] = 1;
	ICP1 = U*S2x2*I;
	ICP1 /= ICP1[2];

	J[0] = 1; J[1] = -1;
	ICP2 = U*S2x2*J;
	ICP2 /= ICP2[2];*/
}

void CRing::eigenValuesNonSym(vpMatrix &A, vpColVector &evalue, vpMatrix &evector)
{
    double rowNum = A.getRows(), colNum = A.getCols();    if (rowNum != colNum) {
        vpERROR_TRACE("Eigen values computation: ERR Matrix not square") ;
        throw(vpException(vpException::dimensionError,
                                "\n\t\tEigen values computation: ERR Matrix not square")) ;
    }
//#ifdef MIXEDVISION_HAVE_GSL /* be careful of the copy below */
    {        // Resize the output matrices
        evalue.resize(rowNum);
        evector.resize(rowNum, colNum);        
        gsl_vector_complex *eval = gsl_vector_complex_alloc (rowNum);
        gsl_matrix_complex *evec = gsl_matrix_complex_alloc (rowNum, colNum);        
        gsl_eigen_nonsymmv_workspace * w = gsl_eigen_nonsymmv_alloc (rowNum);
        gsl_matrix *m = gsl_matrix_alloc(rowNum, colNum);        
        int Atda = m->tda ;
        for (int i=0 ; i < rowNum ; i++)
        {
            int k = i*Atda ;
            for (int j=0 ; j < colNum ; j++)
                m->data[k+j] = A[i][j] ;
        }
        gsl_eigen_nonsymmv (m, eval, evec, w);        
        //  gsl_eigen_nonsymmv_sort (eval, evec, GSL_EIGEN_SORT_VAL_ASC);        
        for (int i=0; i < rowNum; i++) 
        {
            gsl_complex eval_i = gsl_vector_complex_get (eval, i);
            evalue[i] = GSL_REAL(eval_i);
        }
        Atda = evec->tda ;
        for (int i=0; i < rowNum; i++) 
        {
            int k = i*Atda ;
            gsl_vector_complex_view evec_i = gsl_matrix_complex_column (evec, i);
            for (int j=0; j < rowNum; j++) 
            {
        				gsl_complex z = gsl_vector_complex_get(&evec_i.vector, j);
                evector[i][j] = GSL_REAL(z);
            }
        }     
        //    {
        //     int i;        //     for (i = 0; i < rowNum; i++)
        //      {
        //       double eval_i
        //         = gsl_vector_get (eval, i);
        //       gsl_vector_view evec_i
        //         = gsl_matrix_column (evec, i);        //       printf (“eigenvalue = %g\n”, eval_i);
        //       printf (“eigenvector = \n”);
        //       gsl_vector_fprintf (stdout,
        //                 &evec_i.vector, “%g”);
        //      }
        //    }        gsl_eigen_nonsymmv_free (w);
        gsl_vector_complex_free (eval);
        gsl_matrix_free (m);
        gsl_matrix_complex_free (evec);
    }
/*#else
    {
        vpERROR_TRACE("Not implemented since the GSL library is not detected.") ;
        throw(vpException(vpException::notImplementedError,
                                "\n\t\tEigen values Computation: Not implemented "
                                "since the GSL library is not detected")) ;
    }
#endif*/
}


double CRing::get_u() const
{
  return prc_ufloat;
}


double CRing::get_v() const
{
  return prc_vfloat;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


