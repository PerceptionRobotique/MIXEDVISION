/*!
  \file CRing.h
  \brief Deftect a Corner
*/

#ifndef CRing_hh
#define CRing_hh

#include <math.h>
#include <fstream>
#include <list>

#include <visp/vpConfig.h>
#include <visp/vpList.h>

#include <visp/vpImage.h>
//#include <visp/vpDisplay.h>

#include <visp/vpDot2.h>
#include <visp/vpRect.h>

#include <visp/vpImagePoint.h>


/*!
  \class vpDot2

  \ingroup TrackingImageBasic

  \brief This tracker is meant to track a dot (connex pixels with same
  gray level) on a vpImage.

  Cette classe a pour but de détecter un anneau. Elle hérite de vpDot2 
  car la partie centrale de l'anneau est détectable comme telle. L'ajout 
  par rapport à vpDot2 intervient après le processus de tracking de 
  la partie centrale de l'anneau, afin d'en détecter la partie extérieure.

  \sa vpDot2
*/
class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CRing : public vpDot2
{
protected:
  //! coordinates (float) of the point center of gravity of external circle of ring
  //coordinates of internal circle cog are in cog_ufloat and cog_vfloat
  //e for external
  double ecog_ufloat, ecog_vfloat ;

  //! Coordinates (float) of the projected 3D ring center point (differs from cog theoretically)
  double prc_ufloat, prc_vfloat;

  double ewidth;
  double eheight;
  double esurface;
  unsigned int egray_level_min;  // minumum gray level for the dot.
				// pixel with lower level don't belong
				// to this dot.

  unsigned int egray_level_max;  // maximum gray level for the dot.
				// pixel with higher level don't belong
				// to this dot.
  double emean_gray_level; // Mean gray level of the dot
  double egamma ;

  // other
  vpList<int> edirection_list;
  vpList<unsigned int> eu_list;
  vpList<unsigned int> ev_list;

  // Bounding box
  int ebbox_u_min, ebbox_u_max, ebbox_v_min, ebbox_v_max;

  // The first point coodinate on the dot border
  unsigned int efirstBorder_u;
  unsigned int efirstBorder_v;

  double em00; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{00} \f$ is a zero order moment obtained
		with \f$i = j = 0 \f$. This moment corresponds to the dot
		surface.

		\sa setComputeMoments()
	      */
  double em10; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{10} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 0 \f$. \f$ m_{10} \f$
		corresponds to the inertia first order moment along the v axis.

		\sa setComputeMoments()
	      */
  double em01; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{01} \f$ is a first order moment
		obtained with \f$i = 0 \f$ and \f$ j = 1 \f$. \f$ m_{01} \f$
		corresponds to the inertia first order moment along the u axis.

		\sa setComputeMoments()
	      */
  double em11; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{11} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 1 \f$.

		\sa setComputeMoments()
	      */
  double em20; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{20} \f$ is a second order moment
		obtained with \f$i = 2 \f$ and \f$ j = 0 \f$. \f$ m_{20} \f$
		corresponds to the inertia second order moment along the v
		axis.

		\sa setComputeMoments()
	      */
  double em02; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{02} \f$ is a second order moment
		obtained with \f$i = 0 \f$ and \f$ j = 2 \f$. \f$ m_{02} \f$
		corresponds to the inertia second order moment along the u
		axis.

		\sa setComputeMoments()
	      */


public:
  CRing();
  CRing(const unsigned int u, const unsigned int v) : vpDot2(vpImagePoint(v,u)) {}
  CRing(const double u, const double v) : vpDot2(vpImagePoint(v,u)) {}
  CRing(const CRing& c) ;
  virtual ~CRing();

  double get_u() const;
  double get_v() const;

  bool track(vpImage<unsigned char> &I);
  bool track(vpImage<unsigned char> &I, double &u, double &v);

  bool computeExternalParameters(const vpImage<unsigned char> &I,
			 const double &u = -1.0,
			 const double &v = -1.0);

 // void display(vpImage<unsigned char>& I ,vpColor c = vpColor::red, vpColor colext = vpColor::yellow, vpColor colcross = vpColor::green);

  void initTracking(vpImage<unsigned char>& I, unsigned int size = 0);
  void initTracking(vpImage<unsigned char>& I, unsigned int u, unsigned int v,
                     unsigned int size = 0);
  void initTracking(vpImage<unsigned char>& I, unsigned int u, unsigned int v,
		    unsigned int gray_level_min, unsigned int gray_level_max,
        unsigned int size = 0 );

  void computeConicParameters(vpList<unsigned int> u_list, vpList<unsigned int> v_list, vpMatrix & A);
  void computeConicParameters(std::list<vpImagePoint> ip_list, vpMatrix & A);
  void retrieveICPs(vpMatrix &A1, vpMatrix &A2, vpColVector &ICP1, vpColVector &ICP2, vpColVector &phi);

  CRing& operator =(const CRing& f);
  
  static void eigenValuesNonSym(vpMatrix &A, vpColVector &evalue, vpMatrix &evector);
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


