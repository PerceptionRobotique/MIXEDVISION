/*!
  \file CCorner.h
  \brief Deftect a Corner
*/

#ifndef CCorner_hh
#define CCorner_hh

#include <math.h>
#include <fstream>

#include <visp/vpConfig.h>
#include <visp/vpList.h>

#include <visp/vpImage.h>
//#include <visp/vpDisplay.h>

#include <visp/vpTracker.h>
#include <visp/vpRect.h>

//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

/*!
  \class vpDot

  \ingroup TrackingImageBasic

  \brief This tracker is meant to track a dot (connex pixels with same
  gray level) on a vpImage.

  The underground algorithm is based on a binarisation of the image
  and a connex component segmentation to determine the dot
  characteristics (location, moments, size...).

  \sa vpDot2
*/
class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CCorner : public vpTracker
{
	double pos_ufloat, pos_vfloat;
	unsigned int pos_u, pos_v, winu, winv;
	bool graphics;

public:
  void init() ;
  CCorner() ;
  CCorner(const unsigned int u, const unsigned int v) ;
  CCorner(const double u, const double v) ;
  CCorner(const CCorner& c) ;
  virtual ~CCorner() {}

public:
  CCorner& operator =(const CCorner& f) ;
  bool operator ==(const CCorner& m);
  bool operator !=(const CCorner& m);

public:
  /*!

  Return the "u" (column) coordinate of the center of the dot within the image
  it comes from.
  */
  double get_u() const { return pos_ufloat ; }
  /*!

  Return the "v" (row) coordinate of the center of the dot within the image it
  comes from.
  */

  double get_v() const { return pos_vfloat ; }
  
  void set_u(double u) { pos_ufloat = u ; pos_u = (unsigned int)u ; }
  void set_v(double v) { pos_vfloat = v ; pos_v = (unsigned int)v ; }

  void setWin(unsigned int _winu, unsigned int _winv);
  void getWin(unsigned int &_winu, unsigned int &_winv) {_winu = winu; _winv = winv;}

  void detectCorner(vpImage<unsigned char> &I, double _u=-1, double _v=-1, unsigned int _winu=0, unsigned int _winv=0);

  int initTracking(vpImage<unsigned char>& I, unsigned int size = 0);
  void initTracking(vpImage<unsigned char>& I, unsigned int u, unsigned int v,
                     unsigned int size = 0);

	/*inline*/ vpRect getBBox()
	{
		vpRect bbox;

		bbox.setRect(pos_ufloat - winu/2, pos_vfloat-winv/2, pos_ufloat+winu/2, pos_vfloat+winv/2);

		return (bbox);
	};


public:

  /*!
    Print the coordinates of the point center of gravity
    in the stream.
  */
  friend std::ostream& operator<< (std::ostream& os, CCorner& p) {
    return (os <<"("<<p.pos_ufloat<<","<<p.pos_vfloat<<")" ) ;
  } ;
  void print(std::ostream& os) { os << *this << std::endl ; }

public:
  
  void setGraphics(const bool activate) { graphics = activate ; }
  //void display(vpImage<unsigned char> &I, vpColor col = vpColor::green);

} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


