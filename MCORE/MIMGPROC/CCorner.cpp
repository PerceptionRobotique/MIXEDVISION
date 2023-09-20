/*
  \file vpDot.cpp
  \brief Track a white dot
*/

#include <MIXEDVISION/CCorner.h>

#include <visp/vpDisplay.h>
#include <visp/vpColor.h>

// exception handling
#include <visp/vpTrackingException.h>

/*!
  Initialize the tracker with default parameters.
  - connexity is set to 4 (see setConnexity())
  - dot maximal size is set to 25% of the image size (see setMaxDotSize())
*/
void CCorner::init()
{
  pos_u = 0 ;
  pos_v = 0 ;

  pos_ufloat = 0 ;
  pos_vfloat = 0 ;

  graphics = false ;

  winu = 1;
  winv = 1;
}

CCorner::CCorner() : vpTracker()
{
  init() ;
}

/*!
  \brief constructor with initialization of the dot location

  \param u : dot location (column)
  \param v : dot location (row)
 */
CCorner::CCorner(const unsigned int u, const unsigned int v) : vpTracker()
{
  init() ;

  pos_u = u ;
  pos_v = v ;

  pos_ufloat = u ;
  pos_vfloat = v ;

}

/*!
  \brief constructor with initialization of the dot location

  \param u : dot location (column)
  \param v : dot location (row)
 */
CCorner::CCorner(const double u,const  double v) : vpTracker()
{

  init() ;

  pos_u = (unsigned int)u ;
  pos_v = (unsigned int)v ;

  pos_ufloat = u ;
  pos_vfloat = v ;

}

/*!
  \brief copy constructor
 */
CCorner::CCorner(const CCorner& c)  : vpTracker()
{
  *this = c ;
}


/*!
  \brief copy operator
 */
CCorner&
CCorner::operator=(const CCorner& pt)
{
  pos_u = pt.pos_u ;
  pos_v = pt.pos_v ;

  pos_ufloat = pt.pos_ufloat ;
  pos_vfloat = pt.pos_vfloat ;

  graphics = pt.graphics ;
  
  winu = pt.winu;
  winv = pt.winv;

  return *this ;
}

bool
CCorner::operator!=(const CCorner& m)
{
  return ((pos_u!=m.pos_v) || (pos_v!=m.pos_v)) ;
}

bool
CCorner::operator==(const CCorner& m)
{
  return ((pos_u==m.pos_u) && (pos_v==m.pos_v)) ;
}

void
CCorner::setWin(unsigned int _winu, unsigned int _winv)
{
	if(_winu !=0)
		winu = _winu;
	if(_winv !=0)
		winv = _winv;
}

int
CCorner::initTracking(vpImage<unsigned char>& I, unsigned int size)
{
	unsigned int uc, vc;
	vpMouseButton::vpMouseButtonType button;
    vpImagePoint ip;
	vpDisplay::getClick(I,ip,button);
	
	if(button != vpMouseButton::button1)
		return 1;

	detectCorner(I, ip.get_u(), ip.get_v(), size, size);
	
	return 0;
}

void
CCorner::initTracking(vpImage<unsigned char>& I, unsigned int u, unsigned int v,
                     unsigned int size)
{
	detectCorner(I, u, v, size, size);
}

void
CCorner::detectCorner(vpImage<unsigned char> &I, double _u, double _v, unsigned int _winu, unsigned int _winv)
{
	if(_u != -1)
		pos_ufloat = _u;
	if(_v != -1)
		pos_vfloat = _v;
	if(_winu != 0)
		winu = _winu;
	if(_winv != 0)
		winv = _winv;
/*
	if (graphics)
		vpDisplay::displayRectangle(I, _v-winv, _u-winu, 2*winv, 2*winu, vpColor::green, false, (int)(1+vpMath::maximum(I.getWidth(), I.getHeight())*0.001));
*/

	std::vector<cv::Point2f> corners;
	corners.push_back(cv::Point2f(pos_ufloat,pos_vfloat));
	
	cv::Mat Img = cv::Mat( cv::Size(I.getCols(), I.getRows()), CV_8UC1, I.bitmap );

	cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.03 );

	cv::cornerSubPix(Img, corners, cv::Size(winu, winv), cv::Size(-1,-1), criteria);
	
	pos_ufloat = corners[0].x;
	pos_vfloat = corners[0].y;

	pos_u = cvRound(pos_ufloat);
	pos_v = cvRound(pos_vfloat);
/*
	if (graphics==true)
	{
	      vpDisplay::displayPoint(I,pos_v,pos_u,vpColor::green) ;
	}
 */
}

/*void CCorner::display(vpImage<unsigned char> &I, vpColor col)
{
	if (graphics)
	{
	      //vpDisplay::displayPoint(I, pos_v, pos_u, col) ;
		vpDisplay::displayCross(I, pos_v, pos_u, 10*(int)(1+vpMath::maximum(I.getWidth(), I.getHeight())*0.001), col, (int)(1+vpMath::maximum(I.getWidth(), I.getHeight())*0.001)) ;
	      //vpDisplay::displayRectangle(I, pos_v-winv, pos_u-winu, 2*winv, 2*winu, col);
	}
}*/

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


