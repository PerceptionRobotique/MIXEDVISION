#include <MIXEDVISION/CParaboloid.h>

CParaboloid::CParaboloid(double au,double av,double u0,double v0,double hh, double k1, double k2, double k3, double k4, double k5):CModel(au,av,u0,v0, k1, k2, k3, k4, k5)
{
	type = Paraboloid;
	name = "Paraboloid";
	h = hh;
}

CParaboloid::~CParaboloid()
{
}

void CParaboloid::init(double au,double av,double u0,double v0,double hh, double k1, double k2, double k3, double k4, double k5)
{
	CModel::init(au,av,u0,v0, k1, k2, k3, k4, k5);
	type = Paraboloid;
	name = "Paraboloid";
	h=hh;
}

void CParaboloid::project3DImage(CPoint & P)
{
	double X = P.get_X(), Y = P.get_Y(), Z = P.get_Z(), fact;
	fact = h / (sqrt(X*X+Y*Y+Z*Z)+Z);

	P.set_x(fact * X);
	P.set_y(fact * Y);
	//P.set_z(fact * Z); //sur le miroir...
}

void CParaboloid::projectImageMiroir(CPoint & P, double & Xm, double & Ym, double & Zm)
{
	double x = P.get_x(), y = P.get_y();
	
	Xm = x;
	Ym = y;
	Zm = (h*h-x*x-y*y)/(2*h);//((-(h*h)+(x*x+y*y))/(2*h));
	//Zm = (x*x+y*y)/(4*h*h);
}

void CParaboloid::project3DMiroir(CPoint & P, double & Xm, double & Ym, double & Zm)
{
	double X = P.get_X(), Y = P.get_Y(), Z = P.get_Z(), fact;
	fact = h / (sqrt(X*X+Y*Y+Z*Z)+Z);
	
	Xm = fact * X;
	Ym = fact * Y;
	Zm = fact * Z;
}

CParaboloid& CParaboloid::operator=(const CParaboloid& cam)
{
	CModel::operator=(cam);
	this->h = cam.h;
	return *this;
}

