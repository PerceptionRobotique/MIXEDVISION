#ifndef AFX_CParaboloid
#define AFX_CParaboloid

#include <MIXEDVISION/CModel.h>

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CParaboloid : public CModel
{
protected:
	double h;

public:
	CParaboloid(double au=0,double av=0,double u0=0,double v0=0,double h=0.5, double k1=0,double k2=0,double k3=0,double k4=0,double k5=0);
	~CParaboloid();
	
	void init(double au,double av,double u0,double v0,double h, double k1=0,double k2=0,double k3=0,double k4=0,double k5=0);
	double geth(){return h;};
	void project3DImage(CPoint &);
	void projectImageMiroir(CPoint & P, double & Xm, double & Ym, double & Zm);
	void project3DMiroir(CPoint & P, double & Xm, double & Ym, double & Zm);
	
	CParaboloid& operator=(const CParaboloid& cam);
};

#endif
