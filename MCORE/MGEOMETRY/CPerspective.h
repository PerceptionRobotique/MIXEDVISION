#ifndef AFX_CPerspective
#define AFX_CPerspective

#include <MIXEDVISION/CModel.h>

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CPerspective : public CModel
{
	public:
//	CPerspective(CPerspective &c){ init(c); };
	CPerspective(double au=0,double av=0,double u0=0,double v0=0, double k1=0,double k2=0,double k3=0,double k4=0,double k5=0);
	~CPerspective();

	void project3DImage(CPoint &);

	//virtual friend std::ostream& operator << (std::ostream & os, const CPerspective &cam);	
	std::ostream& operator << (std::ostream & os);	
};

#endif
