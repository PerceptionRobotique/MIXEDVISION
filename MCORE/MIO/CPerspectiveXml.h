#ifndef AFX_CPERSPXML
#define AFX_CPERSPXML

#include <MIXEDVISION/CModelXml.h>
#include <MIXEDVISION/CPerspective.h>

class CPerspectiveXml : public CModelXml
{
private:

public:	
	CPerspectiveXml(std::string fic):CModelXml(fic){};
	void operator<<(CPerspective&);
	void operator>>(CPerspective&);
};

#endif
