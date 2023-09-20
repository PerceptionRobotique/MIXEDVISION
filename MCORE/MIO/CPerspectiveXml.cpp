#include <MIXEDVISION/CPerspectiveXml.h>

void CPerspectiveXml::operator<<(CPerspective &cam)
{
	CModelXml::operator<<((CModel*)&cam);
}

void CPerspectiveXml::operator>>(CPerspective &cam)
{
	CModelXml::operator>>((CModel*)&cam);
}
