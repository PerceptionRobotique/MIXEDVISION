#include <MIXEDVISION/CModelStereo.h>

CModelStereo::CModelStereo(unsigned int _nbcams)
{
	init(_nbcams);
}

void CModelStereo::init(unsigned int _nbcams)
{
	nbcams = _nbcams;
	if(nbcams)
	{
		cam = new CModel *[nbcams];
		ciMc1 = new vpHomogeneousMatrix[nbcams]; //attention ciMc1[0] est inutilise
		ciMc1[0].eye(); // au cas où...
	}
}

CModelStereo::~CModelStereo()
{
	if(nbcams)
	{
// marche pô		delete [] cam; // ou delete cam;
//		delete [] ciMc1;
	}
}

void CModelStereo::setCamera(unsigned int i, CModel* _cam)
{
	cam[i] = _cam;
}

void CModelStereo::setciMc1(unsigned int i, vpHomogeneousMatrix & M)
{
	if( (nbcams > 1) && (i > 1) && (i <= nbcams) )
		ciMc1[i-1] = M;
}
