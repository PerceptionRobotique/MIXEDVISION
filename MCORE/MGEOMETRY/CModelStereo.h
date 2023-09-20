#ifndef AFX_CModelStereo
#define AFX_CModelStereo

#include <MIXEDVISION/CModel.h>
#include <MIXEDVISION/COmni.h>

#include <visp/vpHomogeneousMatrix.h>

//!< Modele stereo : N cameras et N-1 matrices de changement de repere camera_1->camera_i

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CModelStereo
{
public:
	CModel **cam; //!<Tableau de pointeurs sur CModel
	vpHomogeneousMatrix *ciMc1; //!<Tableau de matrices de changement de repere

	unsigned int nbcams; //!<Nombre de cameras

	CModelStereo(unsigned int _nbcams);
	~CModelStereo();

	void init(unsigned int _nbcams);

	void setCamera(unsigned int i, CModel* _cam);
	void setciMc1(unsigned int i, vpHomogeneousMatrix & M);
	int get_nbcams() {return nbcams;}
};

#endif

