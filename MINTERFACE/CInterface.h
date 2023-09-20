#ifndef CCoucheInterface
#define CCoucheInterface

enum build {Free=0,Professional=1,Premium=2,Developer=3};
const build buildVersion = Developer;

#include <visp/vpConfig.h>
//#include <visp/vpDebug.h>
#include <visp/vpIoTools.h>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iomanip>
#include <set>
#include <list>
#include <vector>
#include <map>
#include <string>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

//#include <visp/vpDisplayOpenCV.h>
//#include <visp/vpDisplayX.h>
//#include <visp/vpDisplayGDI.h>
//#include <visp/vpDisplayGTK.h>
//#include <visp/vpDisplayD3D.h>

#include <visp/vpMouseButton.h>

#include <MIXEDVISION/CMire.h>

#include <visp/vpDot2.h>
#include <MIXEDVISION/CRing.h>
#include <MIXEDVISION/CCorner.h>

//#include "../GDM/COmni.h"
//#include "../GDM/COmniXml.h"
#include <MIXEDVISION/CPerspective.h>
//#include "../GDM/CPerspectiveXml.h"
#include <MIXEDVISION/CParaboloid.h>
//#include "../GDM/CParaboloidXML.h"

#include <MIXEDVISION/CCalibrationStereo.h>
#include <MIXEDVISION/CCalibrationOmni.h>
#include <MIXEDVISION/CCalibrationPerspective.h>
#include <MIXEDVISION/CCalibrationParaboloid.h>

#include <MIXEDVISION/CPoseOmni.h>
#include <MIXEDVISION/CPosePerspective.h>
#include <MIXEDVISION/CPoseParaboloid.h>

#include <MIXEDVISION/commun.h>

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CInterface
{
public:

	bool *tabImagesActives, *listImagesInitCam;
	bool calibDone, systemePrepare, systemeInit, systemeAndCalibStructsInit, listImagesInit, detectionInit;
	MireType tm;
	ParamMire pm;
    int nbCams, nbShot; //1 shot = nbCams-uplet d'images
	int nbCamParImage;

	std::vector<ModelType> cameras;
	CModel **cam;
    std::map<unsigned int, std::vector<bool> > activeIntrinsicParameters, activeExtrinsicParameters;
    std::map<unsigned int, std::vector<double> > initBaseIntrinsicParameters, initExtrinsicParameters;
	vpHomogeneousMatrix *ciMc1;

	CModelStereo systeme;

	vpTracker *priminit;
	CMire mireinit;
	vpList<CMire> *ensemblemires;
	vpList<bool> *mireUtilisee;
	CCalibrationModel **table_cal;
	CCalibrationStereo *table_calStereo;
	
	double grayLevelPrecision, sizePrecision;
	unsigned int winu, winv;
	bool reloadPoints;

	unsigned int *imLarg, *imHaut;
	
	std::vector<std::string> *nomsFichiers;
    
    vpHomogeneousMatrix **cMoTmp;
    CMire **mirecour;

	unsigned int nbPlansMires;

	// Statistiques de resultat du calibrage (nbIter, err moy, err std)
	calibStats resCalibStats;

	CInterface();
	~CInterface();

	void init();
	void libere();
	void reset();

	void setGrayLevelPrecision(double _grayLevelPrecision) {grayLevelPrecision = _grayLevelPrecision; initParamPrim();}
	void setSizePrecision(double _sizePrecision) {sizePrecision = _sizePrecision; initParamPrim();}
	void setWinSize(unsigned int _winu, unsigned int _winv) {winu = _winu; winv = _winv; initParamPrim();}
	void setReloadPoints() {reloadPoints = true;};
	
    int setTypeMire(MireType _tm);//deprecated
	int setTypeMire(MireType _tm, ParamMire _pm);
	int addMire(MireType _tm, ParamMire _pm, PoseVector _pv);
	int addCamera(ModelType tc);

    int setActiveDistorsionParameters(unsigned int indexCamera, bool active_k1, bool active_k2, bool active_k3, bool active_k4, bool active_k5);
    int setActiveExtrinsicParameters(unsigned int index_ciMc1, bool active_tX, bool active_tY, bool active_tZ, bool active_rX, bool active_rY, bool active_rZ);
    int setInitExtrinsicParameters(unsigned int index_ciMc1, double init_tX, double init_tY, double init_tZ, double init_rX, double init_rY, double init_rZ);
    int setInitBaseIntrinsicParameters(unsigned int indexCam, double init_au, double init_av, double init_u0, double init_v0);

	int deleteCamera(int indexCamera);
	
	void setFOO();
	void unsetFOO();
	
	int initParamPrim();
	int prepareSysteme();
	int initSysteme(unsigned int *imLarg = NULL, unsigned int *imHaut = NULL);
	int initSystemeAndCalibStructs(unsigned int *imLarg, unsigned int *imHaut);

	//int calibSequentielle();
    int initCalib();
	int loadPoints(int *imFirst, std::vector<std::string> *nomsFichiers = NULL);
    int initDetection(int icam, int im);
    int detectionDUnDesPoints(int icam, int im, unsigned int curPt, s_SImagePoint & ip);
		int ignorerUnDesPoints(int icam, int im, unsigned int curPt);
    int preCalib(int icam, int numImage);
    //int afficheMireEtPosePreCalib(vpImage<unsigned char> &I, int icam);
    int detectionTousPoints(int icam, int numImage);
    int postCalib(int icam, int numImage);
    int finalizePostCalib(int icam, int numImage, int retour, std::string dir = std::string(""));
    //int afficheMireDetectee(vpImage<unsigned char> &I);
    int getMireEtPosePreCalib(int icam, int numImage, std::vector<SImagePoint> & listPtsMire, std::vector<SImagePoint> & listPtsPose);
    int getMireDetectee(int icam, int numImage, std::vector<SImagePoint> & ptsMire);
    int getMireReproj(int icam, int numImage, std::vector<SImagePoint> & ptsMire);

	//int clickImage(int numImage, int indexCamera);

	int updateCalib();

	int recomputePoints();

	int setListImages(std::vector<std::string> _nomsFichiers, int indexCamera);

	int reprojMire(std::string chemin); //enregistrement en ppm

	bool isCalib() {return calibDone;}

	std::string getResult();

	int loadXML(std::string nomFichier);
	int saveXML(std::string nomFichier);
	
	int setListImagesActives(bool *_tabool);

	int getListPosesMires(std::list<MatricePose *> &listPoses);
	int getListPosesRelativesCameras(std::list<MatricePose *> &listPoses);
	int getListTypesCameras(std::list<ModelType> &listTypesCameras);
	
	//int reset();
};
#endif
