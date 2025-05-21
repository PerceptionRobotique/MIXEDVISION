#include <MINTERFACE/CInterface.h>

#include <MIXEDVISION/CModelStereoXml.h>

#define AFFDETAILSTXT

CInterface::CInterface() : systeme(0)
{
	init();
}

CInterface::~CInterface()
{
	libere();
}

void CInterface::init()
{
	nbCamParImage = 1;
	
	systeme = CModelStereo(0);
	
	calibDone = false;
	systemeInit = false;
  systemePrepare = false;
	systemeAndCalibStructsInit = false;
    detectionInit = false;
	tm = MIRE_DOTS;
	nbCams = 0;
	nomsFichiers = NULL;	
	ciMc1 = NULL;
	priminit = NULL;
	
	listImagesInit = false;
	
	cam = NULL;
	table_cal = NULL;
	table_calStereo = NULL;
	ensemblemires = NULL;
	mireUtilisee = NULL;
    cMoTmp = NULL;
	
	grayLevelPrecision = 0.7;
	sizePrecision = 0.7;
	
	winu = winv = 11;//3;//7;//5;
	
	reloadPoints = false;	

	nbPlansMires = 1;
}

void CInterface::libere()
{
	if(tabImagesActives != NULL)
		delete [] tabImagesActives;
	if(listImagesInitCam != NULL)
		delete [] listImagesInitCam;
	if(cam != NULL)
	{
		for (int icam = 0; icam < nbCams; icam++)
			if(cam[icam] != NULL)
				delete cam[icam];
		delete [] cam;
	}
	if(ciMc1 != NULL)
		delete [] ciMc1;
	if(ensemblemires != NULL)
		delete [] ensemblemires;
	if(mireUtilisee != NULL)
		delete [] mireUtilisee;
	if(nomsFichiers != NULL)
		delete [] nomsFichiers;
	
	if(table_calStereo != NULL)
	{
		delete [] table_calStereo;
	}
	
	if(table_cal != NULL)
	{
		for(int icam = 0 ; icam < nbCams ; icam++)
			delete [] table_cal[icam];
		delete [] table_cal;
	}
	
	if(priminit != NULL)
		delete priminit;
    if(cMoTmp != NULL)
    {
        for (int i = 0; i < nbCams; i++)
            delete [] cMoTmp[i];
        delete [] cMoTmp;
    }
    if(mirecour != NULL)
    {
        for (int i = 0; i < nbCams; i++)
            delete [] mirecour[i];
        delete [] mirecour;
    }
	
    activeIntrinsicParameters.clear();
    
	//By DD : 
	cameras.clear();
	nbCams=0;
}

void CInterface::reset()
{
	libere();
	init();
}

/*
 Initialise le type de mire
 retourne : 
 0 si tout OK
 */

int CInterface::setTypeMire(MireType _tm)
{
	char fichier[256];
	
	tm = _tm;
	
	sprintf(fichier, "parametresMire%d.txt", tm);
	std::string nomfichiermire(fichier);
	
	mireinit.initMire(nomfichiermire);
	
	initParamPrim();
	
	return 0;
}

/*
 Initialise le type de mire
 retourne : 
 0 si tout OK
 */
int CInterface::setTypeMire(MireType _tm, ParamMire _pm)
{
	char fichier[256];
	
	tm = _tm;
	pm = _pm;
	
	mireinit.initMire(tm, pm);
	
	initParamPrim();

	nbPlansMires = 1;

	return 0;
}

/*
 Ajoute une mire plane rigidement liee a la premiere (donnee avec setTypeMire)
 retourne : 
 0 si tout OK
 */
int CInterface::addMire(MireType _tm, ParamMire _pm, PoseVector _pv)
{
	nbPlansMires++;

	mireinit.addPointsMire(_tm, _pm, _pv);
		
	return 0;
}

/*
 Ajoute une caméra
 retourne : 
 0 si tout OK
 */
int CInterface::addCamera(ModelType tc)
{
	cameras.push_back(tc);
	nbCams++;
	systemeInit = false;
  systemePrepare = false;
	listImagesInit = false;
	
	return 0;
}

/*
 active/desactive les parametres de distorsion de la camera dont l'index est donne
 retourne : 
 0 si tout OK
 */
int CInterface::setActiveDistorsionParameters(unsigned int indexCamera, bool active_k1, bool active_k2, bool active_k3, bool active_k4, bool active_k5)
{
    activeIntrinsicParameters[indexCamera].resize(5);
	activeIntrinsicParameters[indexCamera][0] = active_k1;
    activeIntrinsicParameters[indexCamera][1] = active_k2;
    activeIntrinsicParameters[indexCamera][2] = active_k3;
    activeIntrinsicParameters[indexCamera][3] = active_k4;
    activeIntrinsicParameters[indexCamera][4] = active_k5;
	
	return 0;
}

/*
 active/desactive les parametres extrinseques à estimer
 paramètres :
 index_ciMc1 : 1 pour la transformation entre la caméra de reference et la deuxieme
               2 pour la transformation entre la caméra de reference et la troisieme
               3 etc
 retourne : 
 0 si tout OK
 */
int CInterface::setActiveExtrinsicParameters(unsigned int index_ciMc1, bool active_tX, bool active_tY, bool active_tZ, bool active_rX, bool active_rY, bool active_rZ)
{
    activeExtrinsicParameters[index_ciMc1].resize(6);
    activeExtrinsicParameters[index_ciMc1][0] = active_tX;
    activeExtrinsicParameters[index_ciMc1][1] = active_tY;
    activeExtrinsicParameters[index_ciMc1][2] = active_tZ;
    activeExtrinsicParameters[index_ciMc1][3] = active_rX;
    activeExtrinsicParameters[index_ciMc1][4] = active_rY;
    activeExtrinsicParameters[index_ciMc1][5] = active_rZ;
    
    return 0;
}

/*
 initialise les parametres extrinseques qui seront optimises ou non dans le calibrage
 retourne : 
 0 si tout OK
 */
int CInterface::setInitExtrinsicParameters(unsigned int index_ciMc1, double init_tX, double init_tY, double init_tZ, double init_rX, double init_rY, double init_rZ)
{
    initExtrinsicParameters[index_ciMc1].resize(6);
    initExtrinsicParameters[index_ciMc1][0] = init_tX;
    initExtrinsicParameters[index_ciMc1][1] = init_tY;
    initExtrinsicParameters[index_ciMc1][2] = init_tZ;
    initExtrinsicParameters[index_ciMc1][3] = init_rX;
    initExtrinsicParameters[index_ciMc1][4] = init_rY;
    initExtrinsicParameters[index_ciMc1][5] = init_rZ;
    
    return 0;
}

/*
 initialise les parametres intrinseques qui seront optimises ou non dans le calibrage
 retourne : 
 0 si tout OK
 */
int CInterface::setInitBaseIntrinsicParameters(unsigned int indexCam, double init_au, double init_av, double init_u0, double init_v0)
{
    initBaseIntrinsicParameters[indexCam].resize(4);
    initBaseIntrinsicParameters[indexCam][0] = init_au;
    initBaseIntrinsicParameters[indexCam][1] = init_av;
    initBaseIntrinsicParameters[indexCam][2] = init_u0;
    initBaseIntrinsicParameters[indexCam][3] = init_v0;
    
    return 0;
}


/*
 Supprime une caméra
 retourne : 
 0 si tout OK
 */
int CInterface::deleteCamera(int indexCamera)
{
	int icam=-1;
	std::vector<ModelType>::iterator it_mt = cameras.begin();
	while(++icam < indexCamera)
		it_mt++;
	
	cameras.erase(it_mt);
	nbCams--;
	systemeInit = false;
  systemePrepare = false;
	listImagesInitCam[indexCamera] = false;
	listImagesInit = false;
    
  activeIntrinsicParameters.erase(indexCamera);
	
	return 0;
}

void CInterface::setFOO()
{
	nbCamParImage = 4;
}

void CInterface::unsetFOO()
{
	nbCamParImage = 1;
}

int CInterface::prepareSysteme()
{
    systeme = CModelStereo(nbCams);
	if(cam != NULL)
	{
		for(int icam = 0 ; icam < nbCams ; icam++)
			delete cam[icam];
		delete [] cam;
	}
	cam = new CModel *[nbCams];
	
	if(ciMc1 != NULL)
		delete [] ciMc1;
	ciMc1 = new vpHomogeneousMatrix[nbCams];
    
    systemePrepare = true;
    
    return 0;
}

int CInterface::initSysteme(unsigned int *imLarg, unsigned int *imHaut)
{
	double au, av, u0, v0;
	prepareSysteme();
	
    //std::map<unsigned int, std::vector<double> >::iterator it_initExt;
	std::map<unsigned int, std::vector<double> >::iterator it_initBaseInt;
	for(int icam = 0 ; icam < nbCams ; icam++)
	{
        /*it_initExt = initExtrinsicParameters.find(icam);
        if(it_initExt != initExtrinsicParameters.end())
            ciMc1[icam].buildFrom(it_initExt->second[0], it_initExt->second[1], it_initExt->second[2], it_initExt->second[3], it_initExt->second[4], it_initExt->second[5]);
        else        */
            ciMc1[icam].eye();
        
		it_initBaseInt = initBaseIntrinsicParameters.find(icam);
		if(it_initBaseInt != initBaseIntrinsicParameters.end())
		{
			if(it_initBaseInt->second[0] != 0)
				au = it_initBaseInt->second[0];
			else
				au = 0.;
			if(it_initBaseInt->second[1] != 0)
				av = it_initBaseInt->second[1];
			else
				av = 0.;
			if(it_initBaseInt->second[2] != 0)
				u0 = it_initBaseInt->second[2];
			else
				u0 = 0.;
			if(it_initBaseInt->second[3] != 0)
				v0 = it_initBaseInt->second[3];
			else
				v0 = 0.;
		}
		else
			au = av = u0 = v0 = 0.;

		switch(cameras[icam])
		{
			case Omni :
				if(nbCamParImage == 1)
					cam[icam] = new COmni((au==0)?imHaut[icam]:au, (av==0)?imHaut[icam]:av, (u0==0)?imLarg[icam]/2.0:u0, (v0==0)?imHaut[icam]/2.0:v0, 1.0);
				else
					switch(icam)
				{	
					case 0 : cam[icam] = new COmni(imHaut[icam]/2.0, imHaut[icam]/2.0, imLarg[icam]/4.0, imHaut[icam]/4.0, 1.0); break;
					case 1 : cam[icam] = new COmni(imHaut[icam]/2.0, imHaut[icam]/2.0, 1.5*imLarg[icam], imHaut[icam]/4.0, 1.0); break;
					case 2 : cam[icam] = new COmni(imHaut[icam]/2.0, imHaut[icam]/2.0, 1.5*imLarg[icam], 1.5*imHaut[icam], 1.0); break;
					case 3 : cam[icam] = new COmni(imHaut[icam]/2.0, imHaut[icam]/2.0, imLarg[icam]/4.0, 1.5*imHaut[icam], 1.0); break;
					default: break;
				}
				break;
			case Persp : cam[icam] = new CPerspective((au==0)?imHaut[icam]:au, (av==0)?imHaut[icam]:av, (u0==0)?imLarg[icam]/2.0:u0, (v0==0)?imHaut[icam]/2.0:v0);
				//cam[icam]->setActiveDistorsionParameters(true,false,false,false,false);
				break;
			case Fisheye : cam[icam] = new COmni((au==0)?imHaut[icam]:au, (av==0)?imHaut[icam]:av, (u0==0)?imLarg[icam]/2.0:u0, (v0==0)?imHaut[icam]/2.0:v0, 1.6);
				//std::cout << "Fisheye init : " << cam[icam]->getau() << " " <<  cam[icam]->getav() << " " <<  cam[icam]->getu0() << " " <<  cam[icam]->getv0() << std::endl;
				break;
			case Paraboloid : 
				if(nbCamParImage == 1)
					cam[icam] = new CParaboloid((au==0)?imHaut[icam]*0.5:au, (av==0)?imHaut[icam]*0.5:av, (u0==0)?imLarg[icam]/2.0:u0, (v0==0)?imHaut[icam]/2.0:v0, 1.0);
				else
					switch(icam)
				{	
					case 0 : cam[icam] = new CParaboloid(imHaut[icam]*0.25, imHaut[icam]*0.25, imLarg[icam]/4.0, imHaut[icam]/4.0, 1.0); break;
					case 1 : cam[icam] = new CParaboloid(imHaut[icam]*0.25, imHaut[icam]*0.25, 1.5*imLarg[icam], imHaut[icam]/4.0, 1.0); break;
					case 2 : cam[icam] = new CParaboloid(imHaut[icam]*0.25, imHaut[icam]*0.25, 1.5*imLarg[icam], 1.5*imHaut[icam], 1.0); break;
					case 3 : cam[icam] = new CParaboloid(imHaut[icam]*0.25, imHaut[icam]*0.25, imLarg[icam]/4.0, 1.5*imHaut[icam], 1.0); break;
					default: break;
				}
				break;
			default:
				//std::cout << "initSysteme : Type de primitive inconnu" << std::endl;
				return -1;
				break;
		}
		systeme.setCamera(icam, cam[icam]);
		//ciMc1[icam].eye(4);
		systeme.setciMc1(icam+1, ciMc1[icam]);
	}
	systemeInit = true;
	
	return 0;
}

/*
 Initialise le système stéréo
 retourne : 
 Attention : bug possible avec l'init des vpHomogeneousMatrix
 0 si tout OK
 */
int CInterface::initSystemeAndCalibStructs(unsigned int *imLarg, unsigned int *imHaut)
{
	prepareSysteme();
	
	if(table_cal != NULL)
	{
		for(int icam = 0 ; icam < nbCams ; icam++)
			delete [] table_cal[icam];
		delete [] table_cal;
	}
	table_cal = new CCalibrationModel*[nbCams];
	
	//	for(int icam = 0 ; icam < nbCams ; icam++)
	// = vpHomogeneousMatrix(0,0,0,0,0,0);
	
    //std::map<unsigned int, std::vector<double> >::iterator it_initExt;
	for(int icam = 0 ; icam < nbCams ; icam++)
	{
        /*it_initExt = initExtrinsicParameters.find(icam);
        if(it_initExt != initExtrinsicParameters.end())
            ciMc1[icam].buildFrom(it_initExt->second[0], it_initExt->second[1], it_initExt->second[2], it_initExt->second[3], it_initExt->second[4], it_initExt->second[5]);
        else        */
            ciMc1[icam].eye();
		//		std::cout << cameras[icam] << " " << imHaut[icam] << " " << imLarg[icam] << std::endl;
		switch(cameras[icam])
		{
			case Omni : //cam[icam] = new COmni(imHaut[icam]/2.0, imHaut[icam]/2.0, imLarg[icam]/2.0, imHaut[icam]/2.0, 1.0);
				if(nbCamParImage == 1)
					cam[icam] = new COmni(imHaut[icam], imHaut[icam], imLarg[icam]/2.0, imHaut[icam]/2.0, 1.0);
				else
					switch(icam)
				{	
					case 0 : cam[icam] = new COmni(imHaut[icam]/2.0, imHaut[icam]/2.0, imLarg[icam]/4.0, imHaut[icam]/4.0, 1.0); break;
					case 1 : cam[icam] = new COmni(imHaut[icam]/2.0, imHaut[icam]/2.0, 1.5*imLarg[icam], imHaut[icam]/4.0, 1.0); break;
					case 2 : cam[icam] = new COmni(imHaut[icam]/2.0, imHaut[icam]/2.0, 1.5*imLarg[icam], 1.5*imHaut[icam], 1.0); break;
					case 3 : cam[icam] = new COmni(imHaut[icam]/2.0, imHaut[icam]/2.0, imLarg[icam]/4.0, 1.5*imHaut[icam], 1.0); break;
					default: break;
				}
				table_cal[icam] = new CCalibrationOmni[nbShot];
				for (int j = 0; j < nbShot; j++)
					table_cal[icam][j].setCamera(cam[icam]);
				
				break;
			case Persp : cam[icam] = new CPerspective(imHaut[icam], imHaut[icam], imLarg[icam]/2.0, imHaut[icam]/2.0);
				table_cal[icam] = new CCalibrationPerspective[nbShot];
				for (int j = 0; j < nbShot; j++)
					table_cal[icam][j].setCamera(cam[icam]);
				break;
			case Fisheye : cam[icam] = new COmni(imHaut[icam], imHaut[icam], imLarg[icam]/2.0, imHaut[icam]/2.0, 1.6);
				table_cal[icam] = new CCalibrationOmni[nbShot];
				for (int j = 0; j < nbShot; j++)
					table_cal[icam][j].setCamera(cam[icam]);
				break;
			case Paraboloid : 
				if(nbCamParImage == 1)
					cam[icam] = new CParaboloid(imHaut[icam]*0.5, imHaut[icam]*0.5, imLarg[icam]/2.0, imHaut[icam]/2.0, 1.0);
				else
					switch(icam)
				{	
					case 0 : cam[icam] = new CParaboloid(imHaut[icam]*0.25, imHaut[icam]*0.25, imLarg[icam]/4.0, imHaut[icam]/4.0, 1.0); break;
					case 1 : cam[icam] = new CParaboloid(imHaut[icam]*0.25, imHaut[icam]*0.25, 1.5*imLarg[icam], imHaut[icam]/4.0, 1.0); break;
					case 2 : cam[icam] = new CParaboloid(imHaut[icam]*0.25, imHaut[icam]*0.25, 1.5*imLarg[icam], 1.5*imHaut[icam], 1.0); break;
					case 3 : cam[icam] = new CParaboloid(imHaut[icam]*0.25, imHaut[icam]*0.25, imLarg[icam]/4.0, 1.5*imHaut[icam], 1.0); break;
					default: break;
				}
				table_cal[icam] = new CCalibrationParaboloid[nbShot];
				for (int j = 0; j < nbShot; j++)
					table_cal[icam][j].setCamera(cam[icam]);
				break;
			default:
				//std::cout << "initSysteme : Type de primitive inconnu" << std::endl;
				return -1;
				break;
		}
		systeme.setCamera(icam, cam[icam]);
		//ciMc1[icam].eye(4);
		systeme.setciMc1(icam+1, ciMc1[icam]);
	}
	systemeInit = true;
	
	return 0;
}

/*
 Initialise les paramètres de primitive de la mire
 retourne : 
 0 si tout OK
 -1 si le type de primitive est inconnu
 */
int CInterface::initParamPrim()
{
	if(priminit != NULL)
		delete priminit;
	
	switch(tm)
	{
		case MIRE_DOTS :
			priminit = new vpDot2();
			
			((vpDot2 *)priminit)->setGraphics(false) ;
			((vpDot2 *)priminit)->setGrayLevelPrecision(grayLevelPrecision);
			((vpDot2 *)priminit)->setSizePrecision(sizePrecision);
			//			((vpDot2 *)priminit)->setEllipsoidShapePrecision(shapePrecision);
			break;
		case MIRE_CORNERS :
			priminit = new CCorner();
			
			((CCorner *)priminit)->setGraphics(false) ;
			((CCorner *)priminit)->setWin(winu, winv) ;
			break;
		case MIRE_RINGS :
			priminit = new CRing();
			
			((CRing *)priminit)->setGraphics(false) ;
			((CRing *)priminit)->setGrayLevelPrecision(grayLevelPrecision);
			((CRing *)priminit)->setSizePrecision(sizePrecision);
			//			((CRing *)priminit)->setEllipsoidShapePrecision(shapePrecision);
			break;
		default :
			return -1;
			break;
	}
	
	mireinit.initParametresPrimitive(priminit);
	
	return 0;
}

/*
 Effectue le calibrage complet de toutes les images/caméras et du banc
 retourne :
 0 si tout OK
 -1 si la liste d'images de chaque caméra n'est pas initialisée
 */
int CInterface::initCalib()
{
	if(!listImagesInit)
		return -1;
	
	if(!systemeInit)
	{
		initSystemeAndCalibStructs(imLarg, imHaut);
		if(ensemblemires != NULL)
		{
			delete [] ensemblemires;
			delete [] mireUtilisee;
            for (int i = 0; i < nbCams; i++)
                delete [] cMoTmp[i];
            delete [] cMoTmp;
            for (int i = 0; i < nbCams; i++)
                delete [] mirecour[i];
            delete [] mirecour;
		}
		ensemblemires = new vpList<CMire>[nbCams];
		mireUtilisee = new vpList<bool>[nbCams];
        
        cMoTmp = new vpHomogeneousMatrix *[nbCams];
        for (int i = 0; i < nbCams; i++)
            cMoTmp[i] = new vpHomogeneousMatrix[nbShot];
        mirecour = new CMire *[nbCams];
        for (int i = 0; i < nbCams; i++)
            mirecour[i] = new CMire[nbShot];
        
        
	}
	
	return 0;
}

/*
 Effectue le calibrage complet de toutes les images/caméras et du banc
 retourne :
 0 si tout OK
 -1 si la liste d'images de chaque caméra n'est pas initialisée
 */

int CInterface::loadPoints(int *imFirst, std::vector<std::string> *nomsFichiers)
{
    if(!systemeInit)
        initCalib();
    
    reloadPoints = true;
    
    std::ostringstream s;
		
	for(int icam = 0 ; icam < nbCams ; icam++)
	{
		vpList<vpHomogeneousMatrix> lcMoTmp;
		s.str("");
		if( (nomsFichiers != NULL) && (icam < nomsFichiers->size()) && ((*nomsFichiers)[icam].length() > 0) )
			s << (*nomsFichiers)[icam];
		else		
			s << "points" << icam << ".txt";
		imFirst[icam]=CMire::initTrackingMires(s.str(), ensemblemires[icam], lcMoTmp);
		lcMoTmp.front();
		ensemblemires[icam].front();
    mireUtilisee[icam].front();
		for (int im=0; im < imFirst[icam]; im++)
		{
			table_cal[icam][im].clearPoint();
            if((ensemblemires[icam].value()).points.nbElement() == 0)
            {
                mireUtilisee[icam].addRight(false);
            }
            else
            {
                mireUtilisee[icam].addRight(true);
                table_cal[icam][im].cMo = lcMoTmp.value();
                table_cal[icam][im].setListPoints((ensemblemires[icam].value()).points);
                mirecour[icam][im].points = (ensemblemires[icam].value()).points; //pas propre...
            }
			
			lcMoTmp.next();
			ensemblemires[icam].next();
		}
	}
    
    reloadPoints = false;

	return 0;
}

/*
 Initialisation de la mire dans une image pour une caméra donnée
 (4 clics + contrôle visuel + possibilité de recommencer/laisser tomber)
 retourne :
 1 si l'utilisateur souhaite ignorer cette image (toutes les autres images du même shot le sont aussi)
 0 si tout OK
 -4 si l'image ne peut être chargée
 -3 si le modèle est inconnu
 -2 s'il n'est pas possible d'afficher l'image
 -1 si le type de mire est inconnu
 */
int CInterface::preCalib(int icam, int numImage)
{
    int retour = 0;

    initSysteme(imLarg, imHaut);
    
    //Calcul de pose avec les 4 points d'init puis calib locale avec ces 4 pts
    vpHomogeneousMatrix cMo ;
    CCalibrationModel *calib;
    CPose *pose;
		CModel *curCam;
    
    switch(cameras[icam])
    {
        case Omni :
        case Fisheye : calib = new CCalibrationOmni((COmni *)cam[icam]);
            pose = new CPoseOmni();
						curCam = new COmni(); ((COmni *)curCam)->operator=(*((COmni *)cam[icam]));
            break;
        case Persp :   calib = new CCalibrationPerspective((CPerspective *)cam[icam]);
            pose = new CPosePerspective();
						curCam = new CPerspective(); ((CPerspective *)curCam)->operator=(*((CPerspective *)cam[icam]));
            break;
        case Paraboloid : calib = new CCalibrationParaboloid((CParaboloid *)cam[icam]);
            pose = new CPoseParaboloid();
						curCam = new CParaboloid(); ((CParaboloid *)curCam)->operator=(*((CParaboloid *)cam[icam]));
            break;
        default:
            return -3;
            break;
    }
    
    calib->clearPoint();
    pose->clearPoint() ;
    
    pose->setListPoints(mirecour[icam][numImage].pointsref);
    calib->setListPoints(mirecour[icam][numImage].pointsref);

    try
    {    
    	//Initialisation de la pose, par une méthode linéaire généralement
			//if(nbPlansMires == 1) //imposing the four not ignored target points are on the same plane, this test is not necessary
	    	pose->poseInit(cMo, curCam);
			//else
			//	cMo[2][3] = 0.01;

			cMo.print();
		  //Optimisation de la pose par VVS
		  //pose->setLambda(0.05);
		  pose->poseVirtualVS(curCam, cMo);
			//std::cout << "pose pts cliques : " << std::endl;
		  //cMo.print(); std::cout << std::endl;
		  cMoTmp[icam][numImage] = cMo;
		  //compute local calibration to match the calibration grid with the image
        //calib->setLambda(0.2);
        calib->calibVVS(curCam,cMoTmp[icam][numImage],true);
				//std::cout << "calib pts cliques : " << std::endl;
        //std::cout << curCam->getau() << " " << curCam->getav() << " "
         //<< curCam->getu0() << " " << curCam->getv0() << " "
        // << std::endl; //<< cam[icam].getXi()
        // cMoTmp[icam][numImage].print();std::cout << std::endl;
			switch(cameras[icam])
		  {
		      case Omni :
		      case Fisheye : ((COmni *)cam[icam])->operator=(*((COmni *)curCam));
		          break;
		      case Persp :  ((CPerspective *)cam[icam])->operator=(*((CPerspective *)curCam));
		          break;
		      case Paraboloid : ((CParaboloid *)cam[icam])->operator=(*((CParaboloid *)curCam));
		          break;
		      default:
		          return -3;
		          break;
		  }

    }
    catch(...)
    {
#ifdef AFFDETAILSTXT
        std::cout << "\nPose computation failed." << std::endl;
#endif
        retour = -1;
    }

    delete calib;
    delete pose;
    
	return retour;
}

int
CInterface::getMireDetectee(int icam, int numImage, std::vector<SImagePoint> & ptsMire)
{    
    mirecour[icam][numImage].getPrimitives(ptsMire);
    
    return 0;
}

int
CInterface::getMireEtPosePreCalib(int icam, int numImage, std::vector<SImagePoint> & ptsMire, std::vector<SImagePoint> & ptsPose)
{
    
    mirecour[icam][numImage].getPrimRefs(ptsMire);
    
    CPose *pose;
    
    switch(cameras[icam])
    {
        case Omni :
        case Fisheye :
            pose = new CPoseOmni();
            break;
        case Persp :
            pose = new CPosePerspective();
            break;
        case Paraboloid :
            pose = new CPoseParaboloid();
            break;
        default:
            return -3;
            break;
    }
    
    pose->getFrame(ptsPose, cMoTmp[icam][numImage], cam[icam], 0.2);
    
    delete pose;
    
    return 0;
}




/*
 Initialisation de la mire dans une image pour une caméra donnée
 (4 clics + contrôle visuel + possibilité de recommencer/laisser tomber)
 retourne :
 1 si l'utilisateur souhaite ignorer cette image (toutes les autres images du même shot le sont aussi)
 0 si tout OK
 -4 si l'image ne peut être chargée
 -3 si le modèle est inconnu
 -2 s'il n'est pas possible d'afficher l'image
 -1 si le type de mire est inconnu
 */
int CInterface::detectionTousPoints(int icam, int numImage)
{
	vpImage<unsigned char> I;
	int retour;
	
	try
	{
		//std::cout << nomsFichiers[icam][numImage] << std::endl;
		vpImageIo::read(I, nomsFichiers[icam][numImage]);
	}
	catch(...)
	{
		//		std::cout << "error image : " << e << std::endl;
		return -4;
	}
	
    //now we detect all dots of the grid
    switch(tm)
    {
        case MIRE_DOTS :
            ((vpDot2 *)priminit)->setGraphics(false) ;
            break;
        case MIRE_CORNERS :
            ((CCorner *)priminit)->setGraphics(false) ;
            break;
        case MIRE_RINGS :
            ((CRing *)priminit)->setGraphics(false) ;
            break;
        default :
            return -1;
    }
    mirecour[icam][numImage].initParametresPrimitive(priminit);
    mirecour[icam][numImage].initTrackingNoDisplay(I, cam[icam], cMoTmp[icam][numImage]);
		
    return 0;
}

int
CInterface::postCalib(int icam, int numImage)
{
    ensemblemires[icam].addRight(mirecour[icam][numImage]);
    mireUtilisee[icam].addRight(true);
    //we put the pose matrix in the current calibration structure
    table_cal[icam][numImage].cMo = cMoTmp[icam][numImage] ;
    table_cal[icam][numImage].setListPoints(mirecour[icam][numImage].points);
		
    return 0;
}

int
CInterface::finalizePostCalib(int icam, int numImage, int retour, std::string dir)
{
	if(retour)
	{
		mireUtilisee[icam].addRight(false);
		mirecour[icam][numImage].points.kill();
        ensemblemires[icam].addRight(mirecour[icam][numImage]);
	}
	
	std::ostringstream s;
	s.str("");
	if(dir.empty())
		s << "points" << icam << ".txt";
	else
		s << dir << "/points" << icam << ".txt";

	if(numImage == 0)
		CMire::save(s.str(), mirecour[icam][numImage], cMoTmp[icam][numImage], nbShot);
	else
		CMire::save(s.str(), mirecour[icam][numImage], cMoTmp[icam][numImage]);
	
	return 0;
}


/*
 Réalise l'étalonnage stéréo effectif
 retourne :
 0 si tout OK
 -1 si l'étalonnage stéréo échoue (dont non convergence)
 */
int CInterface::updateCalib()
{
	
	//std::cout << "3 initSystem" << std::endl;
	initSysteme(imLarg, imHaut);
	/*((COmni *)cam[0])->au /= ((COmni *)cam[0])->xi;
	 ((COmni *)cam[0])->av /= ((COmni *)cam[0])->xi;
	 ((COmni *)cam[0])->xi = 1.0;*/
	
	//CCalibrationOmni::setLambda(opt_lambda);
	
	// Calibrage individuel des caméras pour avoir de bonnes inits pour la stéréo
	// (paramètres intrinsèques : cam[icam] + pose de la mire p : table_cal[icam][p].cMo)
	//CCalibrationModel::setLambda(0.1);
    std::map<unsigned int, std::vector<bool> >::iterator it_actPar;
	for(int icam = 0 ; icam < nbCams ; icam++)
	{	
		std::cout << "1 calib indiv cam " << icam << std::endl;
		//Attention
		//si distorsions...
        it_actPar=activeIntrinsicParameters.find(icam);
        if(it_actPar != activeIntrinsicParameters.end())
            cam[icam]->setActiveDistorsionParameters(it_actPar->second[0], it_actPar->second[1], it_actPar->second[2], it_actPar->second[3], it_actPar->second[4]);
        
        for(int j = 0 ; j < nbShot ; j++)
            table_cal[icam][j].updateCamera(cam[icam]);
		////////////
        int nbPosesValides = 0;
        mireUtilisee[icam].front();
        while(!mireUtilisee[icam].outside())
        {
            if(mireUtilisee[icam].value())
                nbPosesValides++;
            mireUtilisee[icam].next();
        }
		int nbIter = CCalibrationModel::calibVVSMulti(nbShot, nbPosesValides, table_cal[icam], cam[icam], false);
#ifdef AFFDETAILSTXT
		std::cout << "camera " << icam << " : " << cam[icam]->name << std::endl;
		std::cout << "number of iterations : " << nbIter << std::endl;
		std::cout << "K : " << std::endl;
		std::cout << cam[icam]->getK() << std::endl;
		switch(cameras[icam])
		{
			case Paraboloid : std::cout << "h : " << ((CParaboloid *)cam[icam])->geth() <<  std::endl;
				break;
			case Fisheye :
			case Omni : std::cout << "xi : " << ((COmni *)cam[icam])->getXi() <<  std::endl;
				break;
			default:
				//std::cout << "initSysteme : Type de primitive inconnu" << std::endl;
				break;
		}
		
		/*if(icam > 0)
			std::cout << "c" << icam << "Mc1 : " << std::endl << systeme.ciMc1[icam] << std::endl;*/
		
		std::cout << std::endl;
#endif
	}
	
    //Mise en correspondance des shots de la camera 0 avec les autres
    unsigned int **match = new unsigned int *[nbCams];
    for(int icam = 1; icam < nbCams ; icam++)
    {
        match[icam] = new unsigned int [nbShot];
        std::list<unsigned int> pc1, pc2, tc1, tc2;
        std::list<unsigned int>::iterator it_pc1, it_pc2, it_pc1b, it_pc2b, it_tc1, it_tc2, it_tc2_sav;
        for(int p = 0 ; p < nbShot ; p++)
        {
            if(mirecour[0][p].points.nbElement() != 0)
                pc1.push_back(p);
            if(mirecour[icam][p].points.nbElement() != 0)
                pc2.push_back(p);
        }
        double dt = 0, dtmin = 100000000000.0, dr = 0, drmin = 100000000000.0;
        unsigned int *assoc = new unsigned int[nbShot];
        for(it_pc1 = pc1.begin() ; it_pc1 != pc1.end() ; it_pc1++)
        {
            tc1.clear();
            for(it_pc1b = pc1.begin() ; it_pc1b != pc1.end() ; it_pc1b++)
                if(*it_pc1 != *it_pc1b)
                    tc1.push_back(*it_pc1b);
            
            for(it_pc2 = pc2.begin() ; it_pc2 != pc2.end() ; it_pc2++)
            {
                vpHomogeneousMatrix cjMc1_ref = table_cal[icam][*it_pc2].cMo * table_cal[0][*it_pc1].cMo.inverse();
                tc2.clear();
                for(it_pc2b = pc2.begin() ; it_pc2b != pc2.end() ; it_pc2b++)
                    if(*it_pc2 != *it_pc2b)
                        tc2.push_back(*it_pc2b);
                
                dt = dr = 0.0;
                //memset(assoc, 0, nbShot*sizeof(unsigned int));
                for (int pp = 0; pp < nbShot; pp++)
                    assoc[pp] = pp;
                
                assoc[*it_pc1] = *it_pc2;
                for(it_tc1 = tc1.begin() ; it_tc1 != tc1.end() ; it_tc1++)
                {
                    double dtcurr = 0, dtcurrmin = 100000000000.0, drcurr = 0, drcurrmin = 100000000000.0;
                    for(it_tc2 = tc2.begin() ; it_tc2 != tc2.end() ; it_tc2++)
                    {
                        //calcul de l'ecart entre cjMc1(*it_pc1, *it_pc2) et cjMc1(*it_tc1, *it_tc2)
                        vpHomogeneousMatrix c1Mc1 = cjMc1_ref.inverse() * (table_cal[icam][*it_tc2].cMo * table_cal[0][*it_tc1].cMo.inverse());
                        vpPoseVector pv(c1Mc1);
                        dtcurr = sqrt(pv[0]*pv[0]+pv[1]*pv[1]+pv[2]*pv[2]);
                        drcurr = sqrt(pv[3]*pv[3]+pv[4]*pv[4]+pv[5]*pv[5]);
                        if( (dtcurr < dtcurrmin) &&  (drcurr < drcurrmin) )
                        {
                            dtcurrmin = dtcurr;
                            drcurrmin = drcurr;
                            it_tc2_sav = it_tc2;
                        }
                    }
                    dt += dtcurrmin;
                    dr += drcurrmin;
                    assoc[*it_tc1] = *it_tc2_sav;
                    tc2.erase(it_tc2_sav);
                }

                if( (dt < dtmin) && (dr < drmin) )
                {
                    dtmin = dt;
                    drmin = dr;
                    memcpy(match[icam], assoc, nbShot*sizeof(unsigned int));
                }
            }
        }
    }

#ifdef AFFDETAILSTXT
    std::cout << "matching result : " << std::endl;
    for(int icam = 1 ; icam < nbCams ; icam++)
    {
        for(int p = 0 ; p < nbShot ; p++)
        {
            std::cout << "c0(" << p << ") matched with c" << icam << "(" << match[icam][p] << ") :" << std::endl;
            vpHomogeneousMatrix ciMc1tmp = table_cal[icam][match[icam][p]].cMo * table_cal[0][p].cMo.inverse();
            
            std::cout << vpPoseVector(ciMc1tmp).t() << std::endl;
        }
    }
#endif
	//Initialisation des transformations relatives du modele stereo 
    std::map<unsigned int, std::vector<double> >::iterator it_initExt;
    std::map<unsigned int, std::vector<bool> >::iterator it_actExt;
	for(int icam = 0 ; icam < nbCams ; icam++)
	{
		if(icam != 0)
		{
			
			//Moyennage de la matrice de rotation et du vecteur translation
			vpMatrix ciRc1(3,3);
			//ciRc1.eye(3, 3);
			vpColVector citc1(3);
			for (int p = 0; p < nbShot; p++)
			{
				vpHomogeneousMatrix ciMc1tmp = table_cal[icam][match[icam][p]].cMo * table_cal[0][p].cMo.inverse();
#ifdef AFFDETAILSTXT
                std::cout << "ciMc1tmp " << p << " : " << std::endl << vpPoseVector(ciMc1tmp).t() << std::endl;
#endif
                
				for(int i = 0 ; i < 3 ; i++)
				{
					for(int j = 0 ; j < 3 ; j++)
						ciRc1[i][j] += ciMc1tmp[i][j];
					citc1[i] += ciMc1tmp[i][3];
				}
			}
			citc1 /= nbShot;
			
			vpMatrix V(3,3), U = ciRc1;
			vpColVector S(3);
#ifdef VISP_HAVE_GSL
			U.svd(S, V);
#else
			//on utilise la svd de OpenCV
			CvMat ciRc1Mat=cvMat(3, 3, CV_64F, ciRc1.data),
			UMat=cvMat(3, 3, CV_64F, U.data),
			SMat=cvMat(3, 1, CV_64F, S.data), 
			VMat=cvMat(3, 3, CV_64F, V.data);
			
			cvSVD(&ciRc1Mat, &SMat, &UMat, &VMat, CV_SVD_MODIFY_A);
#endif
			ciRc1 = U*V.t();
			for(int i = 0 ; i < 3 ; i++)
			{
				for(int j = 0 ; j < 3 ; j++)
					ciMc1[icam][i][j] = ciRc1[i][j];
				ciMc1[icam][i][3] = citc1[i];
			}
            
            it_actExt = activeExtrinsicParameters.find(icam);
            it_initExt = initExtrinsicParameters.find(icam);
            if(it_initExt != initExtrinsicParameters.end())
            {
                vpPoseVector r(ciMc1[icam]);
                
                for (int ddl=0; ddl<6; ddl++)
                {
                    if (it_actExt->second[ddl] == false)
                    {
                        r[ddl] = it_initExt->second[ddl];
                    }
                }
                
                ciMc1[icam].buildFrom(r);
            }

			//ancienne version
			//ciMc1[icam] = table_cal[icam][0].cMo * table_cal[0][0].cMo.inverse();
		}
		else
			ciMc1[icam].eye();

		systeme.setciMc1(icam+1, ciMc1[icam]);
#ifdef AFFDETAILSTXT
		if(icam > 0)
        {
			std::cout << "c" << icam << "Mc1 : " << std::endl << systeme.ciMc1[icam] << std::endl;
            //std::cout << std::endl << table_cal[icam][0].cMo * table_cal[0][0].cMo.inverse() << std::endl;
		}
#endif
		systeme.setCamera(icam, cam[icam]);
	}
	
	if(table_calStereo != NULL)
	{
		delete [] table_calStereo;
	}
	table_calStereo = new CCalibrationStereo[nbShot];
	for (unsigned int i=0 ; i < nbShot ; i++)
	{
		table_calStereo[i].init(nbCams);
		for(int icam = 0 ; icam < nbCams ; icam++)
        {
            if (icam == 0)
                table_calStereo[i].setCalibCam(icam, &(table_cal[icam][i]));
            else
                table_calStereo[i].setCalibCam(icam, &(table_cal[icam][match[icam][i]]));
            
            it_actExt = activeExtrinsicParameters.find(icam);
            if(it_actExt != activeExtrinsicParameters.end())
                table_calStereo[i].setActiveExtrinsicParameters(icam, it_actExt->second[0], it_actExt->second[1], it_actExt->second[2], it_actExt->second[3], it_actExt->second[4], it_actExt->second[5]);
        }
	}
	
	try
	{
		//CCalibrationStereo::setLambda(0.04);
		resCalibStats = CCalibrationStereo::fullCalibVVSMulti(nbShot, table_calStereo, systeme, false); //, mireUtilisee
	}
	catch(...)
	{
		calibDone = false;
		
		return -1;
	}
	
	calibDone = true;
	
	return 0;
}

/*
 Rempli une liste (STL) de 

/*
 Initialise la liste d'image pour une caméra ainsi que tous les autres 
 tableaux sur le nombre d'images
 retourne : 
 0 si tout OK
 -1 si le premier fichier n'est pas une image
 */
int CInterface::setListImages(std::vector<std::string> _nomsFichiers, int icam)
{
	vpImage<vpRGBa> Ic;
	try
	{
		vpImageIo::read(Ic, _nomsFichiers[0]);
	}
	catch(...)
	{
		std::cout << "error reading image : " << _nomsFichiers[0] << std::endl;
		return -1;
	}
	
	if(nomsFichiers == NULL)
	{
		nbShot = _nomsFichiers.size();
		
		nomsFichiers = new std::vector<std::string>[nbCams];
		tabImagesActives = new bool[nbShot];
		imHaut = new unsigned int[nbCams];
		imLarg = new unsigned int[nbCams];		
		listImagesInitCam = new bool[nbCams];
		
		for(int i = 0 ; i < nbShot ; i++)
			tabImagesActives[i] = true;
	}
	
	nomsFichiers[icam] = _nomsFichiers;
	
	imHaut[icam] = Ic.getHeight();
	imLarg[icam] = Ic.getWidth();
	
	listImagesInitCam[icam] = true;
	
	int i = -1;
	while ((++i<nbCams) && listImagesInitCam[i]);
	
	if(i == nbCams)
		listImagesInit = true;
	else
		listImagesInit = false;
	
	return 0;
}


int
CInterface::getMireReproj(int icam, int numImage, std::vector<SImagePoint> & ptsMire)
{
    return table_cal[icam][numImage].getPrimitivesReproj(ptsMire);
}

int CInterface::initDetection(int icam, int im)
{
    table_cal[icam][im].clearPoint();
    mirecour[icam][im] = mireinit;
    detectionInit = true;
    
    return 0;
}

int CInterface::detectionDUnDesPoints(int icam, int im, unsigned int curPt, s_SImagePoint & impo)
{
    //Bof ! eviter le chargement trop frequent des images...
    vpImage<unsigned char> I;
    try
	{
		vpImageIo::read(I, nomsFichiers[icam][im]);
	}
	catch(...)
	{
		return -4;
	}
    

    vpImagePoint ip;
    ip.set_uv(impo.u, impo.v);
    mirecour[icam][im].initTrackingUneDesPrimRef(I, cam[icam], curPt, ip);
    impo.u = ip.get_u();
    impo.v = ip.get_v();
    
    return 0;
}

int CInterface::ignorerUnDesPoints(int icam, int im, unsigned int curPt)
{
	mirecour[icam][im].supprimeUneDesPrimRef(curPt);

	return 0;
}

/*
 Reprojette les mires dans les images pour redétecter les points
 retourne :
 -1 si l'étalonnage n'est pas réalisé
 -4 si une image ne peut être chargée
 0 si tout OK
 */
int CInterface::recomputePoints()
{
	if(!calibDone)
		return -1;
	
	vpImage<unsigned char> I;
	
	for(int icam = 0 ; icam < nbCams ; icam++)
	{
		ensemblemires[icam].kill();
		mireUtilisee[icam].front();
	}
	
	switch(tm)
	{
		case MIRE_DOTS :
			//			((vpDot2 *)priminit)->setGraphics(true) ;
			break;
		case MIRE_CORNERS :
			((CCorner *)priminit)->setWin(5, 5);
			break;
		case MIRE_RINGS :
			//			((CRing *)priminit)->setGraphics(true) ;
			break;
		default :
			return -1;
			break;
	}
	
	mireinit.initParametresPrimitive(priminit);
	
	for(int im = 0 ; im < nbShot ; im++)
		if(tabImagesActives[im])
		{
			for(int icam = 0 ; icam < nbCams ; icam++)
			{
                table_cal[icam][im].clearPoint();
                if(mireUtilisee[icam].value())
                {
                    mirecour[icam][im] = mireinit;
                    
                    try
                    {
                        //std::cout << nomsFichiers[icam][numImage] << std::endl;
                        vpImageIo::read(I, nomsFichiers[icam][im]);
                    }
                    catch(...)
                    {
                        //		std::cout << "error image : " << e << std::endl;
                        return -4;
                    }
                    
                    mirecour[icam][im].initTrackingNoDisplay(I, cam[icam], table_cal[icam][im].cMo);
                    
                    ensemblemires[icam].addRight(mirecour[icam][im]);
                    //				mireUtilisee[icam].addRight(true);
                    
                    //we put the pose matrix in the current calibration structure
                    //table_cal[icam][im].cMo = cMoTmp ; 
                    
                    table_cal[icam][im].setListPoints(mirecour[icam][im].points);
                }
                mireUtilisee[icam].next();                
			}
		}
	
	switch(tm)
	{
		case MIRE_DOTS :
			//			((vpDot2 *)priminit)->setGraphics(true) ;
			break;
		case MIRE_CORNERS :
			((CCorner *)priminit)->setWin(winu, winv);
			break;
		case MIRE_RINGS :
			//			((CRing *)priminit)->setGraphics(true) ;
			break;
		default :
			return -1;
			break;
	}
	
	mireinit.initParametresPrimitive(priminit);
	
	return 0;
}

/*
 Génère une chaîne de résultat (paramètres estimés, erreurs de reproj...)
 retourne : 
 une chaîne vide si l'étalonnage n'est pas réalisé
 la chaîne de caractère susmentionnée
 */
std::string CInterface::getResult()
{	
	std::ostringstream os("");
	
	if(!calibDone)
		return os.str();
	
	os << "Number of iterations : " << resCalibStats.nbIter << std::endl;
	os << "Mean reprojection error : " << resCalibStats.moy << std::endl;
	os << "Std reprojection error : " << resCalibStats.std << std::endl << std::endl;
	
	for(int icam = 0 ; icam < nbCams ; icam++)
	{
		os << "camera " << icam << " : " << cam[icam]->name << std::endl;
		os << "K : " << std::endl;
		os << cam[icam]->getK() << std::endl;
		switch(cameras[icam])
		{
			case Paraboloid : os << "h : " << ((CParaboloid *)cam[icam])->geth() <<  std::endl;
				break;
			case Fisheye :
			case Omni : os << "xi : " << ((COmni *)cam[icam])->getXi() <<  std::endl;
				break;
			default:
				//std::cout << "initSysteme : Type de primitive inconnu" << std::endl;
				break;
		}
		if(cam[icam]->distorsions)
		{
			os << "with distorsions parameters for undistorted to distorted transformation :" << std::endl ;
			if(cam[icam]->activek[0])
				os << "  k1 = " << cam[icam]->getk1();
			if(cam[icam]->activek[1])
				os <<"\t k2 = "<< cam[icam]->getk2();
			if(cam[icam]->activek[2])
				os << "\t  k3 = " << cam[icam]->getk3();
			if(cam[icam]->activek[3])
				os <<"\t k4 = "<< cam[icam]->getk4();
			if(cam[icam]->activek[4])
				os << "\t  k5 = " << cam[icam]->getk5();
			os << std::endl;
            os << "with distorsions parameters for distorted to undistorted transformation :" << std::endl ;
			if(cam[icam]->activek[0])
				os << "  ik1 = " << cam[icam]->getik1();
			if(cam[icam]->activek[1])
				os <<"\t ik2 = "<< cam[icam]->getik2();
			if(cam[icam]->activek[2])
				os << "\t  ik3 = " << cam[icam]->getik3();
			if(cam[icam]->activek[3])
				os <<"\t ik4 = "<< cam[icam]->getik4();
			if(cam[icam]->activek[4])
				os << "\t  ik5 = " << cam[icam]->getik5();
			os << std::endl;
		}
		
		
		if(icam > 0)
		{
			os << "c" << icam+1 << "Mc1 : " << std::endl << systeme.ciMc1[icam] << std::endl;
			os << "r_1" << icam+1 << " : " << std::endl << vpPoseVector(systeme.ciMc1[icam]) << std::endl;
		}
		
		os << std::endl;
	}
	
	return os.str();
}

/*
 Charge un systeme stereo etalonne a partir d'un fichier au format XML
 retourne :
 -1 si le chargement n'a pu se faire
 0 si tout OK
 */
int CInterface::loadXML(std::string nomFichier)
{
    CModelStereoXml chargeSysteme(nomFichier);
    
    chargeSysteme >> nbCams;
    
    chargeSysteme >> cameras;
    
    initSysteme();
    chargeSysteme >> systeme;
    
    calibDone = true;
	
	return 0;
}

/*
 Enregistre les résultats d'étalonnage dans des fichiers au format XML
 retourne : 
 -1 si l'étalonnage n'est pas réalisé
 0 si tout OK
 */
int CInterface::saveXML(std::string nomFichier)
{
	if(!calibDone)
		return -1;
	
	CModelStereoXml sauvegardeSysteme(nomFichier);
    
  sauvegardeSysteme << systeme;


	/*for(int icam = 0 ; icam < nbcams ; icam++)
	 {
	 std::ostringstream nomfichierCam("");
	 // Sauvegarde de la caméra au format XML
	 nomfichierCam << "camera" << icam << ".xml";
	 COmniXml sauvegardeCamera(nomfichierCam.str());
	 sauvegardeCamera << *((COmni *)cam[icam]);
	 }*/
	
	return 0;
}

/*
 Permet de définir le statut des images chargée (utilisées pour le calibrage ou non)
 retourne : 
 0 si tout OK
 */
int CInterface::setListImagesActives(bool *_tabImagesActives)
{
	for(int i = 0 ; i < nbShot ; i++)
		tabImagesActives[i] = _tabImagesActives[i];
	
	return 0;
}

int CInterface::getListPosesMires(std::list<MatricePose *> &listPoses)
{
	if(!calibDone)
		return -1;
	
	listPoses.clear();
	for (unsigned int p = 0 ; p < nbShot ; p++)
	{
		MatricePose *MP = new MatricePose();
		
		for(int i = 0 ; i < 4 ; i++)
			for(int j = 0 ; j < 4 ; j++)
				(*MP)[i][j] = table_calStereo[p].calibcams[0]->cMo[i][j];

		listPoses.push_back(MP);
	}
	
	return 0;
}

int CInterface::getListPosesRelativesCameras(std::list<MatricePose *> &listPoses)
{
	if(!calibDone)
		return -1;
	
	listPoses.clear();
	for(int icam = 0 ; icam < systeme.nbcams ; icam++)
	{
		MatricePose *MP = new MatricePose();
		
		for(int i = 0 ; i < 4 ; i++)
			for(int j = 0 ; j < 4 ; j++)
				(*MP)[i][j] = systeme.ciMc1[icam][i][j];
		
		listPoses.push_back(MP);
	}
	
	return 0;
}

int CInterface::getListTypesCameras(std::list<ModelType> &listTypesCameras)
{
	if(!calibDone)
		return -1;
	
	listTypesCameras.clear();
	for(int i = 0; i < cameras.size() ; i++)
		listTypesCameras.push_back(cameras[i]);
	
	return 0;
}
