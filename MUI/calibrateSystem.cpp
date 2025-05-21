#include <MIXEDVISION/commun.h>
#include <MINTERFACE/CInterface.h>

#include <cmath>
#include <visp/vpConfig.h>
//#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iomanip>

#include <vector>

#define AFFRESULT

//#define VISU3D

#ifdef VISU3D
	#include <QApplication>
	#include <QDesktopWidget>
	#include "glwidget.h"
#endif

// List of allowed command line options
#define GETOPTARGS  "di:p:hf:g:n:s:l:comt:"

//#define PO
//#define PPO
//#define OO
//#define OOO
//#define FEP
//#define P
//#define P_alldist
//#define P_alldistx2

//#define O
//#define Fe
//#define FOO

#define dualF
// dualF implies multi-plane calibration target but we use three possible configurations
// One has 6 "planes" as two half-cubes side by side (see Caron et al. @ ICRA18)
//#define MULTIPLANE6
// Another has 2 planes parallel distant (see Duvinage et al. 2024) 
//#define MULTIPLANE2
// A third one has 5 planes as a single cube (roughly) except one face (see ?? 2025)
#define MULTIPLANE5


void usage( char *name, char *badparam, std::string ipath, std::string ppath,
		   double gray, unsigned first, unsigned nimages, unsigned step, double lambda);

bool getOptions(int argc, char **argv, std::string &ipath, std::string &ppath,
				double &gray, unsigned &first, unsigned &nimages, unsigned &step,
				double &lambda, bool &display, bool &click, bool &saveOnDisk, bool &manuelExtraction, unsigned &primtype);

int main(int argc, char **argv)
{
	std::cout << "toto" << std::endl;
	//---------------------------------------------------
	///////////////////////////////////////////////////////
	std::string env_ipath;
	std::string opt_ipath;
	std::string ipath;
	std::string opt_ppath;
	std::string dirname;
	std::string filename, filenameOut;
	std::string filename_out;

	double opt_gray = 0.9;
	unsigned opt_first = 1;
	unsigned opt_nimages = 4;
	unsigned opt_step = 1, opt_primtype=0;
	double opt_lambda = 0.5;
	bool opt_display = true, opt_save = false, opt_manualExtraction=true;
	bool opt_click = true;

	unsigned int winu, winv;
    
    int fact;

    int nbPtClick = 4; //default
    int ptIgnores = 0;
	
	winu = winv = 3;//5;//33;//3; //15;//24;//9;//13;//11;//33;//6;
	
	///////////////////////////////////////////
	//---------COMMAND LINE--------------------
	
	// Get the VISP_IMAGE_PATH environment variable value
	char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
	if (ptenv != NULL)
		env_ipath = ptenv;
	
	// Set the default input path
	if (! env_ipath.empty())
		ipath = env_ipath;
	
	// Read the command line options
	if (getOptions(argc, argv, opt_ipath, opt_ppath,opt_gray,opt_first, opt_nimages,
				   opt_step, opt_lambda, opt_display, opt_click, opt_save, opt_manualExtraction, opt_primtype) == false) {
		return (-1);
	}

	CInterface interf;
	MireType mt;
	ParamMire mp;
	int **pointsGrilleRef = new int*[4];
	for(int i = 0 ; i < 4 ; i++)
		pointsGrilleRef[i] = new int[3];
	
	std::cout << "opt_primtype: " << opt_primtype << std::endl;
	
	switch(opt_primtype)
	{
		case 0 :
			mt = MIRE_DOTS;
            /*
			pointsGrilleRef[0][0] = 1; pointsGrilleRef[0][1] = 1; pointsGrilleRef[0][2] = 0;
			pointsGrilleRef[1][0] = 1; pointsGrilleRef[1][1] = 4; pointsGrilleRef[1][2] = 0;
			pointsGrilleRef[2][0] = 3; pointsGrilleRef[2][1] = 4; pointsGrilleRef[2][2] = 0;
			pointsGrilleRef[3][0] = 4; pointsGrilleRef[3][1] = 1; pointsGrilleRef[3][2] = 0;
			
			mp = ParamMire(0.03, 0.03, 0.0, 6, 6, 1, (int **)pointsGrilleRef);
			*/
            //for dualF 5
			/*pointsGrilleRef[0][0] = 0; pointsGrilleRef[0][1] = 0; pointsGrilleRef[0][2] = 0;
			pointsGrilleRef[1][0] = 0; pointsGrilleRef[1][1] = 5; pointsGrilleRef[1][2] = 0;
			pointsGrilleRef[2][0] = 5; pointsGrilleRef[2][1] = 5; pointsGrilleRef[2][2] = 0;
			pointsGrilleRef[3][0] = 5; pointsGrilleRef[3][1] = 0; pointsGrilleRef[3][2] = 0;*/
            pointsGrilleRef[0][0] = 0; pointsGrilleRef[0][1] = 0; pointsGrilleRef[0][2] = 0;
			pointsGrilleRef[1][0] = 0; pointsGrilleRef[1][1] = 5; pointsGrilleRef[1][2] = 0;
			pointsGrilleRef[2][0] = 3; pointsGrilleRef[2][1] = 5; pointsGrilleRef[2][2] = 0;
			pointsGrilleRef[3][0] = 5; pointsGrilleRef[3][1] = 5; pointsGrilleRef[3][2] = 0;
			
			mp = ParamMire(0.03, 0.03, 0.0, 6, 6, 1, (int **)pointsGrilleRef);
			break;
		case 1 :
			mt = MIRE_CORNERS;
/*
			pointsGrilleRef[0][0] = 0; pointsGrilleRef[0][1] = 0; pointsGrilleRef[0][2] = 0;
			pointsGrilleRef[1][0] = 5; pointsGrilleRef[1][1] = 0; pointsGrilleRef[1][2] = 0;
			pointsGrilleRef[2][0] = 9; pointsGrilleRef[2][1] = 0; pointsGrilleRef[2][2] = 0;
			pointsGrilleRef[3][0] = 9; pointsGrilleRef[3][1] = 7; pointsGrilleRef[3][2] = 0;
			
			//mp = ParamMire(0.07, 0.07, 0.0, 5, 3, 1, (int **)pointsGrilleRef);
			//mp = ParamMire(0.117, 0.117, 0.0, 8, 6, 1, (int **)pointsGrilleRef);
            mp = ParamMire(0.025, 0.025, 0.0, 10, 8, 1, (int **)pointsGrilleRef);
*/			
            pointsGrilleRef[0][0] = 0; pointsGrilleRef[0][1] = 0; pointsGrilleRef[0][2] = 0;
			pointsGrilleRef[1][0] = 0; pointsGrilleRef[1][1] = 5; pointsGrilleRef[1][2] = 0;
			pointsGrilleRef[2][0] = 4; pointsGrilleRef[2][1] = 5; pointsGrilleRef[2][2] = 0;
			pointsGrilleRef[3][0] = 7; pointsGrilleRef[3][1] = 5; pointsGrilleRef[3][2] = 0;

            mp = ParamMire(0.03, 0.03, 0.0, 8, 6, 1, (int **)pointsGrilleRef);

			break;
		case 2 :
			mt = MIRE_RINGS;
			
			pointsGrilleRef[0][0] = 1; pointsGrilleRef[0][1] = 1; pointsGrilleRef[0][2] = 0;
			pointsGrilleRef[1][0] = 1; pointsGrilleRef[1][1] = 4; pointsGrilleRef[1][2] = 0;
			pointsGrilleRef[2][0] = 3; pointsGrilleRef[2][1] = 4; pointsGrilleRef[2][2] = 0;
			pointsGrilleRef[3][0] = 4; pointsGrilleRef[3][1] = 1; pointsGrilleRef[3][2] = 0;
			
			mp = ParamMire(0.142, 0.142, 0.0, 6, 6, 1, (int **)pointsGrilleRef);
			
			break;
		default :
			exit(-1);
			break;
	}


	interf.setTypeMire(mt);//, mp); // in case the calibration target is defined in the ParametresMire*.txt file in the directory containing CalibrateSystem
	//interf.setTypeMire(mt, mp); // in case the calibration target is fully defined in the c++ code
#ifdef dualF
    #ifdef MULTIPLANE2
    //first target is already there with setTypeMire and the origin of the calibration object coordinates system

    //second target is expressed with respect to the first target origin
    PoseVector mirePose[2];
    mirePose[1][0] = 0.0; // t_X
    mirePose[1][1] = 0.0; // t_Y
    mirePose[1][2] = 0.567;//0.6; // t_Z
    mirePose[1][3] = 0.0; // \theta w_X
    mirePose[1][4] = 0.0; // \theta w_Y
    mirePose[1][5] = 0.0; // \theta w_Z
    interf.addMire(mt, mp, mirePose[1]);

    nbPtClick *= 2; // assuming it was still 4 before

    #elif defined(MULTIPLANE5)
    //first target is already there with setTypeMire and the origin of the calibration object coordinates system
    unsigned nbPtClick_single = nbPtClick;
    //second target is expressed with respect to the first target origin
    PoseVector mirePose[5];
    mirePose[1][0] = 0.0; // t_X
    mirePose[1][1] = -0.03; // t_Y
    mirePose[1][2] = 0.03;//0.6; // t_Z
    mirePose[1][3] = M_PI_2; // \theta w_X
    mirePose[1][4] = 0.0; // \theta w_Y
    mirePose[1][5] = 0.0; // \theta w_Z
    interf.addMire(mt, mp, mirePose[1]); //to do: recup mp from file

    nbPtClick += nbPtClick_single; // assuming same number of points to click for each plane

    mirePose[2][0] = -0.03; // t_X
    mirePose[2][1] = 0.0; // t_Y
    mirePose[2][2] = 0.03;//0.6; // t_Z
    mirePose[2][3] = 0.0; // \theta w_X
    mirePose[2][4] = -M_PI_2; // \theta w_Y
    mirePose[2][5] = 0.0; // \theta w_Z
    interf.addMire(mt, mp, mirePose[2]);

    nbPtClick += nbPtClick_single; // assuming same number of points to click for each plane

    mirePose[3][0] = 0.0; // t_X
    mirePose[3][1] = 0.09; // for dots // 0.154; // for chess // // t_Y
    mirePose[3][2] = 0.31; //for dots //0.365; // for chess // // t_Z
    mirePose[3][3] = 0.0; // \theta w_X
    mirePose[3][4] = 0.0; // \theta w_Y
    mirePose[3][5] = 0.0; // \theta w_Z
    interf.addMire(mt, mp, mirePose[3]);

    nbPtClick += nbPtClick_single; // assuming same number of points to click for each plane

    mirePose[4][0] = 0.0; // t_X
    mirePose[4][1] = 0.27; //for dots // 0.334; // for chess // // t_Y
    mirePose[4][2] = 0.28; // for dots // 0.335; // for chess // // t_Z
    mirePose[4][3] = -M_PI_2; // \theta w_X
    mirePose[4][4] = 0.0; // \theta w_Y
    mirePose[4][5] = 0.0; // \theta w_Z
    interf.addMire(mt, mp, mirePose[4]);

    nbPtClick += nbPtClick_single; // assuming same number of points to click for each plane

    #elif defined(MULTIPLANE6)

    #endif
#endif


	interf.unsetFOO();
	interf.setGrayLevelPrecision(opt_gray);	

#ifdef O
	interf.addCamera(Omni);
    interf.setActiveDistorsionParameters(0, false, false, false, false, false);
	
	std::vector<std::string> nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << iter<< ".png" ;//".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
	
	//interf.setInitBaseIntrinsicParameters(0, 250, 250, 512, 384);
	
#endif
#ifdef Fe
	interf.addCamera(Fisheye);
    interf.setActiveDistorsionParameters(0, false, false, false, false, false);
	
	std::vector<std::string> nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << iter<< ".png" ;//".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
	
	//interf.setInitBaseIntrinsicParameters(0, 250, 250, 512, 384);
	
#endif
#ifdef PO
	interf.addCamera(Persp);
	interf.addCamera(Omni);
	
	std::vector< std::string > nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	int icam = 0;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
	
	icam = 1;
	nomsFichiers.clear();
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 1);
	
#endif
#ifdef PPO
	interf.addCamera(Persp);
	interf.addCamera(Persp);
	interf.addCamera(Omni);
	
	std::vector< std::string > nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	int icam = 0;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
	
	icam = 1;
	nomsFichiers.clear();
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 1);

	icam = 2;
	nomsFichiers.clear();
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 3);
	
#endif
#ifdef OO
	interf.addCamera(Omni);
	interf.addCamera(Omni);
	
	std::vector< std::string > nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	int icam = 0;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
	
	icam = 1;
	nomsFichiers.clear();
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 1);
	
#endif
#ifdef OOO
	interf.addCamera(Omni);
	interf.addCamera(Omni);
	interf.addCamera(Omni);
	
	std::vector< std::string > nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	int icam = 0;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
	
	icam = 1;
	nomsFichiers.clear();
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 1);

		icam = 2;
	nomsFichiers.clear();
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 2);
#endif
#ifdef FEP
	interf.addCamera(Fisheye);
	interf.addCamera(Persp);
    //interf.setActiveExtrinsicParameters(1, true, true, true, true, true, true);
    //interf.setInitExtrinsicParameters(1, 0, 0, 0, 0, 0, 0);
    
    //interf.setActiveDistorsionParameters(0, false, false, false, true, false);
    interf.setActiveDistorsionParameters(1, true, false, false, false, false);
	
	std::vector< std::string > nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	int icam = 0;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
	
	icam = 1;
	nomsFichiers.clear();
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 1);
	
#endif
#ifdef dualF
    //add two Barreto-fisheye cameras 
    //interf.addCamera(Fisheye);
	//interf.addCamera(Fisheye);
    interf.addCamera(Omni);
    interf.addCamera(Omni);

    //set relative rotation only to force a common single viewpoint
    //interf.setActiveExtrinsicParameters(1, false, false, false, true, true, true);
    //interf.setActiveExtrinsicParameters(1, false, false, false, false, true, false);
    //interf.setActiveExtrinsicParameters(1, false, false, true, false, true, false);
    //interf.setInitExtrinsicParameters(1, 0, 0, 0, 0, -M_PI, 0);
    
    //In case the two fisheyes are in the same image
    //when images are 1280x640
    //interf.setInitBaseIntrinsicParameters(0, 360, 360, 960, 320);
    //interf.setInitBaseIntrinsicParameters(1, 360, 360, 320, 320);
    //when images are 1024x512
    //interf.setInitBaseIntrinsicParameters(0, 256, 256, 768, 256);
    //interf.setInitBaseIntrinsicParameters(1, 256, 256, 256, 256);
    //when images are 1440x720
    //interf.setInitBaseIntrinsicParameters(0, 360, 360, 1080, 360);
    //interf.setInitBaseIntrinsicParameters(1, 360, 360, 360, 360);

    //interf.setActiveDistorsionParameters(0, false, false, false, true, true);
    //interf.setActiveDistorsionParameters(1, false, false, false, true, true);

	std::vector< std::string > nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	int icam = 0;
    //set image filenames for the first camera
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
	
	icam = 1;
	nomsFichiers.clear();
    //set image filenames for the second camera
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 1);
#endif
#ifdef P
	interf.addCamera(Persp);
//    interf.setActiveDistorsionParameters(0, true, false, false, false, false);
//	interf.setInitBaseIntrinsicParameters(0, 500, 500, 540, 400);
	
	std::vector<std::string> nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
#endif

#ifdef P_alldist
	interf.addCamera(Persp);
  interf.setActiveDistorsionParameters(0, true, true, true, true, true);
	
	std::vector<std::string> nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
#endif

#ifdef P_alldistx2
	interf.addCamera(Persp);
  interf.setActiveDistorsionParameters(0, true, true, true, true, true);
	interf.addCamera(Persp);
	interf.setActiveDistorsionParameters(1, true, true, true, true, true);
	
	std::vector<std::string> nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	int icam = 0;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
	
	icam = 1;
	nomsFichiers.clear();
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 1);
#endif
	
#ifdef FOO
	
	interf.addCamera(Omni);
	interf.addCamera(Omni);
	interf.addCamera(Omni);
	interf.addCamera(Omni);
	
	interf.setFOO();
	
	std::vector<std::string> nomsFichiers;
	std::vector< std::vector<std::string> > nomsFichiersToutesCameras;
	std::ostringstream s;
	int icam = 0;
	for (int iter = opt_first; iter < (opt_first+opt_nimages); iter++)
	{
		s.str("");
		s << "grid36-" << std::setw(2) << std::setfill('0') << iter<< ".pgm";
		filename = opt_ppath + s.str();
		std::cout<<filename<<std::endl;
		nomsFichiers.push_back(filename);
	}
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 0);
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 1);
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 2);
    nomsFichiersToutesCameras.push_back(nomsFichiers);
	interf.setListImages(nomsFichiers, 3);
	
	interf.setGrayLevelPrecision(0.97);	
#endif

	std::cout << "demarrage calibrage" << std::endl;

	interf.setWinSize(winu, winv);

	double tempsCalibSeq = vpTime::measureTimeMs();
    
    //calibSequentielle
    if(interf.initCalib() != 0)
        return -1;

	int *imFirst = new int[interf.nbCams];
	for(int icam = 0 ; icam < interf.nbCams ; icam++)
		imFirst[icam] = 0;
    
	if(!opt_click)
		interf.loadPoints(imFirst);
	
	for(int im = 0 ; im < interf.nbShot ; im++)
		if(interf.tabImagesActives[im])
		{
			for(int icam = 0 ; icam < interf.nbCams ; icam++)
			{
				if(im >= imFirst[icam])
				{
					int ret = 0;
                    //ClickImage
                    vpImage<unsigned char> I;
                    int retour;
                    
                    try
                    {
                        //std::cout << nomsFichiers[icam][numImage] << std::endl;
                        vpImageIo::read(I, nomsFichiersToutesCameras[icam][im]);
                    }
                    catch (const vpException & e)
                    {
                        std::cerr << "error image with exception: " << e << std::endl;
                        return -4;
                    }
                    
                    fact = (int)(1+vpMath::maximum(I.getWidth(), I.getHeight())*0.001);
                    
                    //interf.initTable_cal(icam, im);
                    
                    bool detectionEnCours = true;
                    while(detectionEnCours)
                    {
                        vpMouseButton::vpMouseButtonType button;
                        
                        interf.initDetection(icam, im);
                        


#if defined(VISP_HAVE_X11)
                        vpDisplayX display;
#elif defined(VISP_HAVE_OPENCV)
                        vpDisplayOpenCV display;
#else
                        std::cout << "Sorry, no video device is available" << std::endl;
                        return -1;
#endif
        
                        // Display the image
                        try
                        {
                            // Display size is automatically defined by the image (I) size
                            display.init(I, 10, 50, "Calibration initialization") ;
                            ///////////////////////////////////////////////////////////////
                            // temporaire pour gérer les images plus grandes que l'écran
                            int largIm, hautIm;
                            if(I.getWidth() > I.getHeight())
                            {
                                largIm = 1024;
                                hautIm = I.getHeight()*largIm/I.getWidth();
                            }
                            else
                            {
                                hautIm = 800;
                                largIm = I.getWidth()*hautIm/I.getHeight();
                            }

                            //cv::resizeWindow("Calibration initialization", largIm, hautIm);

                            ///////////////////////////////////////////////////////////////
                            //display.init(I, 10, 50, 1024, 768, "Calibration initialization") ; //OpenCV seulement
                            // Display the image
                            vpDisplay::display(I) ;
                            vpDisplay::flush(I) ;
                        }
                        catch (const vpException &e)
                        {
                            std::cerr << "Error while displaying the image with exception: " << e << std::endl;
                            return -2;
                        }
                        
                        //Affichage click gauche pour passer directement
                        vpDisplay::displayCharString(I,10*(float)I.getWidth()/640.0f,10,"Select reference points with left click.",
                                                     vpColor(0, 255, 0));

                        vpDisplay::displayCharString(I,22*(float)I.getWidth()/640.0f,10,
                                                         //"A right click to directly jump to the next image.",
                                                         "A right click to ignore a point.",
                                                         vpColor(0, 255, 0));
                        vpDisplay::flush(I);
                        
                        /////////////////////////////////////////////////////
                        //boucler sur les 4+ points init a detecter
                        vpImagePoint ip;
                        s_SImagePoint impo;
												vpMouseButton::vpMouseButtonType btn;
                        //for(unsigned int curPt = 0; curPt < 4 ; curPt++)
                        unsigned int curPt = 0;
                        ptIgnores = 0;
                        interf.mireinit.pointsref.front();
                        while(curPt < (nbPtClick-ptIgnores))
                        {
                            std::cout << "please click on point of indexes -> X: " << interf.mireinit.pointsref.value().get_oX() << " Y: " << interf.mireinit.pointsref.value().get_oY() << " Z: " << interf.mireinit.pointsref.value().get_oZ() << std::endl; 
                            vpDisplay::getClick(I,ip, btn);
                            if(btn != vpMouseButton::button1) //if not left click
                            {
                                if(btn == vpMouseButton::button3) //if right click
                                {
                                    retour = interf.ignorerUnDesPoints(icam, im, curPt);
                                    std::cout << "point " << curPt+ptIgnores << " ignored" << std::endl;
                                    ptIgnores++;
                                }
                                else
                                {
                                    //otherwise (middle click), jump to the next image
                                    retour = 12;
                                    break;
                                }
                            }
                            else
                            {
                                impo.u = ip.get_u();
                                impo.v = ip.get_v();
                                retour = interf.detectionDUnDesPoints(icam, im, curPt, impo);
                                if(retour)
                                    break;
                                
                                                            ip.set_uv(impo.u, impo.v);
                                vpDisplay::displayCross(I, ip, 10, vpColor(255, 0, 0), fact);
                                vpDisplay::flush(I);

                                curPt++;
                            }
                            interf.mireinit.pointsref.next();
                        }
                        if(retour)
                            break;
                        /////////////////////////////////////////////////////
                        
                        /////////////////////////////////////////////////////
                        //debut preCalib
                        retour = interf.preCalib(icam, im);
                        /////////////////////////////////////////////////////
                        
                        if(retour == -1)
                        {
                            vpDisplay::display(I);
                            vpDisplay::displayCharString(I,10*(float)I.getWidth()/640.0f,10,"Pose computation failed",
                                                         vpColor(255, 0, 0));
                            vpDisplay::displayCharString(I,22*(float)I.getWidth()/640.0f,10,
                                                         "A left click to define other dots.",
                                                         vpColor(0, 255, 0));
                            vpDisplay::displayCharString(I,34*(float)I.getWidth()/640.0f,10,
                                                         "A middle click to don't care of this pose.",
                                                         vpColor(0, 255, 0));
                            vpDisplay::flush(I) ;
                            std::cout << "\nPose computation failed." << std::endl;
                            std::cout << "A left click to define other dots." << std::endl;
                            std::cout << "A middle click to don't care of this pose." << std::endl;
                            vpImagePoint ip;
                            vpDisplay::getClick(I,ip,button) ;
                            switch(button)
                            {
                                case 1 :
                                    std::cout << "Left click has been pressed." << std::endl;
                                    std::cout << "2 initSystem" << std::endl;
                                    
                                    continue;
                                    /*case 3 :
                                     std::cout << "Right click has been pressed." << std::endl;
                                     continue;*/
                                case 2 :
                                    std::cout << "Middle click has been pressed." << std::endl;
                                    std::cout << "2 initSystem" << std::endl;
                                    
                                    ret = 1;
                                    break;
                                default:
                                    std::cout << "cas non gere." << std::endl;
                                    break;
                            }
                            if(ret != 0)
                                break;
                        }
                        else
                        {
                            // display the computed pose
                            vpDisplay::display(I) ;
                            
                            std::vector<SImagePoint> ptsMire, ptsRepere;
                            interf.getMireEtPosePreCalib(icam, im, ptsMire, ptsRepere);

                            //Affichage des points de la mire
                            std::vector<SImagePoint>::iterator it_ptsMire = ptsMire.begin();

                            for (unsigned int iPtsMire = 0; iPtsMire < ptsMire.size(); iPtsMire++, it_ptsMire++)
                                vpDisplay::displayCross(I, it_ptsMire->v, it_ptsMire->u, 10, vpColor(255, 0, 0), fact);
                            
                            //Affichage des "fleches" du repere dans l'image
                            vpDisplay::displayArrow ( I,
                                                     vpMath::round ( ptsRepere[0].v ), vpMath::round ( ptsRepere[0].u ),
                                                     vpMath::round ( ptsRepere[1].v ), vpMath::round ( ptsRepere[1].u ),
                                                     vpColor(255, 0, 0), 4*fact, 2*fact, 1*fact) ;
                            vpDisplay::displayArrow ( I,
                                                     vpMath::round ( ptsRepere[0].v ), vpMath::round ( ptsRepere[0].u ),
                                                     vpMath::round ( ptsRepere[2].v ), vpMath::round ( ptsRepere[2].u ),
                                                     vpColor(0, 255, 0), 4*fact, 2*fact, 1*fact ) ;
                            vpDisplay::displayArrow ( I,
                                                     vpMath::round ( ptsRepere[0].v ), vpMath::round ( ptsRepere[0].u ),
                                                     vpMath::round ( ptsRepere[3].v ), vpMath::round ( ptsRepere[3].u ),
                                                     vpColor(0, 0, 255), 4*fact, 2*fact, 1*fact ) ;

                            
                            vpDisplay::flush(I) ;
                            
                            vpDisplay::displayCharString(I,10*(float)I.getWidth()/640.0f,10,
                                                         "A left click to display grid.",
                                                         vpColor(0, 255, 0));
                            vpDisplay::displayCharString(I,22*(float)I.getWidth()/640.0f,10,
                                                         "A right click to define other dots.",
                                                         vpColor(0, 255, 0));
                            vpDisplay::flush(I) ;
                            std::cout << "\nA a left click to display grid." << std::endl;
                            std::cout << "A right click to define other dots." << std::endl;
                            vpImagePoint ip;
                            vpDisplay::getClick(I,ip,button) ;
                            ret = 0;
                            switch(button)
                            {
                                case 1 :
                                    std::cout << "Left click has been pressed." << std::endl;
                                    break;
                                    /*case 2 :
                                     std::cout << "Middle click has been pressed." << std::endl;
                                     continue;*/
                                case 3 :
                                    std::cout << "Right click has been pressed." << std::endl;
                                    std::cout << "2 initSystem" << std::endl;
                                    
                                    ret = 2;
                                    break;
                                    //continue;
                            }
                            
                            if(ret != 0)
                                continue;
                            
                            vpDisplay::display(I) ;
                            vpDisplay::flush(I) ;
                        }
                        
                        /////////////////////////////////////////////////////
                        //fin preCalib
                        
                        /////////////////////////////////////////////////////
                        //debut detectionGrille
                        /////////////////////////////////////////////////////
                        
                        interf.detectionTousPoints(icam, im);
                        
                        std::vector<SImagePoint> ptsMire;
                        interf.getMireDetectee(icam, im, ptsMire);
                        
                        //Affichage des points de la mire
                        std::vector<SImagePoint>::iterator it_ptsMire = ptsMire.begin();
                        
                        for (unsigned int iPtsMire = 0; iPtsMire < ptsMire.size(); iPtsMire++, it_ptsMire++)
                            vpDisplay::displayCross(I, it_ptsMire->v, it_ptsMire->u, 10, vpColor(255, 255, 0), fact);
                        
                        
                        vpDisplay::displayCharString(I,10*(float)I.getWidth()/640.0f,10,"A left click to validate this pose.",
                                                     vpColor(0, 255, 0));
                        vpDisplay::displayCharString(I,22*(float)I.getWidth()/640.0f,10,
                                                     "A right click to retry.",
                                                     vpColor(0, 255, 0));
                        vpDisplay::displayCharString(I,34*(float)I.getWidth()/640.0f,10,
                                                     "A middle click to don't care of this pose.",
                                                     vpColor(0, 255, 0));
                        vpDisplay::flush(I) ;
                        
                        std::cout << "\nA left click to validate this pose." << std::endl;
                        std::cout << "A right click to retry." << std::endl;
                        std::cout << "A middle click to don't care of this pose." << std::endl;
                        
                        vpDisplay::getClick(I,ip,button) ;
                        ret = 0;
                        switch(button){
                            case 1 : //left
                                std::cout << "\nLeft click has been pressed." << std::endl;
                                break;
                            case 2 : //middle
                                std::cout << "Middle click has been pressed." << std::endl;
                                std::cout << "2 initSystem" << std::endl;
                                
                                ret = 1;
                                break;
                                
                            case 3 : //right
                                std::cout << "Right click has been pressed." << std::endl;
                                std::cout << "2 initSystem" << std::endl;
                                
                                ret = 2;
                                break;
                                //continue;
                        }
                        if(ret != 0)
                        {
                            if(ret == 2)
                            {
                                continue;
                            }
                            else
                            {
                                break;
                            }
                        }
                        
                        interf.postCalib(icam, im);
                        
                        detectionEnCours = false;
                        
                        vpDisplay::close(I);
                    }
                    /////////////////////////////////////////////////////
                    //fin detectionEnCours
                    /////////////////////////////////////////////////////
                    
                    interf.finalizePostCalib(icam, im, retour);
                    
                    /////////////////////////////////////////////////////
                    //fin clickImage
                    /////////////////////////////////////////////////////
                    
					if(ret != 0)
					{
						interf.tabImagesActives[im] = false;
						break;
					}
                    /////////////////////////////////////////////////////
                    //fin detectionGrille
                    /////////////////////////////////////////////////////
				}
				
			}
			interf.systemeInit = false;
    }
    
	interf.updateCalib();
	
#ifdef AFFRESULT
	std::cout << " ======> temps calib sequentielle : " << vpTime::measureTimeMs()-tempsCalibSeq << std::endl;
	//PaO (2 caméras, 6 paires d'images, 432pts): 6148ms en GN ; 4005ms en LM ==> -2 secondes (réduction de 35%)
	//FOO (4 caméras, 8 images, 3072pts) : 158654ms (79 it stéréo) en GN; 23001ms en LM (88 it stéréo) ==> -135 sec (réduction de 85%)
	
	std::cout << "Resultats : " << std::endl;
	std::cout << interf.getResult() << std::endl;
#endif
	
	interf.recomputePoints();
	interf.updateCalib();

#ifdef AFFRESULT
	std::cout << "Resultats apres recomputePoints : " << std::endl;
	std::cout << interf.getResult() << std::endl;
#endif

	interf.recomputePoints();
	interf.updateCalib();

#ifdef AFFRESULT
	std::cout << "Resultats apres recomputePoints : " << std::endl;
	std::cout << interf.getResult() << std::endl;
#endif

	interf.recomputePoints();
	interf.updateCalib();

#ifdef AFFRESULT
	std::cout << "Resultats apres recomputePoints : " << std::endl;
	std::cout << interf.getResult() << std::endl;
#endif
     
  if(interf.saveXML("calib.xml"))
		std::cout << "echec de la sauvegarde XML" << std::endl;

	if(opt_save) //enregistrement en couleur et en png
    {
        for(int im = 0 ; im < interf.nbShot ; im++)
        {
            if(interf.tabImagesActives[im])
            {
                for(int icam = 0 ; icam < interf.nbCams ; icam++)
                {
                    vpImage<unsigned char> I;
                    vpImage<vpRGBa> IOut;
                    
                    try
                    {
                        //std::cout << nomsFichiers[icam][numImage] << std::endl;
                        vpImageIo::read(I, nomsFichiersToutesCameras[icam][im]);
                    }
                    catch(...)
                    {
                        //		std::cout << "error image : " << e << std::endl;
                        return -4;
                    }
                    
                    fact = (int)(1+vpMath::maximum(I.getWidth(), I.getHeight())*0.001);
                    
#if defined(VISP_HAVE_X11)
                        vpDisplayX d;
#elif defined(VISP_HAVE_OPENCV)
                        vpDisplayOpenCV d;
#else
                        std::cout << "Sorry, no video device is available" << std::endl;
                        return -1;
#endif
                    
                    d.init(I, 10, 50+10*im, "reprojections");
                    ///////////////////////////////////////////////////////////////
                    // temporaire pour gérer les images plus grandes que l'écran
                    int largIm, hautIm;
                    if(I.getWidth() > I.getHeight())
                    {
                        largIm = 1024;
                        hautIm = I.getHeight()*largIm/I.getWidth();
                    }
                    else
                    {
                        hautIm = 800;
                        largIm = I.getWidth()*hautIm/I.getHeight();
                    }
                    //cv::resizeWindow("reprojections", largIm, hautIm);
                    ///////////////////////////////////////////////////////////////
                    vpDisplay::display(I);

                    //Display the data of the calibration (center of the dots)
                    std::vector<SImagePoint> ptsMire;
                    interf.getMireDetectee(icam, im, ptsMire);
                    //Affichage des points de la mire
                    std::vector<SImagePoint>::iterator it_ptsMire = ptsMire.begin();
                    for (unsigned int iPtsMire = 0; iPtsMire < ptsMire.size(); iPtsMire++, it_ptsMire++)
                    {
                        vpDisplay::displayCircle(I, it_ptsMire->v, it_ptsMire->u, 5, vpColor(255, 255, 0), fact);
                    }                   

                    //Display grid : estimated center of dots using camera parameters
                    //std::cout << "getMireReproj..." << std::endl;
                    interf.getMireReproj(icam, im, ptsMire);
                    //std::cout << " ...done" << std::endl;
                    //Affichage des points de la mire
                    it_ptsMire = ptsMire.begin();
                    for (unsigned int iPtsMire = 0; iPtsMire < ptsMire.size(); iPtsMire++, it_ptsMire++)
                    {
                        vpDisplay::displayCross(I, it_ptsMire->v, it_ptsMire->u, 10, vpColor(0, 255, 0), fact);
                    }
                    
                    vpDisplay::flush(I) ;
                    
                    std::ostringstream filenameOut("");
                    
                    filenameOut << opt_ppath << "reproj-" << std::setw(2) << std::setfill('0') << icam+1 << "-" << std::setw(2) << std::setfill('0') << im << ".ppm";
                    
                    vpDisplay::getImage(I, IOut);
                    vpImageIo::write(IOut,filenameOut.str());
                    
                    vpDisplay::close(I);
                }
            }
        }
    }

    std::list<MatricePose *> listPosesMires;
    std::list<MatricePose *>::iterator it_listPosesMires;
   	interf.getListPosesMires(listPosesMires);
    int numMire = 0;
    for(it_listPosesMires = listPosesMires.begin() ; it_listPosesMires != listPosesMires.end() ; it_listPosesMires++, numMire++)
	{
        std::cout << "Pose mire " << numMire << std::endl;
		for(int i = 0 ; i < 4 ; i++)
        {
			for(int j = 0 ; j < 4 ; j++)
				std::cout << (**it_listPosesMires)[i][j] << " ";
            std::cout << std::endl;
        }
	}
	
    
#ifdef VISU3D
	
	//Récupération/initialisation des listes de pose de mire dans le repère stereo, 
	//de pose relatives caméra j vers caméra 1 et de types de caméras
	std::list<MatricePose *> listPosesRelativesCameras;
	std::list<MatricePose *>::iterator it_Poses;
	std::list<ModelType> listCameraTypes;
	

	interf.getListPosesRelativesCameras(listPosesRelativesCameras);
	interf.getListTypesCameras(listCameraTypes);
	
	//Affichage 3D des poses de la mire et du système de vision
	QApplication a( argc, argv );
	
	GLWidget *w = new GLWidget();
	//a.setMainWidget( w );
	
	w->setParametresMire(mp, mt); //ParamMire, MireType

	w->setListPosesMires(listPosesMires);
	for(it_Poses=listPosesMires.begin() ; it_Poses != listPosesMires.end() ; it_Poses++)
	{
		MatricePose *mpose = *it_Poses;
		delete mpose;
	}
	w->setListPosesRelativesCameras(listPosesRelativesCameras);
	for(it_Poses=listPosesRelativesCameras.begin() ; it_Poses != listPosesRelativesCameras.end() ; it_Poses++)
	{
		MatricePose *mpose = *it_Poses;
		delete mpose;
	}
	w->setListTypesOfCameras(listCameraTypes);
	
	w->show();
	
	return a.exec();
#endif	
	
	return 0;
}


/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param gray : gray level precision.
  \param first : First image.
  \param nimages : Number of images to display.
  \param step : Step between two images.
  \param lambda : Gain of the virtual visual servoing.
  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be usefull for automatic tests using crontab
  under Unix or using the task manager under Windows.

  \param click : Set as false, disable the mouse click.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, std::string &ipath, std::string &ppath,
    double &gray, unsigned &first, unsigned &nimages, unsigned &step,
    double &lambda, bool &display, bool &click, bool &saveOnDisk, bool &manuelExtraction, unsigned &primtype)
{
  const char *optarg;
  int c;
  while ((c = vpParseArgv::parse(argc, (const char **)argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd': display = false; break;
    case 'i': ipath = optarg; break;
    case 'p': ppath = optarg; break;
    case 'g': gray = atof(optarg);break;
    case 'f': first = (unsigned) atoi(optarg); break;
    case 'n': nimages = (unsigned) atoi(optarg); break;
    case 's': step = (unsigned) atoi(optarg); break;
    case 'l': lambda = atof(optarg); break;
    case 'c': click = false; break;
    case 'o': saveOnDisk = true; break;
    case 'm': manuelExtraction = false; break;
    case 't': primtype = (unsigned) atoi(optarg); break;
    case 'h': usage(argv[0], NULL, ipath, ppath,gray, first, nimages, step, lambda);
      return false; break;

    default:
      usage(argv[0], (char *)optarg, ipath, ppath, gray,first, nimages, step, lambda);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, ppath,gray, first, nimages, step, lambda);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param gray : Gray level precision.
  \param first : First image.
  \param nimages : Number of images to manipulate.
  \param step : Step between two images.
  \param lambda : Gain of the virtual visual servoing.

 */
void usage( char *name, char *badparam, std::string ipath, std::string ppath,
      double gray, unsigned first, unsigned nimages, unsigned step, double lambda)
{
  fprintf(stdout, "\n\
  Read images of a calibration grid from the disk and \n\
  calibrate the camera used for grabbing it.\n\
  Each image corresponds to a PGM file.\n\
  The calibration grid used here is available in : \n\
      ViSP-images/calibration/grid2d.{fig,pdf} or \n\
      ./example/calibration/grid2d.fig\n\
  This is a 6*6 dots calibration grid where dots centers \n\
  are spaced by 0.03 meter. You can obviously use another \n\
  calibration grid changing its parameters in the program.\n\
  Then you have to grab some images of this grid (you can use \n\
  grab examples of ViSP to do it), save them as PGM files and\n\
  precise their names with the -p option.\n\
\n\
SYNOPSIS\n\
  %s [-i <test image path>] [-p <personal image path>]\n\
     [-g <gray level precision>] [-f <first image>] \n\
     [-n <number of images>] [-s <step>] [-l lambda] \n\
     [-c] [-d] [-h]\n\
 ", name);

 fprintf(stdout, "\n\
 OPTIONS:                                               Default\n\
  -i <test image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/calibration/grid36-%%02d.pgm\"\n\
     images and the calibration grid data. \n\
     These images come from ViSP-images-x.y.z.tar.gz\n\
     available on the ViSP website.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
 \n\
  -p <personal image path>                             %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     By image sequence, we mean one file per image.\n\
     The following image file formats PNM (PGM P5, PPM P6)\n\
     are supported. The format is selected by analysing \n\
     the filename extension.\n\
     Example : \"/Temp/ViSP-images/calibration/grid36-%%02d.pgm\"\n\
     %%02d is for the image numbering.\n\
 \n\
  -g <gray level precision>                             %f\n\
     Specify a gray level precision to detect dots.\n\
     A number between 0 and 1.\n\
     precision of the gray level of the dot. \n\
     It is a double precision float witch \n\
     value is in ]0,1]. 1 means full precision, \n\
     whereas values close to 0 show a very bad \n\
     precision.\n\
 \n\
  -f <first image>                                     %u\n\
     First image number of the sequence.\n\
 \n\
  -n <number of images>                                %u\n\
     Number of images used to compute calibration.\n\
 \n\
  -s <step>                                            %u\n\
     Step between two images.\n\
 \n\
  -l <lambda>                                          %f\n\
     Gain of the virtual visual servoing.\n\
 \n\
  -d                                             \n\
     Disable the image display. This can be usefull \n\
     for automatic tests using crontab under Unix or \n\
     using the task manager under Windows.\n\
 \n\
  -c\n\
     Disable the mouse click.\n\
     If the image display is disabled (using -d)\n\
     this option is without effect.\n\
\n\
  -h\n\
     Print the help.\n\n",
    ipath.c_str(),ppath.c_str(), gray ,first, nimages, step,lambda);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

