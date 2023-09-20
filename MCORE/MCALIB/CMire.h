/**************************************************************************
*
* Copyright (C) 2009 M.I.S. All rights reserved.
*
* This software was developed at:
* Laboratoire M.I.S.
* 7, rue du moulin neuf
* 80000 Amiens
* http://mis.u-picardie.fr
*
*
* Authors:
* ViSP Lagadic http://www.irisa.fr/lagadic
* Guillaume Caron http://mis.u-picardie.fr/~g-caron/
* Damien Eynard http://mis.u-picardie.fr/~eynard/
*
*****************************************************************************/

/*
Développement débuté le 27 avril 2009
Guillaume Caron

TODO : init des parametres de tracking/detection de chaque type de primitive

*/

#ifndef CMire_hh
#define CMire_hh

#include <iostream>
#include <fstream>

#include <visp/vpConfig.h>
#include <visp/vpList.h>
#include <visp/vpTracker.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpColor.h>

#include <MIXEDVISION/CPoint.h>
#include <MIXEDVISION/CModel.h>

#include <MIXEDVISION/commun.h>

class
#if defined WIN32 || WIN64
	__declspec(dllexport)
#endif
CMire
{
public:
	vpList<CPoint> points;
	vpList<CPoint> pointsref;
	
	double dx,dy,dz; //distance (mm) between two points on the pattern
	unsigned int nPX,nPY,nPZ;

	vpList<vpTracker *> primitives;
	vpTracker *priminitparam; //!< primitive avec parametres aux bonnes valeurs pour init
	double size;

	vpHomogeneousMatrix cMo; //!< Pose de la mire dans le repere camera

	unsigned int nbpt; //!< Nombre de points/primitives trouvees de la mire
	unsigned int nptpose; //!< number of init dots by image

	MireType tprim; //!< type de primitives utilise pour cette mire

	CMire();
	~CMire();
	
	void clearPrimitives();

	//!< chargement des parametres de la mire a partir d'un fichier
	void initMire(std::string nomfichiermire);
	void initMire(MireType & tm, ParamMire & pm);
	int addPointsMire(MireType & tm, ParamMire & pm, PoseVector &pv );

	void operator=(CMire& _CMire);
/*
	//A ajouter plus tard pour suivi de mire (comme celle des quatres dots par exemple)
	void track(vpImage<unsigned char> &I); //Track
*/
    void getPrimRefs(std::vector<SImagePoint> & ptsMire);
    void getPrimitives(std::vector<SImagePoint> & ptsMire);

	//!< Init de priminitparam. A appeler imperativement avant toute chose
	void initParametresPrimitive(vpTracker *_priminit);
	//!< Init de la primitive passee en paramètre par priminitparam
	void setParametresPrimitive(vpTracker *primAinit);
	
	//!<  Init du tracking des autres prim
    void initTrackingNoDisplay(vpImage<unsigned char>& I, CModel *cam, vpHomogeneousMatrix & cMo, unsigned int _size = 0);
	//!< Init du tracking de chaque primref au clic
    int initTrackingUneDesPrimRef(vpImage<unsigned char>& I, CModel *cam, unsigned int curPt, vpImagePoint & ip,  unsigned int _size = 0);
		int supprimeUneDesPrimRef(unsigned int curPt);

	//!< Init du tracking de chaque primitive par fichier
	void initTracking(std::ifstream & fichierpoints);
	//!< Init du tracking des mires par fichier
	static int initTrackingMires(std::string nomfichierpoints, 
					vpList<CMire> & ensemblemires, vpList<vpHomogeneousMatrix> & lcMo);
	static void save(std::string filename, CMire & ensemblemires, vpHomogeneousMatrix & cMo, int nbElements = -1);
	static void	saveALL(std::string filename, vpList<CMire> & ensemblemires);
	
	/*inline*/ MireType getPrimitive(){return tprim;};
	/*inline*/ void setPrimitive(unsigned int _in){tprim=(MireType)_in;};
	/*inline*/ double get_dX(){return dx;};
	/*inline*/ double get_dY(){return dy;};
	/*inline*/ double get_dZ(){return dz;};
	/*inline*/ double get_nPX(){return nPX;};
	/*inline*/ double get_nPY(){return nPY;};
	/*inline*/ double get_nPZ(){return nPZ;};
	/*inline*/ void set_dX(double in){dx=in;};
	/*inline*/ void set_dY(double in){dy=in;};
	/*inline*/ void set_dZ(double in){dz=in;};
	/*inline*/ void set_nPX(unsigned int in){nPX=in;};
	/*inline*/ void set_nPY(unsigned int in){nPY=in;};
	/*inline*/ void set_nPZ(unsigned int in){nPZ=in;};
};

#endif
