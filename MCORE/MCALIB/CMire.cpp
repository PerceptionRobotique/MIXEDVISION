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

TODO : test des BBox
*/

#include <MIXEDVISION/CMire.h>

#include <MIXEDVISION/CRing.h>
#include <MIXEDVISION/CCorner.h>

#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpRect.h>
#include <visp/vpMath.h>

#include <visp/vpHomogeneousMatrix.h>

CMire::CMire()
{
	points.kill();
	pointsref.kill();

	cMo.eye();

	dx=dy=dz=0.0;
	size = 0.0;
}

CMire::~CMire()
{
	clearPrimitives();
}

void
CMire::clearPrimitives()
{
	primitives.front();
	while(!primitives.outside())
	{
		vpTracker *t = primitives.value();
		delete t;
		primitives.next();
	}
	primitives.kill();
}

void
CMire::initMire(std::string nomfichiermire) //=4 par defaut
{
	//Calibration grid parameters
	unsigned int typePrim; //size of the calibration grid along x, y and z axis 

	// Chargement des informations du fichier nomfichiermire
	std::ifstream paramsMire(nomfichiermire.c_str());

	if (paramsMire.is_open() == false)
	{
		std::cerr << "Can't open file \"" << nomfichiermire.c_str() << "\"" << std::endl;
		return;
	}

	// Chargement du type de primitive présent sur la mire
	paramsMire >> typePrim;
	
	//std::cout << typePrim << std::endl;
	switch(typePrim)
	{
		case 0 : tprim = MIRE_DOTS;
			break;
		case 1 : tprim = MIRE_CORNERS;
			break;
		case 2 : tprim = MIRE_RINGS;
			break;
		default:
#ifdef AFFDETAILSTXT
			std::cout << "initMire : Type de primitive inconnu" << std::endl;
#endif
			break;
	}

	// Chargement des Lx, Ly, Lz
	paramsMire >> dx >> dy >> dz;
	//std::cout << Lx << " " << Ly << " " << Lz << std::endl;
	// Chargement des sizeX, sizeY, sizeZ
	paramsMire >> nPX >> nPY >> nPZ;
	nbpt = nPX*nPY;
	//std::cout << sizeX << " " << sizeY << " " << sizeZ << std::endl;
	//Chargement du nptpose
	paramsMire >> nptpose;
	//std::cout << nptpose << std::endl;
	// Chargement des 4 points servant au premier calcul de pose
	//plan xOy
	pointsref.kill();
	pointsref.front();
	
	CPoint PTmp;
	double ix, iy, iz;
	for(int i = 0;i < nptpose;i++)
	{
		paramsMire >> ix >> iy >> iz;
		//std::cout << ix << " " << iy << " " << iz << std::endl;
		PTmp.setWorldCoordinates(ix*dx,iy*dy,iz*dz);
		pointsref.addRight(PTmp);
	}

	//Fin du chargement d'infos du fichier parametresMire.txt
	paramsMire.close();
	
	points.kill();
	points.front();
	for (unsigned int k=0 ; k < nPZ ; k++)
	{
		for (unsigned int i=0 ; i < nPX ; i++)
		{
			for(unsigned int j=0 ; j < nPY ; j++)
			{
				PTmp.setWorldCoordinates(i*dx,j*dy,k*dz);
				points.addRight(PTmp);
			}
		}
	}
}

void
CMire::initMire(MireType & tm, ParamMire & pm)
{
	//std::cout << tm << std::endl;
	switch(tm)
	{
		case 0 : tprim = MIRE_DOTS;
			break;
		case 1 : tprim = MIRE_CORNERS;
			break;
		case 2 : tprim = MIRE_RINGS;
			break;
		default:
#ifdef AFFDETAILSTXT
			std::cout << "initMire : Type de primitive inconnu" << std::endl;
#endif
			break;
	}

	// Chargement des Lx, Ly, Lz
	dx = pm.Lx;
	dy = pm.Ly;
	dz = pm.Lz;
	
	// Chargement des sizeX, sizeY, sizeZ
	nPX = pm.nbx;
	nPY = pm.nby;
	nPZ = pm.nbz;
	
	nbpt = nPX*nPY*nPZ;

	//Chargement du nptpose
	nptpose = pm.nptPose;

	// Chargement des 4 points servant au premier calcul de pose
	//plan xOy
	pointsref.kill();
	pointsref.front();
	
	CPoint PTmp;
	double ix, iy, iz;
	for(int i = 0;i < nptpose;i++)
	{
		ix = pm.pointsGrilleRef[i][0];
		iy = pm.pointsGrilleRef[i][1];
		iz = pm.pointsGrilleRef[i][2];

		//std::cout << ix << " " << iy << " " << iz << std::endl;
		PTmp.setWorldCoordinates(ix*dx,iy*dy,iz*dz);
		pointsref.addRight(PTmp);
	}
	
	points.kill();
	points.front();
	for (unsigned int k=0 ; k < nPZ ; k++)
	{
		for (unsigned int i=0 ; i < nPX ; i++)
		{
			for(unsigned int j=0 ; j < nPY ; j++)
			{
				PTmp.setWorldCoordinates(i*dx,j*dy,k*dz);
				points.addRight(PTmp);
			}
		}
	}
}

int
CMire::addPointsMire(MireType & tm, ParamMire & pm, PoseVector &pv )
{
	if(tm != tprim)
	{
#ifdef AFFDETAILSTXT
		std::cout << "addPointsMire : un seul type de mire à la fois" << std::endl;
		return -1;
#endif
	}
	
	nbpt += pm.nbx*pm.nby*pm.nbz;

	//Chargement du nptpose
	nptpose += pm.nptPose;

	// Chargement des 4 points servant au premier calcul de pose
	//plan xOy
	vpHomogeneousMatrix o1Mo2;
	o1Mo2.buildFrom(pv[0], pv[1], pv[2], pv[3], pv[4], pv[5]);
	std::cout << "o1Mo2 : " << o1Mo2 << std::endl;
	CPoint PTmp;
	double ix, iy, iz;
	for(int i = 0;i < pm.nptPose;i++)
	{
		ix = pm.pointsGrilleRef[i][0];
		iy = pm.pointsGrilleRef[i][1];
		iz = pm.pointsGrilleRef[i][2];

		//std::cout << ix << " " << iy << " " << iz << std::endl;
		PTmp.setWorldCoordinates(ix*dx,iy*dy,iz*dz);
		std::cout << "oX avant : " << PTmp.get_oX() << " " << PTmp.get_oY() << " " << PTmp.get_oZ() << std::endl;
		PTmp.changeFrame(o1Mo2);
		PTmp.setWorldCoordinates(PTmp.get_X(),PTmp.get_Y(),PTmp.get_Z());
		std::cout << "oX apres : " << PTmp.get_oX() << " " << PTmp.get_oY() << " " << PTmp.get_oZ() << std::endl;
		pointsref.addRight(PTmp);
	}
	
	for (unsigned int k=0 ; k < pm.nbz ; k++)
	{
		for (unsigned int i=0 ; i < pm.nbx ; i++)
		{
			for(unsigned int j=0 ; j < pm.nby ; j++)
			{
				PTmp.setWorldCoordinates(i*dx,j*dy,k*dz);
				PTmp.changeFrame(o1Mo2);
				PTmp.setWorldCoordinates(PTmp.get_X(),PTmp.get_Y(),PTmp.get_Z());
				points.addRight(PTmp);
			}
		}
	}

	return 0;
}

void
CMire::operator=(CMire& _mire)
{
	clearPrimitives();
	
	primitives.front();
	_mire.primitives.front();
	while(!_mire.primitives.outside())
	{
		vpTracker *t = new vpTracker();
		*t = *(_mire.primitives.value());
		primitives.addRight(t);

		_mire.primitives.next();
	}
	points = _mire.points;
	pointsref = _mire.pointsref;
	nptpose = _mire.nptpose;
	nbpt = _mire.nbpt;
	priminitparam = _mire.priminitparam;
	tprim = _mire.tprim;
}

void
CMire::getPrimRefs(std::vector<SImagePoint> & ptsMire)
{
    SImagePoint ptIm;
    
    ptsMire.clear();
	pointsref.front();
	while(!pointsref.outside())
	{
        ptIm.u = pointsref.value().get_u();
        ptIm.v = pointsref.value().get_v();
        
		ptsMire.push_back(ptIm);
		pointsref.next();
	}
}

void
CMire::getPrimitives(std::vector<SImagePoint> & ptsMire)
{
    SImagePoint ptIm;
    
    ptsMire.clear();
	points.front(); //primitives
	while(!points.outside())
	{
        /*
		switch(tprim)
		{
            case MIRE_DOTS :
            {
                vpImagePoint ip;
                ip = ((vpDot2 *)(primitives.value()))->getCog();
                ptIm.u = ip.get_u();
                ptIm.v = ip.get_v();
            }
                break;
            case MIRE_CORNERS :
                ptIm.u = ((CCorner *)(primitives.value()))->get_u();
                ptIm.v = ((CCorner *)(primitives.value()))->get_v();
                break;
            case MIRE_RINGS :
                ptIm.u = ((CRing *)(primitives.value()))->get_u();
                ptIm.v = ((CRing *)(primitives.value()))->get_v();
                break;
            default :
                return;
		}
         */
        ptIm.u = points.value().get_u();
        ptIm.v = points.value().get_v();
		ptsMire.push_back(ptIm);
		points.next();
	}
}

void
CMire::initParametresPrimitive(vpTracker *_priminit)
{
	priminitparam = _priminit;
}

void
CMire::setParametresPrimitive(vpTracker *primAinit)
{
	switch(tprim)
	{
		case MIRE_DOTS :
			{
			vpDot2 *primAinitb = (vpDot2 *)primAinit;
			*primAinitb = *((vpDot2 *)priminitparam);
			}
			break;
		case MIRE_CORNERS :
			{
			CCorner *primAinitb = (CCorner *)primAinit;
			*primAinitb = *((CCorner *)priminitparam);
			}
			break;
		case MIRE_RINGS :
			{
			CRing *primAinitb = (CRing *)primAinit;
			*primAinitb = *((CRing *)priminitparam);
			}
			break;
		default :
			return;
	}
}

int
CMire::supprimeUneDesPrimRef(unsigned int curPt)
{
	int retour = 0;

	pointsref.front();

	for(unsigned int i=0;i<curPt;i++)
	  pointsref.next();
        
	pointsref.suppress();
	
	return retour;
}

int
CMire::initTrackingUneDesPrimRef(vpImage<unsigned char>& I, CModel *cam, unsigned int curPt, vpImagePoint & ip,  unsigned int _size)
{
	int retour = 0;
	vpTracker *primref;
    
	if(!_size)
		size = 0;
    
	switch(tprim)
	{
		case MIRE_DOTS :
			primref = new vpDot2();
			break;
		case MIRE_CORNERS :
			primref = new CCorner();
			break;
		case MIRE_RINGS :
			primref = new CRing();
			break;
		default :
			return 1;
	}
    
	try
	{
		pointsref.front();

		for(unsigned int i=0;i<curPt;i++)
            pointsref.next();
        
        CPoint P = pointsref.value();
		std::cout << "P clique : " << P.get_oX() << " " << P.get_oY() << " " << P.get_oZ() << std::endl;
		setParametresPrimitive(primref);
            
        try
        {
            switch(tprim)
            {
                case MIRE_DOTS :
                    ((vpDot2 *)primref)->initTracking(I, ip);
                    break;
                case MIRE_CORNERS :
                    /*retour =*/ ((CCorner *)primref)->initTracking(I, ip.get_u(), ip.get_v());
                    break;
                case MIRE_RINGS :
                    ((CRing *)primref)->initTracking(I, ip.get_u(), ip.get_v());
                    break;
                default :
                    return 2;
            }
            
        }
        catch (const vpException &e)
        {
#ifdef AFFDETAILSTXT
            std::cerr << "Erreur init tracking with exception: " << e << std::endl;
#endif
            return 3;
        }
        
        /*if(retour)// && i==0)
         {
         break;
         }*/
        
        
        double u, v;
        switch(tprim)
        {
            case MIRE_DOTS :
            {
                vpDot2 *d = ((vpDot2 *)primref);
                u = d->getCog().get_u();
                v = d->getCog().get_v();
                if(!_size)
                    size += d->getWidth()+d->getHeight();
            }
                break;
            case MIRE_CORNERS :
            {
                CCorner *d = ((CCorner *)primref);
                u = d->get_u();
                v = d->get_v();
            }
                break;
            case MIRE_RINGS :
            {
                CRing *d = ((CRing *)primref);
                u = d->get_u();
                v = d->get_v();
            }
                break;
            default :
                return 3;
        }
        
        ip.set_uv(u, v);        
        
        // Conversion des u,v en x,y
        P.setPixUV(u, v);

        cam->pixelMeterConversion(P);

        pointsref.modify(P);

		if(!_size)
			size /= nptpose;
		//std::cout << "dotSize : " << size << std::endl;
	}
	catch(vpException e){
		vpERROR_TRACE("Error while tracking dots") ;
		vpCTRACE << e;
		delete primref;
		return 4;
	}
    
	delete primref;
	
	return retour;
}


void
CMire::initTrackingNoDisplay(vpImage<unsigned char>& I, CModel *cam, vpHomogeneousMatrix & cMo, unsigned int _size)
{
	if(_size)
		size = _size;    
    
	//now we detect all dots of the grid
	vpTracker *priminit;
    
	switch(tprim)
	{
		case MIRE_DOTS :
			priminit = new vpDot2();
			break;
		case MIRE_CORNERS :
			priminit = new CCorner();
			break;
		case MIRE_RINGS :
			priminit = new CRing();
			break;
		default :
			return;
	}
    
	setParametresPrimitive(priminit);
    
	clearPrimitives();
	primitives.front();
    
	for(unsigned int i=0;i<nbpt;i++)
	{
		vpTracker *t;
        
		switch(tprim)
		{
			case MIRE_DOTS :
				{
				t = new vpDot2(*(vpDot2 *)priminit);
				break;
				}
			case MIRE_CORNERS :
				t = new CCorner(*(CCorner *)priminit);
				break;
			case MIRE_RINGS :
				t = new CRing(*(CRing *)priminit);
				break;
			default :
				return;
		}
        
		primitives.addRight(t);
	}

	std::cout << " initTrackingNoDisplay : ";
	cMo.print(); std::cout << std::endl;
    
	// pixel-> meter conversion
	bool valid;
	points.front();
	primitives.front();
	int fact = (int)(1+vpMath::maximum(I.getWidth(), I.getHeight())*0.001);
	for (unsigned int i=0 ; i < nbpt ; i++)
	{
		CPoint P = points.value();
		vpTracker *mp = primitives.value(); //moving primitive
		double u=0.,v=0., ut, vt;
        
		valid = true;
		P.changeFrame(cMo);
		
		if(P.get_Z() < 0) //(false)// false only relevant to OCamCalib's dataset images
			valid = false;
		else
		{
			cam->project3DImage(P);
			cam->meterPixelConversion(P);
		      
			u = P.get_u();
			v = P.get_v();
		      
			if((10<u) && (u<(I.getWidth()-10)) && (10<v) && (v<(I.getHeight()-10)))
			{
				try
				{
					vpRect bbox;
					switch(tprim)
					{
						case MIRE_DOTS :
		                  {
		                      vpDot2 *d = (vpDot2 *)mp;
		                      d->initTracking(I, vpImagePoint(v, u));//, (unsigned int)size);
		                      bbox = d->getBBox();
		                      ut = d->getCog().get_u();
		                      vt = d->getCog().get_v();
													//std::cout << "dot : " << u << " " << v << " " << ut << " " << vt << std::endl;
		                      if( (bbox.getLeft()<5) || (bbox.getRight()>(double)I.getWidth()-5) ||
		                         (bbox.getTop()<5) || (bbox.getBottom()>(double)I.getHeight()-5)
		                         || (vpMath::abs(u-ut)>10) || (vpMath::abs(v-vt)>10) )
		                      {
		                          valid = false;
		                      }
		                      
		                  }
							break;
						case MIRE_CORNERS :
		                  {
		                      unsigned int winuv[2];
		                      CCorner *d = (CCorner *)mp;
		                      d->initTracking(I, (unsigned int)u, (unsigned int)v);//, (unsigned int)dotSize);
		                      d->getWin(winuv[0], winuv[1]);
		                      /*winuv[0] /= 2;
		                       winuv[1] /= 2;
		                       winuv[0]++; winuv[1]++;*/
		                      ut = d->get_u();
		                      vt = d->get_v();
		                      /*if( (vpMath::abs(u-ut)>8) || (vpMath::abs(v-vt)>8) )
		                      {
		                          valid = false;
		                      }*/
		                  }
							break;
						case MIRE_RINGS :
		                  {
		                      CRing *d = (CRing *)mp;
		                      d->initTracking(I, (unsigned int)u, (unsigned int)v, (unsigned int)size);
		                      bbox = d->getBBox();
		                      ut = d->get_u();
		                      vt = d->get_v();
		                      
		                      if(bbox.getLeft()<5 || bbox.getRight()>(double)I.getWidth()-5 ||
		                         bbox.getTop()<5 || bbox.getBottom()>(double)I.getHeight()-5
		                         || vpMath::abs(u-ut)>10 || vpMath::abs(v-vt)>10)
		                      {
		                          valid = false;
		                      }
		                      
		                  }
							break;
						default :
							return;
					}
				
					if(valid)
					{
						//u v are expressed in pixel
						// conversion in meter
						P.setPixUV(ut, vt);
						cam->pixelMeterConversion(P);
		                  
						points.modify(P);
						primitives.modify(mp);
					}
		              
				}
				catch(...)
				{
					valid = false;
				}
			}
			else
			{
				valid = false;
			}
		}
        
		if(valid)
		{
			primitives.next();
			points.next();
		}
		else
		{
			primitives.suppress();
			points.suppress();
		}
        
	}
    
	nbpt = points.nbElements();
    
	delete priminit;
}

void
CMire::initTracking(std::ifstream & fichierpoints)
{
	double oX, oY, oZ, u, v;

	fichierpoints >> nbpt;
//	std::cout << "nbPts : " << nbpt << std::endl;

	points.kill();
	points.front();

  	for (unsigned int i=0 ; i < nbpt ; i++)
	{
		CPoint PTmp;
		fichierpoints >> oX >> oY >> oZ >> u >> v;
//		std::cout << "oX : " << oX << " oY : " << oY << " oZ : " << oZ << " u : " << u << " v : " << v << std::endl;
		PTmp.setObjetImage(oX, oY, oZ, u, v);
		//Ajout d'une etape de tracking ?... (utile dans le cas où l'extraction a été faite avec vpdot2 et que l'on souhaite utiliser des vpRing et vice versa
		points.addRight(PTmp);
	}
}

//static
int
CMire::initTrackingMires(std::string nomfichierpoints, vpList<CMire> & ensemblemires, vpList<vpHomogeneousMatrix> & lcMo)
{
	std::ifstream pff(nomfichierpoints.c_str()); //pointsFromFile
	int nbImgs;
	double tx,ty,tz,rx,ry,rz;

	if (pff.is_open() == false)
	{
		std::cerr << "Can't open file \"" << nomfichierpoints.c_str() << "\"" << std::endl;
		return 0;
	}

	pff >> nbImgs;
#ifdef AFFDETAILSTXT
	std::cout << "nbImgs : " << nbImgs << std::endl;
#endif

	ensemblemires.kill();
	ensemblemires.front();
	int niter = -1;
	while(!pff.eof() && (++niter < nbImgs))
	//for (int niter = 0 ; niter < nbImgs ; niter++)
	{
		CMire miretmp;
		miretmp.initTracking(pff);
		
		pff >> tx >> ty >> tz >> rx >> ry >> rz;
		vpPoseVector r(tx,ty,tz,rx,ry,rz);
		lcMo.addRight(vpHomogeneousMatrix(r));
		
		ensemblemires.addRight(miretmp);
	}

	pff.close();

	return niter; //nbImgs
}

//static
void
CMire::saveALL(std::string filename, vpList<CMire> & ensemblemires)
{
	//std::cout << filename.c_str() << std::endl;
	std::ofstream ptf(filename.c_str()); //points to file

	//Sauvegarde du nombre d'images de mire
	ptf << ensemblemires.nbElements() << std::endl;

	ensemblemires.front();
	while(!(ensemblemires.outside()))
	{
		//std::cout << "mire" << std::endl;
		vpList<CPoint> lp = (ensemblemires.value()).points;
		ptf << lp.nbElements() << std::endl;

		lp.front();
		while(!(lp.outside()))
		{
			CPoint p = lp.value();
			ptf << p.get_oX() << " " << p.get_oY() << " " << p.get_oZ() << " "
				<< p.get_u() << " " << p.get_v() << std::endl;

			lp.next();
		}

		ensemblemires.next();
	}

	ptf.close();
}

//static
void
CMire::save(std::string filename, CMire & mire, vpHomogeneousMatrix & cMo, int nbElements)
{
	//std::cout << filename.c_str() << std::endl;
	std::ios_base::openmode mode = std::ios_base::out;
	if(nbElements == -1)
		mode = std::ios_base::app;
	
	std::ofstream ptf(filename.c_str(), mode); //points to file
	
	
	if(nbElements != -1)
		ptf << nbElements << std::endl; //Sauvegarde du nombre d'images de mire

	
	vpList<CPoint> lp = mire.points;
	ptf << lp.nbElements() << std::endl;
		
	lp.front();
	while(!(lp.outside()))
	{
		CPoint p = lp.value();
		ptf << p.get_oX() << " " << p.get_oY() << " " << p.get_oZ() << " "
			<< p.get_u() << " " << p.get_v() << std::endl;
			
		lp.next();
	}
	
	vpPoseVector r(cMo);
	ptf << r.t() << std::endl;
	
	ptf.close();
}
