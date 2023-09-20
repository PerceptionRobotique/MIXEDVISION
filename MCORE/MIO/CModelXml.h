#ifndef AFX_CMODELXML
#define AFX_CMODELXML

#include <MIXEDVISION/CXml.h>
#include <MIXEDVISION/CModel.h>

#define LABEL_XML_CAMERA                             "camera"
#define LABEL_XML_CAMERA_NAME                        "name"
#define LABEL_XML_WIDTH                              "image_width"
#define LABEL_XML_HEIGHT                             "image_height"
#define LABEL_XML_MODEL                              "model"
#define LABEL_XML_CAMERA_TYPE                         "type"
#define LABEL_XML_CAMERA_OMNI                         "omni"
#define LABEL_XML_CAMERA_PERSP                        "persp"
#define LABEL_XML_CAMERA_FISHEYE                      "fisheye"
#define LABEL_XML_U0                                 "u0"
#define LABEL_XML_V0                                 "v0"
#define LABEL_XML_AU                                 "au"
#define LABEL_XML_AV                                 "av"
#define LABEL_XML_KUD                                "kud"
#define LABEL_XML_KDU                                "kdu"


//!!GERER LES DIFFERENTS MODELES 
class CModelXml : public CXml
{
protected:
	//ModelXML
	int readModel(CModel*);
	void writeModel(CModel*);

	//MireXML
	/*xmlNodePtr putMire(CMire);
	int getMire(xmlNodePtr,CMire);

	//ModelStereoXML
	xmlNodrPtr putModelStereo(*/

public:
    static xmlNodePtr putModel(CModel*);
	static int getModel(xmlDocPtr &doc, xmlNodePtr,CModel*);
	static int getModelType(xmlDocPtr& doc, xmlNodePtr node_model);
    
	CModelXml(std::string fic):CXml(fic){};
	void operator<<(CModel*);
	void operator>>(CModel*);
};

#endif
