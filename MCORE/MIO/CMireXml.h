#ifndef AFX_CMIREXML
#define AFX_CMIREXML

#include <MIXEDVISION/CXml.h>
#include <MIXEDVISION/CMire.h>

#define LABEL_XML_NODE_MIRE                        "mire" //noeud
#define LABEL_XML_NODE_DEFMIRE			    	   "definition" //noeud
#define LABEL_XML_PRIMITIVE						   "primitive"
#define LABEL_XML_DSTPRIMITIVEX					   "dst_primitive_x"
#define LABEL_XML_DSTPRIMITIVEY					   "dst_primitive_y"
#define LABEL_XML_DSTPRIMITIVEZ					   "dst_primitive_z"
#define LABEL_XML_NBPRIMITIVEX					   "nb_primitive_x"
#define LABEL_XML_NBPRIMITIVEY					   "nb_primitive_y"
#define LABEL_XML_NBPRIMITIVEZ					   "nb_primitive_z"
#define LABEL_XML_NODE_LISTPOINTMIRE			   "list_point_mire" //noeud
#define LABEL_XML_NODE_LISTPOINTREF				   "list_point_ref" //noeud
#define LABEL_XML_NODE_POINTREF					   "point_ref" //noeud
#define LABEL_XML_NODE_POINTMIRE				   "point_mire" //noeud
#define LABEL_XML_OX							   "ox"
#define LABEL_XML_OY							   "oy"
#define LABEL_XML_OZ							   "oz"
#define LABEL_XML_U							       "u"
#define LABEL_XML_V							       "v"


//!!GERER LES DIFFERENTS MODELES 
class CMireXml : public CXml
{
protected:
	//ModelXML
	xmlNodePtr putModel(CMire*);
	int getMire(xmlNodePtr,CMire*);
	int readModel(CMire*);
	void writeModel(CMire*);

public:	
	CMireXml(std::string fic):CXml(fic){};
	void operator<<(CMire&);
	void operator>>(CMire&);
};

#endif
