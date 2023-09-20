#ifndef AFX_COMNIXML
#define AFX_COMNIXML

#include <MIXEDVISION/CModelXml.h>
#include <MIXEDVISION/COmni.h>

#define LABEL_XML_XI                                "xi"

class COmniXml : public CModelXml
{
private:
	int readModel(COmni*);
	void writeModel(COmni*);

public:
    static xmlNodePtr putModel(COmni*);
	static int getModel(xmlDocPtr &doc, xmlNodePtr,COmni*);
    
	COmniXml(std::string fic):CModelXml(fic){};
	void operator<<(COmni&);
	void operator>>(COmni&);
};

#endif
