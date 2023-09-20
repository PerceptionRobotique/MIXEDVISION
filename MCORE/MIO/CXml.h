#ifndef AFX_CXML
#define AFX_CXML

#define LABEL_XML_ROOT                               "root"

#include <libxml/xmlmemory.h> 
#include <string>

class CXml
{
protected:
	xmlNodePtr node;
	xmlDocPtr doc;
	std::string file;

	CXml(std::string ficIn);
	CXml();
	
	void xmlOpenToParse();
	void xmlOpenToWrite();
	void xmlEndAccessFile();
	void setPath(std::string nPath);
	std::string getPath(){ return file; };
	static void xmlReadCharChild(xmlDocPtr&, xmlNodePtr &node,char **res);
	static void xmlReadDoubleChild(xmlDocPtr&, xmlNodePtr node, double &res);
	static void xmlReadIntChild(xmlDocPtr&,xmlNodePtr &node,int &res);
	static std::string xmlC2S(const xmlChar *str){ std::string tmp=(char*)str; return tmp;};
	
public:
	void xmlWriteToFile();
};

#endif
