#include <MIXEDVISION/COmniXml.h>

xmlNodePtr COmniXml::putModel(COmni* cam)
{
	xmlNodePtr node_tmp;
	xmlNodePtr node_model = CModelXml::putModel((CModel*)cam);
	char str[21];

	node_tmp = xmlNewComment((xmlChar*)"Xi");
	xmlAddChild(node_model,node_tmp);	
	sprintf(str,"%.10f",cam->getXi());
	xmlNewTextChild(node_model,NULL,(xmlChar*)LABEL_XML_XI,(xmlChar*)str);	

	return node_model;
}

int COmniXml::getModel(xmlDocPtr& doc, xmlNodePtr node_model,COmni* cam)
{
	std::string camera_name_tmp = "";
	double u0,v0,au,av,vald,xi;
	int vali;
	ModelType model_type;
	char *val_char;

	for (node_model = node_model->xmlChildrenNode; node_model != NULL;  node_model = node_model->next)
	{
		if(node_model->type != XML_ELEMENT_NODE) continue;

		if(xmlC2S(node_model->name) == LABEL_XML_CAMERA_NAME)
		{
			xmlReadCharChild (doc,node_model, &val_char);
			camera_name_tmp = val_char;
		}

		if(xmlC2S(node_model->name) == LABEL_XML_CAMERA_TYPE)
		{
			xmlReadIntChild (doc,node_model, vali);
			model_type = (ModelType)vali;
		}

		if(xmlC2S(node_model->name) == LABEL_XML_U0)
		{
			xmlReadDoubleChild (doc,node_model, vald);
			u0=vald;
		}

		if(xmlC2S(node_model->name) == LABEL_XML_V0)
		{
			xmlReadDoubleChild (doc,node_model, vald);
			v0=vald;
		}

		if(xmlC2S(node_model->name) == LABEL_XML_AU)
		{
			xmlReadDoubleChild (doc,node_model, vald);
			au=vald;
		}

		if(xmlC2S(node_model->name) == LABEL_XML_AV)
		{
			xmlReadDoubleChild (doc,node_model, vald);
			av=vald;
		}

		if(xmlC2S(node_model->name) == LABEL_XML_XI)
		{
			xmlReadDoubleChild (doc,node_model, vald);
			xi=vald;
		}

	}
	cam->init(au,av,u0,v0,xi);
	//cam->setName(camera_name_tmp);
	//cam->setType(model_type);

	return 0;
}

void COmniXml::writeModel(COmni* cam)
{
	xmlAddChild(node,putModel(cam));
}

int COmniXml::readModel(COmni* cam)
{
	int nbCamera = 0;

	for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
	{
		if(node->type != XML_ELEMENT_NODE) continue;
		if(xmlC2S(node->name) == LABEL_XML_CAMERA)
		{
			nbCamera++;
			getModel(doc,node,cam);
		}
	}

	//renvoie la derni�re camera par defaut
	return nbCamera;
}


void COmniXml::operator<<(COmni &cam)
{
	xmlOpenToWrite();
	writeModel(&cam);
	xmlWriteToFile();
	xmlEndAccessFile();
}

void COmniXml::operator>>(COmni &cam)
{
	xmlOpenToParse();
	readModel(&cam);	
	xmlEndAccessFile();
}
