#include <MIXEDVISION/CMireXml.h>

void CMireXml::operator<<(CMire& mire)
{
	xmlOpenToWrite();
	writeModel(&mire);
	xmlWriteToFile();
	xmlEndAccessFile();
}

void CMireXml::operator>>(CMire& mire)
{
	xmlOpenToParse();
	readModel(&mire);	
	xmlEndAccessFile();
}	

xmlNodePtr CMireXml::putModel(CMire* mire)
{
	xmlNodePtr node_mire,node_def,node_ptref,node_ptmire,node_listpt,node_listref;
	xmlNodePtr node_tmp;
	char str[21];
	
	//Mire - parent node
	node_mire = xmlNewNode(NULL,(xmlChar*)LABEL_XML_NODE_MIRE);	

	//Definition - mire node	
	node_def = xmlNewComment((xmlChar*)"Definition of the pattern");
	node_def = xmlNewNode(NULL,(xmlChar*)LABEL_XML_NODE_DEFMIRE);
	
	//Primitive - def
	node_tmp = xmlNewComment((xmlChar*)"Primitive of the pattern");
	xmlAddChild(node_def,node_tmp);
	sprintf(str,"%d",mire->getPrimitive());
	xmlNewTextChild(node_def,NULL,(xmlChar*)LABEL_XML_PRIMITIVE,(xmlChar*)str);

	//Distance - def
	node_tmp = xmlNewComment((xmlChar*)"Distance between two points on the pattern (DX,DY,DZ)");
	xmlAddChild(node_def,node_tmp);
	sprintf(str,"%.10f",mire->get_dX());
	xmlNewTextChild(node_def,NULL,(xmlChar*)LABEL_XML_DSTPRIMITIVEX,(xmlChar*)str);
	sprintf(str,"%.10f",mire->get_dY());
	xmlNewTextChild(node_def,NULL,(xmlChar*)LABEL_XML_DSTPRIMITIVEY,(xmlChar*)str);
	sprintf(str,"%.10f",mire->get_dZ());
	xmlNewTextChild(node_def,NULL,(xmlChar*)LABEL_XML_DSTPRIMITIVEZ,(xmlChar*)str);

	//Nb primitives
	node_tmp = xmlNewComment((xmlChar*)"Number of primitives on the pattern (NX,NY,NZ)");
	xmlAddChild(node_def,node_tmp);
	sprintf(str,"%.10f",mire->get_nPX());
	xmlNewTextChild(node_def,NULL,(xmlChar*)LABEL_XML_NBPRIMITIVEX,(xmlChar*)str);
	sprintf(str,"%.10f",mire->get_nPY());
	xmlNewTextChild(node_def,NULL,(xmlChar*)LABEL_XML_NBPRIMITIVEY,(xmlChar*)str);
	sprintf(str,"%.10f",mire->get_nPZ());
	xmlNewTextChild(node_def,NULL,(xmlChar*)LABEL_XML_NBPRIMITIVEZ,(xmlChar*)str);

	//Pt ref - def		
	node_listref = xmlNewComment((xmlChar*)"Index of reference points on the pattern (X,Y,Z)");
	node_listref = xmlNewNode(NULL,(xmlChar*)LABEL_XML_NODE_LISTPOINTREF);
	mire->pointsref.front();
	for(int i=0;i<mire->pointsref.nb;i++)
	{
		double vTmp;
		CPoint ptTmp = mire->pointsref.value();
		mire->pointsref.next();		
		node_ptref = xmlNewNode(NULL,(xmlChar*)LABEL_XML_NODE_POINTREF);	
		(mire->get_dX())?vTmp = ptTmp.get_oX()/mire->get_dX():vTmp=0;
		sprintf(str,"%.10f",vTmp);
		xmlNewTextChild(node_ptref,NULL,(xmlChar*)LABEL_XML_OX,(xmlChar*)str);
		(mire->get_dY())?vTmp = ptTmp.get_oY()/mire->get_dY():vTmp=0;
		sprintf(str,"%.10f",vTmp);
		xmlNewTextChild(node_ptref,NULL,(xmlChar*)LABEL_XML_OY,(xmlChar*)str);
		(mire->get_dZ())?vTmp = ptTmp.get_oZ()/mire->get_dZ():vTmp=0;
		sprintf(str,"%.10f",vTmp);
		xmlNewTextChild(node_ptref,NULL,(xmlChar*)LABEL_XML_OZ,(xmlChar*)str);
		xmlAddChild(node_listref,node_ptref);
	}
	xmlAddChild(node_def,node_listref);
	xmlAddChild(node_mire,node_def);
	//end of definition

	//List of points on pattern
	node_listpt = xmlNewComment((xmlChar*)"Points on the pattern (X,Y,Z)");
	node_listpt = xmlNewNode(NULL,(xmlChar*)LABEL_XML_NODE_LISTPOINTMIRE);	
	mire->points.front();
	for(int i=0;i<mire->points.nb;i++)
	{
		CPoint ptTmp = mire->points.value();
		mire->points.next();		
		node_ptmire = xmlNewNode(NULL,(xmlChar*)LABEL_XML_NODE_POINTMIRE);	
		sprintf(str,"%.10f",ptTmp.get_oX());
		xmlNewTextChild(node_ptmire,NULL,(xmlChar*)LABEL_XML_OX,(xmlChar*)str);
		sprintf(str,"%.10f",ptTmp.get_oY());
		xmlNewTextChild(node_ptmire,NULL,(xmlChar*)LABEL_XML_OY,(xmlChar*)str);
		sprintf(str,"%.10f",ptTmp.get_oZ());
		xmlNewTextChild(node_ptmire,NULL,(xmlChar*)LABEL_XML_OZ,(xmlChar*)str);
		xmlAddChild(node_listpt,node_ptmire);
		sprintf(str,"%.10f",ptTmp.get_u());
		xmlNewTextChild(node_ptmire,NULL,(xmlChar*)LABEL_XML_U,(xmlChar*)str);
		xmlAddChild(node_listpt,node_ptmire);
		sprintf(str,"%.10f",ptTmp.get_v());
		xmlNewTextChild(node_ptmire,NULL,(xmlChar*)LABEL_XML_V,(xmlChar*)str);
		xmlAddChild(node_listpt,node_ptmire);
	}
	xmlAddChild(node_mire,node_listpt);
	//end of list of points

	return node_mire;
}

int CMireXml::getMire(xmlNodePtr node_mire,CMire* mire)
{
	double vald;
	int vali;
	unsigned int valui;
	xmlNodePtr node_def,
			   node_ptref,
			   node_listref,
			   node_listpt,
			   node_ptmire;
	
	for (node_mire = node_mire->xmlChildrenNode; node_mire != NULL;  node_mire = node_mire->next)
	{
		if(node_mire->type != XML_ELEMENT_NODE) continue;

		//Definition
		if(xmlC2S(node_mire->name) == LABEL_XML_NODE_DEFMIRE)
		{
			for (node_def = node_mire->xmlChildrenNode; node_def != NULL;  node_def = node_def->next)
			{
				if(node_def->type != XML_ELEMENT_NODE) continue;

				//Primitive - def
				if(xmlC2S(node_def->name) == LABEL_XML_PRIMITIVE)
				{					
					xmlReadIntChild (node_def, vali);
					mire->setPrimitive((unsigned int)vali);					
				}

				//Distance - def
				if(xmlC2S(node_def->name) == LABEL_XML_DSTPRIMITIVEX)
				{
					xmlReadDoubleChild (node_def, vald);
					mire->set_dX(vald);
				}

				if(xmlC2S(node_def->name) == LABEL_XML_DSTPRIMITIVEY)
				{
					xmlReadDoubleChild (node_def, vald);
					mire->set_dY(vald);
				}

				if(xmlC2S(node_def->name) == LABEL_XML_DSTPRIMITIVEZ)
				{
					xmlReadDoubleChild (node_def, vald);
					mire->set_dZ(vald);
				}

				//Nb primitives
				if(xmlC2S(node_def->name) == LABEL_XML_NBPRIMITIVEX)
				{
					xmlReadDoubleChild (node_def, vald);
					mire->set_nPX(vald);
				}

				if(xmlC2S(node_def->name) == LABEL_XML_NBPRIMITIVEY)
				{
					xmlReadDoubleChild (node_def, vald);
					mire->set_nPY(vald);
				}

				if(xmlC2S(node_def->name) == LABEL_XML_NBPRIMITIVEZ)
				{
					xmlReadDoubleChild (node_def, vald);
					mire->set_nPZ(vald);
				}

				//Pt ref - def					
				if(xmlC2S(node_def->name) == LABEL_XML_NODE_LISTPOINTREF)
				{
					CPoint ptTmp;
					mire->pointsref.kill();
					//list points ref
					for (node_listref = node_def->xmlChildrenNode; node_listref != NULL;  node_listref = node_listref->next)
						if(xmlC2S(node_listref->name) == LABEL_XML_NODE_POINTREF)							
							for (node_ptref = node_listref->xmlChildrenNode; node_ptref != NULL;  node_ptref = node_ptref->next)
							{
								if(node_ptref->type != XML_ELEMENT_NODE) continue;

								if(xmlC2S(node_ptref->name) == LABEL_XML_OX)
								{
									xmlReadDoubleChild (node_ptref, vald);
									mire->get_dX()?ptTmp.set_oX(vald*mire->get_dX()):ptTmp.set_oX(0);
								}

								if(xmlC2S(node_ptref->name) == LABEL_XML_OY)
								{
									xmlReadDoubleChild (node_ptref, vald);
									mire->get_dY()?ptTmp.set_oY(vald*mire->get_dY()):ptTmp.set_oY(0);
								}

								if(xmlC2S(node_ptref->name) == LABEL_XML_OZ)
								{
									xmlReadDoubleChild (node_ptref, vald);
									mire->get_dZ()?ptTmp.set_oZ(vald*mire->get_dZ()):ptTmp.set_oZ(0);
								}
								mire->pointsref.addRight(ptTmp);
							}
				}				
			}
		}
		//List points on the pattern
		if(xmlC2S(node_mire->name) == LABEL_XML_NODE_LISTPOINTMIRE)
		{
			CPoint ptTmp;
			mire->points.kill();
			//node list points
			for (node_listpt = node_mire->xmlChildrenNode; node_listpt != NULL;  node_listpt = node_listpt->next)
			{
				if(node_listpt->type != XML_ELEMENT_NODE) continue;
				std::cout<< "on passe"<<std::endl;
				//node point mide
				if(xmlC2S(node_listpt->name) == LABEL_XML_NODE_POINTMIRE)
					for (node_ptmire = node_listpt->xmlChildrenNode; node_ptmire != NULL;  node_ptmire = node_ptmire->next)
					{
						if(node_ptmire->type != XML_ELEMENT_NODE) continue;

						if(xmlC2S(node_ptmire->name) == LABEL_XML_OX)
						{
							xmlReadDoubleChild (node_ptmire, vald);
							ptTmp.set_oX(vald);
						}

						if(xmlC2S(node_ptmire->name) == LABEL_XML_OY)
						{
							xmlReadDoubleChild (node_ptmire, vald);
							ptTmp.set_oY(vald);
						}

						if(xmlC2S(node_ptmire->name) == LABEL_XML_OZ)
						{
							xmlReadDoubleChild (node_ptmire, vald);
							ptTmp.set_oZ(vald);
						}

						if(xmlC2S(node_ptmire->name) == LABEL_XML_U)
						{
							xmlReadDoubleChild (node_ptmire, vald);
							ptTmp.set_u(vald);
						}

						if(xmlC2S(node_ptmire->name) == LABEL_XML_V)
						{
							xmlReadDoubleChild (node_ptmire, vald);
							ptTmp.set_v(vald);
						}							
					}			
					mire->points.addRight(ptTmp);
			}
		}		
	}
	return 0;
}


void CMireXml::writeModel(CMire* mire)
{
	xmlAddChild(node,putModel(mire));
}

int CMireXml::readModel(CMire* mire)
{
	int nbMire = 0;

	for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
	{
		if(node->type != XML_ELEMENT_NODE) continue;
		if(xmlC2S(node->name) == LABEL_XML_NODE_MIRE)
		{
			nbMire++;
			getMire(node,mire);			
		}
	}

	//renvoie la derni\E8re camera par defaut
	return nbMire;
}
