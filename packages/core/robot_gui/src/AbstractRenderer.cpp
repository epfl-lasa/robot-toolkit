/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "AbstractRenderer.h"

AbstractRenderer::AbstractRenderer(){
}
AbstractRenderer::~AbstractRenderer(){
    Free();
}

void  AbstractRenderer::Free(){
    for(int i=0;i<int(mSubRenderers.size());i++){
        delete mSubRenderers[i];
    }
    mSubRenderers.clear();
}
  
void  AbstractRenderer::Render(){
    for(int i=0;i<int(mSubRenderers.size());i++){
        mSubRenderers[i]->Render();
    }
}
void  AbstractRenderer::RenderShadow(int pass, const Vector3& light, const Vector3& observer){
    for(int i=0;i<int(mSubRenderers.size());i++){
        mSubRenderers[i]->RenderShadow(pass,light,observer);
    }
}
void    AbstractRenderer::RenderOutline(const Vector3& observer){
    for(int i=0;i<int(mSubRenderers.size());i++){
        mSubRenderers[i]->RenderOutline(observer);
    }
}
void    AbstractRenderer::RenderBoundingBox(){
    for(int i=0;i<int(mSubRenderers.size());i++){
        mSubRenderers[i]->RenderBoundingBox();
    }
}

bool  AbstractRenderer::Load(const pXmlTree tree){  
    return true;
}
bool  AbstractRenderer::Configure(const pXmlTree tree){
    for(int i=0;i<int(mSubRenderers.size());i++){
        mSubRenderers[i]->Configure(tree);
    }
    return true;
}
void  AbstractRenderer::AddRenderer(AbstractRenderer* renderer){
    mSubRenderers.push_back(renderer);
}

