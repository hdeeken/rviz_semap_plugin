/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2013 University of Osnabrück
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in 
 * the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT 
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
 * BUSINESS  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * label_tool.cpp
 *
 *      Author: Tristan Igelbrink <tigelbri@uos.de>
 *  	        Marcel Mrozinski <mmrozins@uos.de>
 *              Johannes Heitmann <joheitma@uos.de>
 *
 */
#include <label_tool.h>

#include <geometry_msgs/Point32.h>

namespace rviz_label_tool
{
const float LabelTool::BOX_SIZE_TOLERANCE = 0.0001;
const size_t LabelTool::MAXIMUM_PICKED_FACES = 10000;
LabelTool::LabelTool()
{
    shortcut_key_ = 'l';
    m_labelDialog = NULL;
}

LabelTool::~LabelTool()
{
    if (m_labelDialog != NULL)
    {
        delete m_labelDialog;
    }

    for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
    {
        it->second.clear();
    }
    m_goalFaces.clear();
    context_->getSceneManager()->destroyManualObject("SelectMesh");
    context_->getSceneManager()->destroyManualObject("SelectionBox");
    context_->getSceneManager()->destroySceneNode(m_sceneNode);
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
void LabelTool::onInitialize()
{
    m_sceneNode = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
    m_selectingMesh = context_->getSceneManager()->createManualObject("SelectMesh");
    m_selectingMesh->setDynamic(false);
    m_sceneNode->attachObject(m_selectingMesh);

    m_selectionBox = context_->getSceneManager()->createManualObject("SelectionBox");
    m_selectionBox->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    m_selectionBox->setUseIdentityProjection(true);
    m_selectionBox->setUseIdentityView(true);
    m_selectionBox->setQueryFlags(0);
    m_sceneNode->attachObject(m_selectionBox);

    m_selectMaterial = Ogre::MaterialManager::getSingleton().create("SelectMaterial",
                                                                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                    true);
    m_selectMaterial->getTechnique(0)->removeAllPasses();
    m_selectMaterial->getTechnique(0)->createPass();
    m_selectMaterial->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(0,
                                                                                0,
                                                                                255,
                                                                                0.5));
    m_selectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,0,0.5);
    m_selectMaterial->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    m_selectMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    m_selectMaterial->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_SOLID);
    m_selectMaterial->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);

    m_selectionBoxMaterial = Ogre::MaterialManager::getSingleton().create("SelectionBoxMaterial",
                                                                          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                          true);
    m_selectionBoxMaterial->getTechnique(0)->removeAllPasses();
    m_selectionBoxMaterial->getTechnique(0)->createPass();
    m_selectionBoxMaterial->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(0,
                                                                                      0,
                                                                                      255,
                                                                                      0.5));
    m_selectionBoxMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,0,0.5);
    m_selectionBoxMaterial->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    m_selectionBoxMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    m_selectionBoxMaterial->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_SOLID);
    m_selectionBoxMaterial->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
    m_singleSelect = false;
    m_singleDeselect = false;
    m_multipleSelect = false;
}

void LabelTool::activate() 
{
    m_labelDialog = new LabelViz(this);
}

void LabelTool::deactivate() 
{ 
}

void LabelTool::clearSelection() 
{
    m_selectingMesh->clear();
    for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
    {
        it->second.clear();
    }
    m_goalFaces.clear();
}

void LabelTool::setDBPanel(DBPanel* dbPanel)
{
    m_dbPanel = dbPanel;
}

DBPanel* LabelTool::getDBPanel()
{
    return m_dbPanel;
}

void LabelTool::getSelectedRooms(std::vector<std::string> &roomIDs)
{
    std::map<std::string, size_t>* roomToMeshIndex = &(m_dbPanel->getRoomToMeshIndexMap());
    for (std::map<size_t, std::vector<size_t> >::iterator itGoalFaces = m_goalFaces.begin(); itGoalFaces != m_goalFaces.end(); itGoalFaces++)
    {
        for (std::map<std::string, size_t>::iterator itRoomToMeshIndex = roomToMeshIndex->begin(); itRoomToMeshIndex != roomToMeshIndex->end(); itRoomToMeshIndex++)
        {
            if (itGoalFaces->first == itRoomToMeshIndex->second)
            {
                roomIDs.push_back(itRoomToMeshIndex->first);
                break;
            }
        }
    }
}

bool LabelTool::areFacesSelected()
{
    return (m_goalFaces.size() > 0);
}

size_t LabelTool::getGoalsectionFromRoom(std::string roomLabel)
{
    return m_dbPanel->getRoomToMeshIndexMap().at(roomLabel);
}

void LabelTool::getSelectedFaces(size_t goalSection, std::string regionLabel, lvr_ros::Mesh &meshMsg)
{
    lvr_ros::LabeledFaces region;
    region.label = regionLabel;
    meshMsg = m_dbPanel->getMeshMsg(goalSection);
    meshMsg.labeledfaces.clear();
    for (size_t i = 0; i < m_goalFaces[goalSection].size(); i++)
    {
        region.ids.push_back(m_goalFaces[goalSection][i] / 3);
    }
    meshMsg.labeledfaces.push_back(region);
}

void LabelTool::updateSelectionBox()
{
    float left, right, top, bottom;
    
    left = m_selectionStart.x * 2 - 1;
    right = m_selectionStop.x * 2 - 1;
    top = 1 - m_selectionStart.y * 2;
    bottom = 1 - m_selectionStop.y * 2;

    m_selectionBox->clear();
    m_selectionBox->begin("SelectionBoxMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    m_selectionBox->position(left, top, -1);
    m_selectionBox->position(right, top, -1);
    m_selectionBox->position(right, bottom, -1);
    m_selectionBox->position(left, bottom, -1);
    m_selectionBox->triangle(0, 1, 2);
    m_selectionBox->triangle(0, 2, 3);
    m_selectionBox->end();
}

void LabelTool::updateSelectionMesh()
{
    size_t facesSize = 0;
    size_t vertexCount = 0;
    Ogre::Vector3* vertices;
    size_t indexCount = 0;
    unsigned long* indices;
    m_selectingMesh->clear();
    Ogre::ManualObject* mesh = context_->getSceneManager()->getManualObject("TriangleMeshDB");
    m_selectingMesh->begin("SelectMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
    {
        getRawManualObjectData(mesh, it->first, vertexCount, vertices, indexCount, indices);
        facesSize += it->second.size();

        for (size_t j = 0; j < it->second.size(); j++)
        {
            m_selectingMesh->position(vertices[indices[it->second[j]]].x,
                                      vertices[indices[it->second[j]]].y, 
                                      vertices[indices[it->second[j]]].z);
            m_selectingMesh->position(vertices[indices[it->second[j] + 1]].x,
                                      vertices[indices[it->second[j] + 1]].y, 
                                      vertices[indices[it->second[j] + 1]].z);
            m_selectingMesh->position(vertices[indices[it->second[j] + 2]].x,
                                      vertices[indices[it->second[j] + 2]].y, 
                                      vertices[indices[it->second[j] + 2]].z);
        }
        delete[] vertices;
        delete[] indices;
    }

    for (size_t j = 0; j < facesSize; j++)
    {
        m_selectingMesh->triangle(3 * j, 3 * j + 2, 3 * j + 1);
    }
    m_selectingMesh->end();
}

// Handling key events to label marked faces or to get db structure
int LabelTool::processKeyEvent(QKeyEvent *event, rviz::RenderPanel* panel)
{
    // if 'n' is pressed start QTWidget labviz to get Label information
    if (event->key() == Qt::Key_N)
    {
        m_labelDialog->exec();
    }

    // if 's' is pressed clear the current selection of faces
    if (event->key() == Qt::Key_R)
    {
        clearSelection();
    }
    return Render;
}

// Handling mouse event and mark the clicked faces
int LabelTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    if (event.leftDown() && event.control())
    {
        selectRegion(event);
    }

    else if (event.leftDown() && event.shift())
    {
        m_multipleSelect = true;
        selectionBoxStart(event);
    }

    else if (event.leftUp() && m_multipleSelect)
    {
        m_multipleSelect = false;
        selectMultipleFaces(event);
    }

    else if (event.rightDown() && event.control())
    {
        deselectRegion(event);
    }
    else if (event.rightDown() && event.shift())
    {
        m_multipleSelect = true;
        selectionBoxStart(event);
    }

    else if (event.rightUp() && m_multipleSelect)
    {
        m_multipleSelect = false;
        deselectMultipleFaces(event);
    }

    else if (m_multipleSelect)
    {
        selectionBoxMove(event);
    }

    else if (event.leftDown())
    {
        m_singleSelect = true;
        selectSingleFace(event);
    }

    else if (event.leftUp())
    {
        m_singleSelect = false;
        selectSingleFace(event);
    }

    else if (m_singleSelect)
    {
        selectSingleFace(event);
    }

    else if (event.rightDown())
    {
        m_singleDeselect = true;
        deselectSingleFace(event);
    }

    else if (event.rightUp())
    {
        m_singleDeselect = false;
        deselectSingleFace(event);
    }

    else if (m_singleDeselect)
    {
        deselectSingleFace(event);
    }

    return Render;
}

void LabelTool::selectionBoxStart(rviz::ViewportMouseEvent& event)
{
    m_selectionStart.x = (float) event.x / event.viewport->getActualWidth();
    m_selectionStart.y = (float) event.y / event.viewport->getActualHeight();
    m_selectionStop = m_selectionStart;
    m_selectionBox->clear();
    m_selectionBox->setVisible(true);
}

void LabelTool::selectionBoxMove(rviz::ViewportMouseEvent& event)
{
    m_selectionStop.x = (float) event.x / event.viewport->getActualWidth();
    m_selectionStop.y = (float) event.y / event.viewport->getActualHeight();
    updateSelectionBox();
}

void LabelTool::selectMultipleFaces(rviz::ViewportMouseEvent& event)
{
    m_selectionBox->setVisible(false);

    float left = m_selectionStart.x;
    float right = m_selectionStop.x;
    float top = m_selectionStart.y;
    float bottom = m_selectionStop.y;
    
    size_t goalSection;
    size_t goalIndex;

    if (left > right)
    {
        std::swap(left, right);
    }

    if (top > bottom)
    {
        std::swap(top, bottom);
    }
    
    if ((right - left) * (bottom - top) < BOX_SIZE_TOLERANCE)
    {
        selectSingleFace(event);
        return;
    }

    Ogre::PlaneBoundedVolume volume = event.viewport->getCamera()->getCameraToViewportBoxVolume(left, top, right, bottom, true);
    Ogre::PlaneBoundedVolumeList volumeList;
    volumeList.push_back(volume);
    Ogre::PlaneBoundedVolumeListSceneQuery* sceneQuery = context_->getSceneManager()->createPlaneBoundedVolumeQuery(Ogre::PlaneBoundedVolumeList());
    sceneQuery->setVolumes(volumeList);
    Ogre::SceneQueryResult& queryResult = sceneQuery->execute();
    Ogre::SceneQueryResultMovableList::iterator iter;
    for (iter = queryResult.movables.begin(); iter != queryResult.movables.end(); ++iter)
    {
        Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(*iter);
        if (mesh->getName().find("TriangleMeshDB") != std::string::npos)
        {
            m_pickingFinished = false;
	    m_startOfPickingFace = 0;
            m_startOfPickingSection = 0;
            while (!m_pickingFinished)
            {
                startPickingMode(mesh);
                Ogre::PlaneBoundedVolumeListSceneQuery* facesSceneQuery = context_->getSceneManager()->createPlaneBoundedVolumeQuery(Ogre::PlaneBoundedVolumeList());
                facesSceneQuery->setVolumes(volumeList);
                Ogre::SceneQueryResult& facesQueryResult = facesSceneQuery->execute();
                Ogre::SceneQueryResultMovableList::iterator facesIter;

                for (facesIter = facesQueryResult.movables.begin(); facesIter != facesQueryResult.movables.end(); ++facesIter)
                {
                    Ogre::ManualObject *faceMesh = static_cast<Ogre::ManualObject*>(*facesIter);
                    if (faceMesh->getName().find("meshCopy") != std::string::npos)
                    {
                        std::string::size_type position = faceMesh->getName().find_first_of('_');
                        goalSection = std::stoi(faceMesh->getName().substr(0, position));
                        position = faceMesh->getName().find_first_of('_', position + 1);
                        goalIndex = std::stoi(faceMesh->getName().substr(position + 1));
                        if (m_goalFaces.find(goalSection) == m_goalFaces.end())
                        {
                            std::vector<size_t> faces;
                            m_goalFaces.insert(std::pair<size_t, std::vector<size_t> >(goalSection, faces));
                        }
                        m_goalFaces[goalSection].push_back(goalIndex);
                        std::sort(m_goalFaces[goalSection].begin(), m_goalFaces[goalSection].end());
                        m_goalFaces[goalSection].erase(std::unique(m_goalFaces[goalSection].begin(),
                                                                   m_goalFaces[goalSection].end()),
                                                       m_goalFaces[goalSection].end());
                    }
                }
                stopPickingMode();
            }

            for (size_t i = 0; i < m_goalFaces.size() ; i++)
            {
                std::sort(m_goalFaces[i].begin(), m_goalFaces[i].end());
                m_goalFaces[i].erase(std::unique(m_goalFaces[i].begin(),
                                                 m_goalFaces[i].end()),
                                     m_goalFaces[i].end());
            }
            updateSelectionMesh();
        }
    }
}

void LabelTool::selectRegion(rviz::ViewportMouseEvent& event)
{
    size_t goalSection;
    size_t goalIndex;
    Ogre::Ray ray = event.viewport->getCamera()->getCameraToViewportRay((float) event.x / event.viewport->getActualWidth(),
                                                                        (float) event.y / event.viewport->getActualHeight());
    Ogre::RaySceneQuery* raySceneQuery = context_->getSceneManager()->createRayQuery(ray,
                                                                                     Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
    raySceneQuery->execute();
    Ogre::RaySceneQueryResult &queryResult = raySceneQuery->getLastResults();
    for (size_t i = 0; i < queryResult.size(); i++)
    {
        Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(queryResult[i].movable);
        if (mesh->getName().find("TriangleMeshDB") != std::string::npos)
        {
            goalSection = -1;
            goalIndex = -1;
            getIdentityOfSingleFace(mesh, ray, goalSection, goalIndex);
            if (goalIndex != -1)
            {
                size_t vertexCount = 0;
                Ogre::Vector3* vertices;
                size_t indexCount = 0;
                unsigned long* indices;
                std::vector<int> usedFaces;
                usedFaces.push_back(goalIndex);
                getRawManualObjectData(mesh, goalSection, vertexCount, vertices, indexCount, indices);
                std::map<size_t, std::vector<size_t> > vertexFaceMap;
                for (size_t i = 0; i < indexCount; i += 3)
                {
                    for (char j = 0; j < 3; j++)
                    {
                        if (vertexFaceMap.find(indices[i + j]) == vertexFaceMap.end())
                        {
                            std::vector<size_t> faces;
                            vertexFaceMap.insert(std::pair<size_t, std::vector<size_t> >(indices[i + j], faces));
                        }
                        vertexFaceMap[indices[i + j]].push_back(i);
                    }
                }
                getConnectedFaces(usedFaces, vertexFaceMap, indices, goalIndex);
                delete [] vertices;
                delete [] indices;
                if (m_goalFaces.find(goalSection) == m_goalFaces.end())
                {
                    std::vector<size_t> faces;
                    m_goalFaces.insert(std::pair<size_t, std::vector<size_t> >(goalSection, faces));
                }
                for (size_t j = 0; j < usedFaces.size(); j++)
                {
                    m_goalFaces[goalSection].push_back(usedFaces[j]);
                }
                std::sort(m_goalFaces[goalSection].begin(), m_goalFaces[goalSection].end());
                m_goalFaces[goalSection].erase(std::unique(m_goalFaces[goalSection].begin(),
                                                           m_goalFaces[goalSection].end()),
                                               m_goalFaces[goalSection].end());
                updateSelectionMesh();
            }
        }
    }
}

void LabelTool::selectSingleFace(rviz::ViewportMouseEvent& event)
{
    size_t goalSection;
    size_t goalIndex;
    Ogre::Ray ray = event.viewport->getCamera()->getCameraToViewportRay((float) event.x / event.viewport->getActualWidth(),
                                                                        (float) event.y / event.viewport->getActualHeight());
    Ogre::RaySceneQuery* raySceneQuery = context_->getSceneManager()->createRayQuery(ray,
                                                                                     Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
    raySceneQuery->execute();
    Ogre::RaySceneQueryResult &queryResult = raySceneQuery->getLastResults();
    for (size_t i = 0; i < queryResult.size(); i++)
    {
        Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(queryResult[i].movable);
        if (mesh->getName().find("TriangleMeshDB") != std::string::npos)
        {
            goalSection = -1;
            goalIndex = -1;
            getIdentityOfSingleFace(mesh, ray, goalSection, goalIndex);
            if (goalIndex != -1)
            {
                if (m_goalFaces.find(goalSection) == m_goalFaces.end())
                {
                    std::vector<size_t> faces;
                    m_goalFaces.insert(std::pair<size_t, std::vector<size_t> >(goalSection, faces));
                }
                m_goalFaces[goalSection].push_back(goalIndex);
                std::sort(m_goalFaces[goalSection].begin(), m_goalFaces[goalSection].end());
                m_goalFaces[goalSection].erase(std::unique(m_goalFaces[goalSection].begin(),
                                                           m_goalFaces[goalSection].end()),
                                               m_goalFaces[goalSection].end());
                updateSelectionMesh();
            }
        }
    }
}

void LabelTool::deselectMultipleFaces(rviz::ViewportMouseEvent& event)
{
    m_selectionBox->setVisible(false);

    float left = m_selectionStart.x;
    float right = m_selectionStop.x;
    float top = m_selectionStart.y;
    float bottom = m_selectionStop.y;
    
    size_t goalSection;
    size_t goalIndex;

    if (left > right)
    {
        std::swap(left, right);
    }

    if (top > bottom)
    {
        std::swap(top, bottom);
    }

    if ((right - left) * (bottom - top) < BOX_SIZE_TOLERANCE)
    {
        deselectSingleFace(event);
        return;
    }

    Ogre::PlaneBoundedVolume volume = event.viewport->getCamera()->getCameraToViewportBoxVolume(left, top, right, bottom, true);
    Ogre::PlaneBoundedVolumeList volumeList;
    volumeList.push_back(volume);
    Ogre::PlaneBoundedVolumeListSceneQuery* sceneQuery = context_->getSceneManager()->createPlaneBoundedVolumeQuery(Ogre::PlaneBoundedVolumeList());
    sceneQuery->setVolumes(volumeList);
    Ogre::SceneQueryResult& queryResult = sceneQuery->execute();
    Ogre::SceneQueryResultMovableList::iterator iter;
    for (iter = queryResult.movables.begin(); iter != queryResult.movables.end(); ++iter)
    {
        Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(*iter);
        if (mesh->getName().find("TriangleMeshDB") != std::string::npos)
        {
            m_pickingFinished = false;
	    m_startOfPickingFace = 0;
            m_startOfPickingSection = 0;
            while (!m_pickingFinished)
            {
                startPickingMode(mesh);
                Ogre::PlaneBoundedVolumeListSceneQuery* facesSceneQuery = context_->getSceneManager()->createPlaneBoundedVolumeQuery(Ogre::PlaneBoundedVolumeList());
                facesSceneQuery->setVolumes(volumeList);
                Ogre::SceneQueryResult& facesQueryResult = facesSceneQuery->execute();
                Ogre::SceneQueryResultMovableList::iterator facesIter;

                for (facesIter = facesQueryResult.movables.begin(); facesIter != facesQueryResult.movables.end(); ++facesIter)
                {
                    Ogre::ManualObject *faceMesh = static_cast<Ogre::ManualObject*>(*facesIter);
                    if (faceMesh->getName().find("meshCopy") != std::string::npos)
                    {
                        std::string::size_type position = faceMesh->getName().find_first_of('_');
                        goalSection = std::stoi(faceMesh->getName().substr(0, position));
                        position = faceMesh->getName().find_first_of('_', position + 1);
                        goalIndex = std::stoi(faceMesh->getName().substr(position + 1));
                        if (m_goalFaces.find(goalSection) != m_goalFaces.end())
                        {
                            if (std::find(m_goalFaces[goalSection].begin(),
                                          m_goalFaces[goalSection].end(),
                                          goalIndex) != m_goalFaces[goalSection].end())
                            {
                                m_goalFaces[goalSection].erase(std::find(m_goalFaces[goalSection].begin(),
                                                                         m_goalFaces[goalSection].end(),
                                                                         goalIndex));
                                if (m_goalFaces[goalSection].size() < 1)
                                {
                                    m_goalFaces.erase(goalSection);
                                }
                            }
                        }
                    }
                }
                stopPickingMode();
            }
            updateSelectionMesh();
        }
    }
}

void LabelTool::deselectRegion(rviz::ViewportMouseEvent& event)
{
    size_t goalSection;
    size_t goalIndex;
    Ogre::Ray ray = event.viewport->getCamera()->getCameraToViewportRay((float) event.x / event.viewport->getActualWidth(),
                                                                        (float) event.y / event.viewport->getActualHeight());
    Ogre::RaySceneQuery* raySceneQuery = context_->getSceneManager()->createRayQuery(ray,
                                                                                     Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
    raySceneQuery->execute();
    Ogre::RaySceneQueryResult &queryResult = raySceneQuery->getLastResults();
    for (size_t i = 0; i < queryResult.size(); i++)
    {
        Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(queryResult[i].movable);
        if (mesh->getName().find("TriangleMeshDB") != std::string::npos)
        {
            goalSection = -1;
            goalIndex = -1;
            getIdentityOfSingleFace(mesh, ray, goalSection, goalIndex);
            if (goalIndex != -1)
            {
                size_t vertexCount = 0;
                Ogre::Vector3* vertices;
                size_t indexCount = 0;
                unsigned long* indices;
                std::vector<int> usedFaces;
                usedFaces.push_back(goalIndex);
                getRawManualObjectData(mesh, goalSection, vertexCount, vertices, indexCount, indices);
                std::map<size_t, std::vector<size_t> > vertexFaceMap;
                for (size_t i = 0; i < indexCount; i += 3)
                {
                    for (char j = 0; j < 3; j++)
                    {
                        if (vertexFaceMap.find(indices[i + j]) == vertexFaceMap.end())
                        {
                            std::vector<size_t> faces;
                            vertexFaceMap.insert(std::pair<size_t, std::vector<size_t> >(indices[i + j], faces));
                        }
                        vertexFaceMap[indices[i + j]].push_back(i);
                    }
                }
                getConnectedFaces(usedFaces, vertexFaceMap, indices, goalIndex);
                delete [] vertices;
                delete [] indices;
                if (m_goalFaces.find(goalSection) != m_goalFaces.end())
                {
                    for (size_t j = 0; j < usedFaces.size(); j++)
                    {
                        if (std::find(m_goalFaces[goalSection].begin(),
                                      m_goalFaces[goalSection].end(),
                                      usedFaces[j]) != m_goalFaces[goalSection].end())
                        {
                            m_goalFaces[goalSection].erase(std::find(m_goalFaces[goalSection].begin(),
                                                                     m_goalFaces[goalSection].end(),
                                                                     usedFaces[j]));
                            if (m_goalFaces[goalSection].size() < 1)
                            {
                                m_goalFaces.erase(goalSection);
                            }
                        }
                    }
                    updateSelectionMesh();
                }
            }
        }
    }
}


void LabelTool::deselectSingleFace(rviz::ViewportMouseEvent& event)
{
    size_t goalSection;
    size_t goalIndex;
    Ogre::Ray ray = event.viewport->getCamera()->getCameraToViewportRay((float) event.x / event.viewport->getActualWidth(),
                                                                        (float) event.y / event.viewport->getActualHeight());
    Ogre::RaySceneQuery* raySceneQuery = context_->getSceneManager()->createRayQuery(ray,
                                                                                     Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
    raySceneQuery->execute();
    Ogre::RaySceneQueryResult &queryResult = raySceneQuery->getLastResults();
    for (size_t i = 0; i < queryResult.size(); i++)
    {
        Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(queryResult[i].movable);
        if (mesh->getName().find("TriangleMeshDB") != std::string::npos)
        {
            goalSection = -1;
            goalIndex = -1;
            getIdentityOfSingleFace(mesh, ray, goalSection, goalIndex);
            if (m_goalFaces.find(goalSection) != m_goalFaces.end())
            {
                if (std::find(m_goalFaces[goalSection].begin(),
                              m_goalFaces[goalSection].end(),
                              goalIndex) != m_goalFaces[goalSection].end())
                {
                    m_goalFaces[goalSection].erase(std::find(m_goalFaces[goalSection].begin(),
                                                             m_goalFaces[goalSection].end(),
                                                             goalIndex));
                    if (m_goalFaces[goalSection].size() < 1)
                    {
                        m_goalFaces.erase(goalSection);
                    }
                    updateSelectionMesh();
                }
            }
        }
    }
}

void LabelTool::getIdentityOfSingleFace(Ogre::ManualObject* mesh,
                                        Ogre::Ray &ray,
                                        size_t &goalSection,
                                        size_t &goalIndex)
{
    Ogre::Real closestDistance = -1.0f;
    size_t vertexCount = 0;
    Ogre::Vector3* vertices;
    size_t indexCount = 0;
    unsigned long* indices;
    size_t numSections = mesh->getNumSections();
    for (size_t i = 0; i < numSections; i++)
    {
        getRawManualObjectData(mesh, i, vertexCount, vertices, indexCount, indices);
        for (size_t j = 0; j < indexCount; j += 3)
        {
            std::pair<bool, Ogre::Real> goal = Ogre::Math::intersects(ray,
                                                                      vertices[indices[j]],
                                                                      vertices[indices[j + 1]],
                                                                      vertices[indices[j + 2]],
                                                                      true,
                                                                      true);
            if (goal.first)
            {
                if ((closestDistance < 0.0f) || (goal.second < closestDistance))
                {
                    closestDistance = goal.second;
                    goalIndex = j;
                    goalSection = i;
                }
            }
        }
        delete[] vertices;
        delete[] indices;
    }
}

void LabelTool::getConnectedFaces(std::vector<int> &usedFaces,
                                  std::map<size_t, std::vector<size_t> > &vertexFaceMap,
                                  unsigned long* &indices,
                                  size_t goalIndex)
{

    for (char i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < vertexFaceMap[indices[goalIndex + i]].size(); j++)
        {
            if (std::find(usedFaces.begin(), usedFaces.end(), vertexFaceMap[indices[goalIndex + i]][j]) == usedFaces.end())
            {
                usedFaces.push_back(vertexFaceMap[indices[goalIndex + i]][j]);
                getConnectedFaces(usedFaces, vertexFaceMap, indices, vertexFaceMap[indices[goalIndex + i]][j]);
            }
        }
    }
}

void LabelTool::getRawManualObjectData(Ogre::ManualObject *mesh,
                                       size_t sectionNumber,
                                       size_t &vertexCount,
                                       Ogre::Vector3* &vertices,
                                       size_t &indexCount,
                                       unsigned long* &indices)
{
    Ogre::VertexData* vertexData;
    const Ogre::VertexElement* vertexElement;
    Ogre::HardwareVertexBufferSharedPtr vertexBuffer;
    unsigned char* vertexChar;
    float* vertexFloat;

    vertexData = mesh->getSection(sectionNumber)->getRenderOperation()->vertexData;
    vertexElement = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
    vertexBuffer = vertexData->vertexBufferBinding->getBuffer(vertexElement->getSource());
    vertexChar = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

    vertexCount = vertexData->vertexCount;
    vertices = new Ogre::Vector3[vertexCount];

    for (size_t i = 0; i < vertexCount; i++, vertexChar += vertexBuffer->getVertexSize())
    {
        vertexElement->baseVertexPointerToElement(vertexChar, &vertexFloat);
        vertices[i] = (mesh->getParentNode()->_getDerivedOrientation() *
                      (Ogre::Vector3(vertexFloat[0], vertexFloat[1], vertexFloat[2]) * mesh->getParentNode()->_getDerivedScale())) + 
                      mesh->getParentNode()->_getDerivedPosition();
    }
    vertexBuffer->unlock();

    Ogre::IndexData* indexData;
    Ogre::HardwareIndexBufferSharedPtr indexBuffer;

    indexData = mesh->getSection(sectionNumber)->getRenderOperation()->indexData;
    indexCount = indexData->indexCount;
    indices = new unsigned long[indexCount];
    indexBuffer = indexData->indexBuffer;

    unsigned int* pLong = static_cast<unsigned int*>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
    unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);
    for (size_t i = 0; i < indexCount; i++)
    {
        unsigned long index;
        if (indexBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
        {
            index = static_cast<unsigned long>(pLong[i]);
        }

        else
        {
            index = static_cast<unsigned long>(pShort[i]);
        }

        indices[i] = index;
    }
    indexBuffer->unlock();
}

void LabelTool::startPickingMode(Ogre::ManualObject *mesh)
{
    std::stringstream sstm;
    Ogre::ManualObject* meshCopy;

    size_t vertexCount;
    Ogre::Vector3* vertices;
    size_t indexCount;
    unsigned long* indices;
    size_t numSections = mesh->getNumSections();
    for (size_t i = m_startOfPickingSection; i < numSections; i++)
    {
        getRawManualObjectData(mesh, i, vertexCount, vertices, indexCount, indices);
        std::vector<Ogre::ManualObject*> faceMeshes;
        m_meshCopy.insert(std::pair<size_t,  std::vector<Ogre::ManualObject*> >(i, faceMeshes));
        for (size_t j = m_startOfPickingFace, k = 0; j < indexCount; j += 3, k += 3)
        {
            sstm.str("");
            sstm << i << "_meshCopy_" << j;
            meshCopy = context_->getSceneManager()->createManualObject(sstm.str());
            meshCopy->setDynamic(false);
            m_sceneNode->attachObject(meshCopy);
            meshCopy->begin("SelectMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            meshCopy->position(vertices[indices[j]].x, vertices[indices[j]].y, vertices[indices[j]].z);
            meshCopy->position(vertices[indices[j + 1]].x, vertices[indices[j + 1]].y, vertices[indices[j + 1]].z);
            meshCopy->position(vertices[indices[j + 2]].x, vertices[indices[j + 2]].y, vertices[indices[j + 2]].z);
            meshCopy->triangle(0, 2, 1);
            meshCopy->end();
            m_meshCopy[i].push_back(meshCopy);
            if (k > MAXIMUM_PICKED_FACES)
            {
                m_startOfPickingFace = j + 3;
                m_startOfPickingSection = i;
                delete [] indices;
                delete [] vertices;
                m_sceneNode->_update(false, false);
                return;
            }
        }
        delete [] indices;
        delete [] vertices;
        m_startOfPickingFace = 0;
    }
    m_sceneNode->_update(false, false);
    m_pickingFinished = true;
}

void LabelTool::stopPickingMode()
{
    Ogre::SceneNode::ObjectIterator iterator = m_sceneNode->getAttachedObjectIterator();
    while (iterator.hasMoreElements())
    {
        Ogre::MovableObject* face = static_cast<Ogre::MovableObject*>(iterator.getNext());
        if (face->getName().find("SelectMesh") == std::string::npos && face->getName().find("SelectionBox") == std::string::npos)
        {
            m_sceneNode->detachObject(face);
            m_sceneNode->getCreator()->destroyMovableObject(face);
        }
    }
    for (size_t i = 0; i < m_meshCopy.size(); i++)
    {
        m_meshCopy[i].clear();
    }
    m_meshCopy.clear();
}
} // end namespace rviz_label_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_label_tool::LabelTool,rviz::Tool )
