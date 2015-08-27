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
 * DAMAGES  (INCLUDING,  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE   OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * rviz_label_tool.cpp
 *
 *	Author: Tristan Igelbrink <tigelbri@uos.de>
 *          Johannes Heitmann <joheitma@uos.de>
 *          Marcel Mrozinski  <mmronzs@uos.de
 *
 */
#ifndef RVIZ_LABEL_TOOL_H
#define RVIZ_LABEL_TOOL_H

#include <vector>
#include <map>
#include <boost/lexical_cast.hpp>

//#include <QMessageBox>
//#include <QApplication>
//#include <QIcon>

#include <ros/console.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/display_group.h>

#ifndef Q_MOC_RUN
#include <rviz/mesh_loader.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreStringConverter.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneQuery.h>

#include <geometry_msgs/Point32.h>
#include <mesh_msgs/TriangleMeshStamped.h>

#include <spatial_db_msgs/ObjectDescription.h>
#include <spatial_db_msgs/ObjectInstance.h>
#include <spatial_db_ros/GetObjectInstances.h>
#include <spatial_db_ros/GetObjectDescriptions.h>
#include <spatial_db_ros/AddTriangleMesh3DModel.h>

#include <lvr_ros/lvr_ros_converter.h>

#endif

//QTWidgets
//#include <label_viz.h>

/**
 *
 *@class LabelTool
 *
 *@brief Implements a rviz tool for marking single faces in a mesh
 *
 * with this rviz tool the user can mark single faces in a displayed
 * OGRE mesh. The marking can be done by left click or with a selection
 * box
 *
 */

// OGRE stuff
namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace lvr_ros_converter
{
class LvrRosConverter;
}

namespace rviz_semap_plugin
{
class LabelTool: public rviz::Tool
{
Q_OBJECT
public:

  LabelTool();
  ~LabelTool();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);
  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  static const float BOX_SIZE_TOLERANCE;
  static const size_t MAXIMUM_PICKED_FACES;

  void clearSelection();

  bool areFacesSelected();
  void getSelectedFaces(size_t goalSection, std::string regionLabel, mesh_msgs::TriangleMesh &meshMsg);

private:

    void initNode();
    void initOGRE();
    bool loadObjectGeometries( std::vector<int> ids );
    bool storeObjectGeometries(int id, mesh_msgs::TriangleMesh mesh);
    void updateSelectionBox();
    void updateSelectionMesh();

    void setReferenceMesh( mesh_msgs::TriangleMesh mesh);
    void getSegmentMesh( mesh_msgs::TriangleMesh& mesh);

    void selectSingleFace(rviz::ViewportMouseEvent& event);
    void deselectSingleFace(rviz::ViewportMouseEvent& event);
    void deselectConnectedFaces(rviz::ViewportMouseEvent& event);
    void selectConnectedFaces(rviz::ViewportMouseEvent& event);

    //void selectBoxFaces(rviz::ViewportMouseEvent& event);
    //void deselectBoxFaces(rviz::ViewportMouseEvent& event);
    //void selectionBoxStart(rviz::ViewportMouseEvent& event);
    //void selectionBoxMove(rviz::ViewportMouseEvent& event);

    bool singleRayQuery(rviz::ViewportMouseEvent& event, int num_results, Ogre::Ray& ray);

    void getIdentityOfSingleFace(Ogre::ManualObject* mesh,
                                 Ogre::Ray &ray,
                                 size_t &goalSection,
                                 size_t &goalIndex);

    void getConnectedFaces(std::vector<int> &usedFaces,
                           std::map<size_t, std::vector<size_t> > &vertexFaceMap,
                           unsigned long* &indices,
                           size_t goalIndex);

    void getRawManualObjectData(Ogre::ManualObject *mesh,
                                size_t sectionNumber,
                                size_t &vertexCount,
                                Ogre::Vector3* &vertices,
                                size_t &indexCount,
                                unsigned long* &indices);

    void restoreTopology( mesh_msgs::TriangleMesh& mesh );

    void startPickingMode(Ogre::ManualObject *mesh);
    void stopPickingMode();

    Ogre::ManualObject* reference_mesh;
    Ogre::MaterialPtr reference_mesh_material;

    Ogre::ManualObject* segment_mesh;
    Ogre::MaterialPtr segment_mesh_material;

    Ogre::ManualObject* selection_box;
    Ogre::MaterialPtr selection_box_material;

    Ogre::SceneNode* scene_node;
    std::map<size_t, std::vector<size_t> > m_goalFaces;
    std::map<std::string, size_t>* m_roomToMeshIndex;
    mesh_msgs::TriangleMesh* m_meshMsg;

    Ogre::Vector2 m_selectionStart;
    Ogre::Vector2 m_selectionStop;
    std::map<size_t, std::vector<Ogre::ManualObject*> > m_meshCopy;
    bool m_singleSelect;
    bool m_singleDeselect;
    bool m_multipleSelect;

    size_t m_startOfPickingFace;
    size_t m_startOfPickingSection;
    bool m_pickingFinished;

    ros::NodeHandle n;
    ros::ServiceClient get_object_descriptions_client;
    ros::ServiceClient get_object_geometries_client;
    ros::ServiceClient add_triangle_mesh_client;
    ros::Publisher mesh_pub;

    int object_id;
    bool object_loaded;
    int num_results;

    lvr_ros_converter::LvrRosConverter converter;

    int reference_color_r = 0;
    int reference_color_g = 155;
    int reference_color_b = 155;
    float reference_color_a = 0.5;

    int segment_color_r = 0;
    int segment_color_g = 0;
    int segment_color_b = 255;
    float segment_color_a = 0.5;

    mesh_msgs::TriangleMesh absolute_reference_mesh;
    mesh_msgs::TriangleMesh relative_reference_mesh;

};
}
#endif
