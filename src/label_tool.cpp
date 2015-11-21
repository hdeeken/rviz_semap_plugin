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
 *              Marcel Mrozinski <mmrozins@uos.de>
 *              Johannes Heitmann <joheitma@uos.de>
 *
 */
#include <label_tool.h>

namespace rviz_semap_plugin
{
const float LabelTool::BOX_SIZE_TOLERANCE = 0.0001;
const size_t LabelTool::MAXIMUM_PICKED_FACES = 10000;

LabelTool::LabelTool()
{
  shortcut_key_ = 'l';
  object_id = 92;
  num_results = 10;
  object_loaded = false;

  label_viz = NULL;
  
  m_singleSelect = false;
  m_singleDeselect = false;
  m_multipleSelect = false;

  object_id_property = new rviz::IntProperty( "Object ID", 0,
                               "ID of the object to be segmented.",
                                getPropertyContainer(), SLOT( updateObjectID() ), this );
}

LabelTool::~LabelTool()
{
  /*
  if (label_viz != NULL)
  {
      delete label_viz;
  }*/

  for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
  {
      it->second.clear();
  }

  m_goalFaces.clear();
  context_->getSceneManager()->destroyManualObject("ReferenceMesh");
  context_->getSceneManager()->destroyManualObject("SegmentedMesh");
  context_->getSceneManager()->destroyManualObject("SelectionBox");
  context_->getSceneManager()->destroySceneNode(scene_node);
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
void LabelTool::onInitialize()
{
  ROS_INFO("Call Init");

  initNode();
  initOGRE();

  label_viz = new LabelViz(this);
}

void LabelTool::initOGRE()
{
  scene_node = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();

  reference_mesh = context_->getSceneManager()->createManualObject("ReferenceMesh");
  reference_mesh->setDynamic(false);
  reference_mesh->setVisible(true);
  scene_node->attachObject(reference_mesh);

  reference_mesh_material = Ogre::MaterialManager::getSingleton().create("ReferenceMeshMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);
  reference_mesh_material->getTechnique(0)->removeAllPasses();
  reference_mesh_material->getTechnique(0)->createPass();
  reference_mesh_material->getTechnique(0)->getPass(0)->setAmbient( Ogre::ColourValue(reference_color_r, reference_color_g, reference_color_b, reference_color_a) );
  reference_mesh_material->getTechnique(0)->getPass(0)->setDiffuse(0,0,0,reference_color_a);

  //if (reference_color_a < 1.0)
  {
    reference_mesh_material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    reference_mesh_material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  }
  reference_mesh_material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_SOLID);
  reference_mesh_material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);

  segment_mesh = context_->getSceneManager()->createManualObject("SegmentedMesh");
  segment_mesh->setDynamic(false);
  scene_node->attachObject(segment_mesh);

  segment_mesh_material = Ogre::MaterialManager::getSingleton().create("SegmentMatrial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);
  segment_mesh_material->getTechnique(0)->removeAllPasses();
  segment_mesh_material->getTechnique(0)->createPass();
  segment_mesh_material->getTechnique(0)->getPass(0)->setAmbient( Ogre::ColourValue(segment_color_r, segment_color_g, segment_color_b, segment_color_a) );
  segment_mesh_material->getTechnique(0)->getPass(0)->setDiffuse(0,0,0, segment_color_a);

  //if (segment_color_a < 1.0)
  {
    segment_mesh_material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    segment_mesh_material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  }
  segment_mesh_material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_SOLID);
  segment_mesh_material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);

  selection_box = context_->getSceneManager()->createManualObject("SelectionBox");
  selection_box->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  selection_box->setUseIdentityProjection(true);
  selection_box->setUseIdentityView(true);
  selection_box->setQueryFlags(0);
  scene_node->attachObject(selection_box);

  selection_box_material = Ogre::MaterialManager::getSingleton().create("SelectionBoxMaterial",
                                                                        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                        true);
  selection_box_material->getTechnique(0)->removeAllPasses();
  selection_box_material->getTechnique(0)->createPass();
  selection_box_material->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(0,
                                                                                    0,
                                                                                    255,
                                                                                    0.5));
  selection_box_material->getTechnique(0)->getPass(0)->setDiffuse(0,0,0,0.5);
  selection_box_material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  selection_box_material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  selection_box_material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_SOLID);
  selection_box_material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
}

void LabelTool::initNode()
{
  get_object_geometries_client = n.serviceClient<semap_ros::GetObjectInstances>("get_object_instances");
  get_object_descriptions_client = n.serviceClient<semap_ros::GetObjectDescriptions>("get_object_descriptions");
  add_triangle_mesh_client = n.serviceClient<semap_ros::AddTriangleMesh3DModel>("add_triangle_mesh_3d_model");
  add_object_descriptions_client = n.serviceClient<semap_ros::AddObjectDescriptions>("add_object_descriptions");
  add_object_instances_client = n.serviceClient<semap_ros::AddObjectInstances>("add_object_instances");
  make_relative_client = n.serviceClient<semap_ros::UpdateObjectDescriptions>("make_relative3d");
  activate_objects_client = n.serviceClient<semap_env::ActivateObjects>("activate_objects");
  
  converter = lvr_ros_converter::LvrRosConverter();

  mesh_pub = n.advertise<mesh_msgs::TriangleMeshStamped>( "segment", 1, true);
  //id_sub = n.subscribe( "label_tool_id", 1, &LabelTool::idCallback, this);
   
}

//void idCallback(const std_msgs::Int32::ConstPtr& id)
//{
  //ROS_INFO("New ID %d", id->data);
//}

/*
bool LabelTool::loadObjectGeometries(std::vector<int> ids)
{
  semap_ros::GetObjectInstances get_objects;
  get_objects.request.ids = ids;

  if (get_object_geometries_client.call(get_objects))
  {
    ROS_INFO("Got Objects");

    for ( int i = 0; i < get_objects.response.objects.size(); i++ )
    {
      semap_msgs::ObjectInstance obj = get_objects.response.objects[i];
      ROS_INFO(" loaded %s", obj.name.c_str() );

      for (int j = 0; j < obj.description.geometries.trianglemesh3d_models.size() ; j++ )
      {
        ROS_INFO(" has model %s", obj.absolute.geometries.trianglemesh3d_models[j].type.c_str() );
        if(obj.absolute.geometries.trianglemesh3d_models[j].type == "Body" )
        {
          absolute_reference_mesh = obj.absolute.geometries.trianglemesh3d_models[j].geometry;
          ROS_INFO("absolute mesh has %d verts %d faces", absolute_reference_mesh.vertices.size(), absolute_reference_mesh.triangles.size() );
          converter.removeDuplicates(absolute_reference_mesh);
          ROS_INFO("absolute mesh has %d verts %d faces", absolute_reference_mesh.vertices.size(), absolute_reference_mesh.triangles.size() );
          //setReferenceMesh(absolute_reference_mesh);
        }

        if(obj.description.geometries.trianglemesh3d_models[j].type == "Body" )
        {
          relative_reference_mesh = obj.description.geometries.trianglemesh3d_models[j].geometry;
          ROS_INFO("relative mesh has %d verts %d faces", relative_reference_mesh.vertices.size(), relative_reference_mesh.triangles.size() );
          converter.removeDuplicates(relative_reference_mesh);
          ROS_INFO("relative mesh has %d verts %d faces", relative_reference_mesh.vertices.size(), relative_reference_mesh.triangles.size() );
          setReferenceMesh(relative_reference_mesh);
        }

      }
    }

    size_t numSections = reference_mesh->getNumSections();

    ROS_INFO("ref mesh has %d sections", numSections);

    return true;
  }
  else
  {
    ROS_ERROR("Failed to get objects");
    return false;
  }
}*/

bool LabelTool::loadObjectGeometries(std::vector<int> ids)
{
  semap_ros::GetObjectDescriptions get_objects;
  get_objects.request.ids = ids;

  if (get_object_descriptions_client.call(get_objects))
  {
    //ROS_INFO("Got Objects");

    for ( int i = 0; i < get_objects.response.descriptions.size(); i++ )
    {
      semap_msgs::ObjectDescription obj = get_objects.response.descriptions[i];
      ROS_INFO(" loaded %s", obj.type.c_str() );

      for (int j = 0; j < obj.geometries.trianglemesh3d_models.size() ; j++ )
      {
        ROS_INFO(" has model %s", obj.geometries.trianglemesh3d_models[j].type.c_str() );

        if(obj.geometries.trianglemesh3d_models[j].type == "Body" )
        {
          relative_reference_mesh = obj.geometries.trianglemesh3d_models[j].geometry;
          ROS_INFO("relative mesh has %d verts %d faces", relative_reference_mesh.vertices.size(), relative_reference_mesh.triangles.size() );
          converter.removeDuplicates(relative_reference_mesh);
          //ROS_INFO("relative mesh has %d verts %d faces", relative_reference_mesh.vertices.size(), relative_reference_mesh.triangles.size() );
          setReferenceMesh(relative_reference_mesh);
        }
      }
    }

    size_t numSections = reference_mesh->getNumSections();

    //ROS_INFO("ref mesh has %d sections", numSections);

    return true;
  }
  else
  {
    ROS_ERROR("LabelTool failed to load objects. Please make sure the SEMAP DB Services are running.");
    return false;
  }
}

bool LabelTool::addObjectGeometry(int id, string type, mesh_msgs::TriangleMesh mesh)
{
  semap_msgs::TriangleMesh3DModel model;
  model.type = type;
  model.geometry = mesh;

  semap_ros::AddTriangleMesh3DModel add_model;
  add_model.request.id = id;
  add_model.request.model = model;

  if ( add_triangle_mesh_client.call(add_model) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool LabelTool::createObjectDescription(string object_type, string geometry_type, mesh_msgs::TriangleMesh mesh)
{
  semap_msgs::ObjectDescription desc = semap_msgs::ObjectDescription();
  desc.type = object_type;
  
  semap_ros::AddObjectDescriptions add_description;
  add_description.request.descriptions.push_back( desc );

  if ( add_object_descriptions_client.call( add_description ) )
  {
    semap_msgs::TriangleMesh3DModel model;
    model.type = geometry_type;
    model.geometry = mesh;

    semap_ros::AddTriangleMesh3DModel add_model;
    add_model.request.id = add_description.response.ids[0];
    add_model.request.model = model;

    if ( add_triangle_mesh_client.call(add_model) )
    {
      semap_msgs::ObjectInstance inst = semap_msgs::ObjectInstance();
      inst.description.id = add_description.response.ids[0];
      inst.pose.header.frame_id = "world";

      semap_ros::AddObjectInstances add_instance;
      add_instance.request.objects.push_back( inst );

      if ( add_object_instances_client.call( add_instance ) )
      {
         semap_ros::UpdateObjectDescriptions make_relative;
         make_relative.request.ids.push_back( add_model.response.id );

         if ( make_relative_client.call( make_relative ) )
         {
            semap_env::ActivateObjects activate_objects;
            activate_objects.request.ids = add_instance.response.ids;
            if ( activate_objects_client.call( activate_objects ) )
            {
            }
            else return false;
         }
      } else return false;
    }
    else
    {
      return false;
    }

  }
  else
  {
    return false;
  }

}

void LabelTool::updateObjectID()
{
  object_id = object_id_property->getInt();
  object_loaded = false;
  clearSelection();
  activate();
}

void LabelTool::activate()
{
  std::vector<int> ids;
  ids.push_back(object_id);

  if( !object_loaded )
  {
    if( loadObjectGeometries( ids ) )
    {
      ROS_INFO("LabelTool sucessfully loaded geoms");
      object_loaded = true;
    }
    else
    {
      ROS_INFO("LabelTool could not loaded geoms");
    }
  }
}

void LabelTool::deactivate()
{
}

void LabelTool::setReferenceMesh( mesh_msgs::TriangleMesh mesh )
{
  clearSelection();
  reference_mesh->begin("ReferenceMeshMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  for ( size_t i = 0; i < mesh.vertices.size(); i++ )
  {
    reference_mesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
  }

  for ( size_t i = 0; i < mesh.triangles.size(); i++ )
  {
    reference_mesh->triangle( mesh.triangles[i].vertex_indices[0], mesh.triangles[i].vertex_indices[1],  mesh.triangles[i].vertex_indices[2] );
  }

  reference_mesh->end();
}

void LabelTool::getSegmentMesh(mesh_msgs::TriangleMesh &mesh) //size_t goalSection, std::string regionLabel
{
  size_t numSections = segment_mesh->getNumSections();
  //ROS_INFO("Mesh has %d sections, we only take the first", numSections);

  size_t vertexCount;
  Ogre::Vector3* vertices;
  size_t indexCount;
  unsigned long* indices;

  if(numSections > 0)
  {
    getRawManualObjectData(segment_mesh, 0, vertexCount, vertices, indexCount, indices);
    //ROS_INFO("#vert %d #ind %d", vertexCount, indexCount );

    geometry_msgs::Point vertex;
    mesh_msgs::TriangleIndices index;
    for ( size_t i = 0; i < vertexCount; i++ )
    {
      vertex.x = vertices[i].x;
      vertex.y = vertices[i].y;
      vertex.z = vertices[i].z;
      mesh.vertices.push_back(vertex);
    }

    for ( size_t i = 0; i < indexCount; i+=3 )
    {
      index.vertex_indices[0] = indices[i];
      index.vertex_indices[1] = indices[i+1];
      index.vertex_indices[2] = indices[i+2];
      mesh.triangles.push_back(index);
    }
  }
}

/*
void LabelTool::getSegmentMesh(mesh_msgs::TriangleMesh &mesh) //size_t goalSection, std::string regionLabel
{
  size_t numSections = segment_mesh->getNumSections();
  ROS_INFO("Mesh has %d sections, we only take the first", numSections);

  size_t vertexCount;
  Ogre::Vector3* vertices;
  size_t indexCount;
  unsigned long* indices;

  if(numSections > 0)
  {
    getRawManualObjectData(segment_mesh, 0, vertexCount, vertices, indexCount, indices);
    ROS_INFO("#vert %d #ind %d", vertexCount, indexCount );

    std::map<std::vector<float>, unsigned int> vertexMap;
    unsigned int pos;
    for ( size_t i = 0; i < indexCount; i+=3 )
    {
      mesh_msgs::TriangleIndices triangle;
      std::cout << "triangle_indices " << indices[i + 0] << " "  << indices[i + 1] << " " << indices[i + 2] << " " << std::endl;

      for (int j = 0; j < 3; j++)
      {
         std::cout << relative_reference_mesh.vertices.size() << std::endl;
          std::vector<float> vertex;
          vertex.push_back( relative_reference_mesh.vertices[ indices[i + j] ].x );
          vertex.push_back( relative_reference_mesh.vertices[ indices[i + j] ].y );
          vertex.push_back( relative_reference_mesh.vertices[ indices[i + j] ].z );
          std::cout << "vertex " << vertex[0] << " "  << vertex[1] << " " << vertex[2] << " " << std::endl;

          if( vertexMap.find( vertex ) != vertexMap.end() )
          {
            std::cout << "exists" << std::endl;
            pos = vertexMap[ vertex ];
          }
          else
          {
            pos = mesh.vertices.size();

            geometry_msgs::Point v;
            v.x = relative_reference_mesh.vertices[ indices[i + j] ].x;
            v.y = relative_reference_mesh.vertices[ indices[i + j] ].y;
            v.z = relative_reference_mesh.vertices[ indices[i + j] ].z;

            mesh.vertices.push_back(v);
            vertexMap.insert( pair<std::vector<float>, unsigned int>( vertex, pos ) ) ;
            std::cout << "insert" << std::endl;
        }

        triangle.vertex_indices[j] = pos;
      }
      std::cout << "triangle_indices " << triangle.vertex_indices[0] << " "  << triangle.vertex_indices[1] << " " << triangle.vertex_indices[2] << " " << std::endl;

      mesh.triangles.push_back(triangle);
    }
  }
}
*/
/*

std::map<lvr::Vertex<float>, unsigned int> vertexMap;
  size_t pos;
  int index;
  //std::cout << "OLD" << std::endl << std::endl << std::endl;

  for( int i = 0; i < old_numIndices ; i++ )
  {
    //std::cout << "face #" << i << std::endl;

    for (int j = 0; j < 3; j++)
    {
      index = old_indexBuffer[ 3 * i + j ];
      /*std::cout << index;
      std::cout << " | ";
      std::cout << old_vertexBuffer[  3 * index ] << " ";
      std::cout << old_vertexBuffer[  3 * index + 1 ] << " ";
      std::cout << old_vertexBuffer[  3 * index + 2 ] << " " << std::endl;* s/

      lvr::Vertex<float> vertex =
      lvr::Vertex<float>( old_vertexBuffer[  3 * index ],
                          old_vertexBuffer[  3 * index + 1],
                          old_vertexBuffer[  3 * index + 2] );

      if( vertexMap.find( vertex ) != vertexMap.end() )
      {
        pos = vertexMap[ vertex ];

        //std::cout << "is old at " << pos << std::endl;
      }
      else
      {
        pos = new_vertexBuffer.size() / 3;
        new_vertexBuffer.push_back( vertex[0] );
        new_vertexBuffer.push_back( vertex[1] );
        new_vertexBuffer.push_back( vertex[2] );

        vertexMap.insert( pair<lvr::Vertex<float>, unsigned int>( vertex, pos ) );
        //std::cout << "is new at " << pos << std::endl;
      }

      new_indexBuffer.push_back( pos );
    }
*/


void LabelTool::clearSelection()
{
  segment_mesh->clear();
  for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
  {
    it->second.clear();
  }
  m_goalFaces.clear();
}

bool LabelTool::areFacesSelected()
{
  return (m_goalFaces.size() > 0);
}

void LabelTool::updateSelectionBox()
{
  float left, right, top, bottom;

  left = m_selectionStart.x * 2 - 1;
  right = m_selectionStop.x * 2 - 1;
  top = 1 - m_selectionStart.y * 2;
  bottom = 1 - m_selectionStop.y * 2;

  selection_box->clear();
  selection_box->begin("SelectionBoxMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  selection_box->position(left, top, -1);
  selection_box->position(right, top, -1);
  selection_box->position(right, bottom, -1);
  selection_box->position(left, bottom, -1);
  selection_box->triangle(0, 1, 2);
  selection_box->triangle(0, 2, 3);
  selection_box->end();
}

void LabelTool::updateSelectionMesh()
{
  size_t facesSize = 0;
  size_t vertexCount = 0;
  Ogre::Vector3* vertices;
  size_t indexCount = 0;
  unsigned long* indices;
  segment_mesh->clear();
  Ogre::ManualObject* mesh = context_->getSceneManager()->getManualObject("ReferenceMesh");
  segment_mesh->begin("SegmentMatrial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
  {
      getRawManualObjectData(mesh, it->first, vertexCount, vertices, indexCount, indices);
      facesSize += it->second.size();

      for (size_t j = 0; j < it->second.size(); j++)
      {
        segment_mesh->position(vertices[indices[it->second[j]]].x,
                                  vertices[indices[it->second[j]]].y,
                                  vertices[indices[it->second[j]]].z);
        segment_mesh->position(vertices[indices[it->second[j] + 1]].x,
                                  vertices[indices[it->second[j] + 1]].y,
                                  vertices[indices[it->second[j] + 1]].z);
        segment_mesh->position(vertices[indices[it->second[j] + 2]].x,
                                  vertices[indices[it->second[j] + 2]].y,
                                  vertices[indices[it->second[j] + 2]].z);
      }
      delete[] vertices;
      delete[] indices;
  }

  for (size_t j = 0; j < facesSize; j++)
  {
      segment_mesh->triangle(3 * j, 3 * j + 2, 3 * j + 1);
  }
  segment_mesh->end();
}

// Handling key events to label marked faces or to get db structure
int LabelTool::processKeyEvent(QKeyEvent *event, rviz::RenderPanel* panel)
{
  if (event->key() == Qt::Key_P)
  {
    mesh_msgs::TriangleMeshStamped mesh;
    getSegmentMesh(mesh.mesh);
    mesh.header.frame_id = "world";
    mesh.header.stamp = ros::Time::now();
    mesh_pub.publish( mesh );
  }
  
  // if 'n' is pressed start QTWidget labviz to get Label information
  if (event->key() == Qt::Key_N)
  {
    label_viz->exec();
    if ( label_viz->getStatus() )
    {

      if(label_viz->getModus() == "Add")
      {
        mesh_msgs::TriangleMesh mesh;
        getSegmentMesh(mesh);
        addObjectGeometry(object_id, label_viz->getSegmentName(), mesh);
      }
      else if(label_viz->getModus() == "New Object")
      {
        mesh_msgs::TriangleMesh mesh;
        getSegmentMesh(mesh);
        createObjectDescription(label_viz->getObjectName(), label_viz->getSegmentName(), mesh);
      }
    }
    else
    {
      ROS_INFO("do nothing" );
    }

  }

  // if 'r' is pressed clear the current selection of faces
  if (event->key() == Qt::Key_R)
  {
    clearSelection();
  }

  if (event->key() == Qt::Key_T)
  {
    reference_mesh->setVisible( !reference_mesh->isVisible() );
    segment_mesh->setVisible( !segment_mesh->isVisible() );
  }

  return Render;
}

// Handling mouse event and mark the clicked faces
int LabelTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{

  if (event.leftDown() && event.control())
  {
      selectConnectedFaces(event);
  }

  else if (event.rightDown() && event.control())
  {
      deselectConnectedFaces(event);
  }

  //else if (event.leftDown() && event.shift())
  //{
  //    m_multipleSelect = true;
  //    selectionBoxStart(event);
  //}

  //else if (event.leftUp() && m_multipleSelect)
  //{
      //m_multipleSelect = false;
      //selectMultipleFaces(event);
  //}

  //else if (event.rightDown() && event.shift())
  //{
      //m_multipleSelect = true;
      //selectionBoxStart(event);
  //}

  //else if (event.rightUp() && m_multipleSelect)
  //{
      //m_multipleSelect = false;
      //deselectMultipleFaces(event);
  //}

  //else if (m_multipleSelect)
  //{
      //selectionBoxMove(event);
  //}

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

// test whether a single ray intersects with the reference mesh
// by checking if the reference mesh is within the num_results closest targets
// num_results must be >= 2 to allow labeling direct sight, higher allows to label trough objects... unsure if that is desirable
bool LabelTool::singleRayQuery(rviz::ViewportMouseEvent& event, int num_results, Ogre::Ray& ray)
{
  ray = event.viewport->getCamera()->getCameraToViewportRay((float) event.x / event.viewport->getActualWidth(),
                                                            (float) event.y / event.viewport->getActualHeight());
  Ogre::RaySceneQuery* query = context_->getSceneManager()->createRayQuery(ray, Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
  query-> setSortByDistance(true, num_results);
  query->execute();
  Ogre::RaySceneQueryResult &results = query->getLastResults();

  for (size_t i = 0; i < results.size(); i++)
  {

    Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(results[i].movable);
    //ROS_INFO("%s", mesh->getName().c_str() );
    if (mesh->getName().find("ReferenceMesh") != std::string::npos)
    {
      return true;
    }
  }
  return false;
}

void LabelTool::selectSingleFace(rviz::ViewportMouseEvent& event)
{
  Ogre::Ray ray;
  size_t goalSection = -1 ;
  size_t goalIndex = -1;

  if ( singleRayQuery( event, num_results, ray) )
  {
    getIdentityOfSingleFace(reference_mesh, ray, goalSection, goalIndex);

    if (goalIndex != -1)
    {
      if (m_goalFaces.find( goalSection ) == m_goalFaces.end())
      {
        std::vector<size_t> faces;
        m_goalFaces.insert( std::pair<size_t, std::vector<size_t> >( goalSection, faces ) );
      }
      m_goalFaces[goalSection].push_back(goalIndex);
      std::sort( m_goalFaces[goalSection].begin(), m_goalFaces[goalSection].end() );
      m_goalFaces[goalSection].erase( std::unique( m_goalFaces[goalSection].begin(),
                                                   m_goalFaces[goalSection].end() ),
                                                   m_goalFaces[goalSection].end() );
      updateSelectionMesh();
    }
  }
}

void LabelTool::deselectSingleFace(rviz::ViewportMouseEvent& event)
{
  Ogre::Ray ray;
  size_t goalSection = -1 ;
  size_t goalIndex = -1;

  if ( singleRayQuery( event, num_results, ray) )
  {
    getIdentityOfSingleFace(reference_mesh, ray, goalSection, goalIndex);

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

/*
void LabelTool::selectMultipleFaces(rviz::ViewportMouseEvent& event)
{
  selection_box->setVisible(false);

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
  Ogre::SceneQueryResult& Result = sceneQuery->execute();
  Ogre::SceneQueryResultMovableList::iterator iter;
  for (iter = Result.movables.begin(); iter != Result.movables.end(); ++iter)
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
                      goalSection =  boost::lexical_cast<int>( faceMesh->getName().substr(0, position) );
                      position = faceMesh->getName().find_first_of('_', position + 1);
                      goalIndex =  boost::lexical_cast<int>( faceMesh->getName().substr(position + 1) );
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

void LabelTool::deselectMultipleFaces(rviz::ViewportMouseEvent& event)
{
  selection_box->setVisible(false);

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
  Ogre::SceneQueryResult& Result = sceneQuery->execute();
  Ogre::SceneQueryResultMovableList::iterator iter;
  for (iter = Result.movables.begin(); iter != Result.movables.end(); ++iter)
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
            goalSection = boost::lexical_cast<int>(faceMesh->getName().substr(0, position));
            position = faceMesh->getName().find_first_of('_', position + 1);
            goalIndex = boost::lexical_cast<int>(faceMesh->getName().substr(position + 1));
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
*/

void LabelTool::selectConnectedFaces(rviz::ViewportMouseEvent& event)
{
  Ogre::Ray ray;
  size_t goalSection = -1 ;
  size_t goalIndex = -1;

  if ( singleRayQuery( event, num_results, ray) )
  {
    getIdentityOfSingleFace(reference_mesh, ray, goalSection, goalIndex);

    if (goalIndex != -1)
    {
      size_t vertexCount = 0;
      Ogre::Vector3* vertices;
      size_t indexCount = 0;
      unsigned long* indices;
      std::vector<int> usedFaces;
      usedFaces.push_back(goalIndex);
      getRawManualObjectData(reference_mesh, goalSection, vertexCount, vertices, indexCount, indices);
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

void LabelTool::deselectConnectedFaces(rviz::ViewportMouseEvent& event)
{
  Ogre::Ray ray;
  size_t goalSection = -1 ;
  size_t goalIndex = -1;

  if ( singleRayQuery( event, num_results, ray) )
  {
    getIdentityOfSingleFace(reference_mesh, ray, goalSection, goalIndex);

    if (goalIndex != -1)
    {
      size_t vertexCount = 0;
      Ogre::Vector3* vertices;
      size_t indexCount = 0;
      unsigned long* indices;
      std::vector<int> usedFaces;
      usedFaces.push_back(goalIndex);
      getRawManualObjectData(reference_mesh, goalSection, vertexCount, vertices, indexCount, indices);
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
/*
void LabelTool::selectionBoxStart(rviz::ViewportMouseEvent& event)
{
  m_selectionStart.x = (float) event.x / event.viewport->getActualWidth();
  m_selectionStart.y = (float) event.y / event.viewport->getActualHeight();
  m_selectionStop = m_selectionStart;
  selection_box->clear();
  selection_box->setVisible(true);
}

void LabelTool::selectionBoxMove(rviz::ViewportMouseEvent& event)
{
  m_selectionStop.x = (float) event.x / event.viewport->getActualWidth();
  m_selectionStop.y = (float) event.y / event.viewport->getActualHeight();
  updateSelectionBox();
}
*/
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
    if(indexCount != 0)
    {
      for (size_t j = 0; j < indexCount; j += 3)
      {
        std::pair<bool, Ogre::Real> goal =
            Ogre::Math::intersects(
              ray,
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
            scene_node->attachObject(meshCopy);
            meshCopy->begin("SegmentMatrial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
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
                scene_node->_update(false, false);
                return;
            }
        }
        delete [] indices;
        delete [] vertices;
        m_startOfPickingFace = 0;
    }
    scene_node->_update(false, false);
    m_pickingFinished = true;
}

void LabelTool::stopPickingMode()
{
    Ogre::SceneNode::ObjectIterator iterator = scene_node->getAttachedObjectIterator();
    while (iterator.hasMoreElements())
    {
        Ogre::MovableObject* face = static_cast<Ogre::MovableObject*>(iterator.getNext());
        if (face->getName().find("SegmentedMesh") == std::string::npos && face->getName().find("SelectionBox") == std::string::npos)
        {
            scene_node->detachObject(face);
            scene_node->getCreator()->destroyMovableObject(face);
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
PLUGINLIB_EXPORT_CLASS( rviz_semap_plugin::LabelTool, rviz::Tool )
