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

  label_viz = NULL; //LabelViz( this );
  
  m_singleSelect = false;
  m_singleDeselect = false;
  m_multipleSelect = false;

  object_id = new IntProperty( "Object ID", 0,
                               "RViz will try to render this many frames per second.",
                                global_options_, SLOT( updateFps() ), this );
    

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

  segment_mesh = context_->getSceneManager()->createManualObject("SelectMesh");
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
  add_object_instances_client = n.serviceClient<semap_ros::AddObjectDescriptions>("add_object_descriptions");
  
  converter = lvr_ros_converter::LvrRosConverter();

  mesh_pub = n.advertise<mesh_msgs::TriangleMeshStamped>( "segment", 1, true);
}

bool LabelTool::loadObjectGeometries(std::vector<int> ids)
{
  semap_ros::GetObjectDescriptions get_objects;
  get_objects.request.ids = ids;

  if (get_object_descriptions_client.call(get_objects))
  {
    for ( int i = 0; i < get_objects.response.descriptions.size(); i++ )
    {
      semap_msgs::ObjectDescription obj = get_objects.response.descriptions[i];

      for (int j = 0; j < obj.geometries.trianglemesh3d_models.size() ; j++ )
      {
        if(obj.geometries.trianglemesh3d_models[j].type == "Body" )
        {
          relative_reference_mesh = obj.geometries.trianglemesh3d_models[j].geometry;
          converter.removeDuplicates(relative_reference_mesh);
          setReferenceMesh(relative_reference_mesh);
        }
      }
    }

    size_t numSections = reference_mesh->getNumSections();

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
      return true;
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

void LabelTool::activate()
{
  std::vector<int> ids;
  ids.push_back(object_id);

  if( !object_loaded )
  {
    if( loadObjectGeometries( ids ) )
    {
      object_loaded = true;
    }
  }
}

void LabelTool::deactivate()
{
}

void LabelTool::setReferenceMesh( mesh_msgs::TriangleMesh mesh )
{
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

void LabelTool::clearSelection()
{
  segment_mesh->clear();
  for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
  {
    it->second.clear();
  }
  m_goalFaces.clear();
}

int LabelTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if (event.leftDown())
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

} // end namespace rviz_label_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_semap_plugin::LabelTool, rviz::Tool )
