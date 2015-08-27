#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <ros/console.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include <polygon_tool.h>

namespace rviz_polygon_tool
{

  PolygonTool::PolygonTool()
  {
    shortcut_key_ = 'p';
  }

  PolygonTool::~PolygonTool()
  {
    for( unsigned i = 0; i < flag_nodes_.size(); i++ )
    {
      scene_manager_->destroySceneNode( flag_nodes_[ i ]);
    }
  }

  void PolygonTool::onInitialize()
  {
   
  }

  void PolygonTool::activate()
  {
   
  }

  void PolygonTool::deactivate()
  {
   
  }

  int PolygonTool::processMouseEvent( rviz::ViewportMouseEvent& event )
  {
    if( !moving_flag_node_ )
    {
      return Render;
    }
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
    ground_plane,
    event.x, event.y, intersection ))
    {
      moving_flag_node_->setVisible( true );
      moving_flag_node_->setPosition( intersection );
      current_flag_property_->setVector( intersection );
      if( event.leftDown() )
      {
        makeFlag( intersection );
        current_flag_property_ = NULL;
        return Render | Finished;
      }
    }
    else
    {
      moving_flag_node_->setVisible( false ); 
    }
    return Render;
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_tool::PolygonTool, rviz::Tool )
