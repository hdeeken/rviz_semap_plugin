#ifndef POLYGON_TOOL_H
#define POLYGON_TOOL_H
#include <rviz/tool.h>
#include <rviz/ogre_helpers/billboard_line.h>
namespace Ogre
{
class SceneNode;
class Vector3;
}
namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}
namespace rviz_polygon_tool
{

class PolygonTool: public rviz::Tool
{
Q_OBJECT
public:
PolygonTool();
~PolygonTool();

virtual void onInitialize();
virtual void activate();
virtual void deactivate();
virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

private:

};

} 
#endif 
