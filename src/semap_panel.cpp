#include <stdio.h>


#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

#include "semap_panel.h"

namespace rviz_semap_plugin
{

SemapPanel::SemapPanel( QWidget* parent ) : rviz::Panel( parent )
{
  initNode();

  QVBoxLayout* layout = new QVBoxLayout;

  choice = new ObjectChoice(this);
  //layout->addWidget( choice );

  btn_set_objects = new QPushButton(this);
  btn_set_objects->setText("Set Objects...");
  connect( btn_set_objects, SIGNAL( clicked() ), this, SLOT( set_objects() ) );
  //layout->addWidget( btn_set_objects );

  btn_activate_all = new QPushButton(this);
  btn_activate_all->setText("Activate All...");
  connect( btn_activate_all, SIGNAL( clicked() ), this, SLOT( activate_all() ) );
  layout->addWidget( btn_activate_all );
  
  btn_deactivate_all = new QPushButton(this);
  btn_deactivate_all->setText("Deactivate All...");
  connect( btn_deactivate_all, SIGNAL( clicked() ), this, SLOT( deactivate_all() ) );
  layout->addWidget( btn_deactivate_all );

  btn_show_all = new QPushButton(this);
  btn_show_all->setText("Show All...");
  connect( btn_show_all, SIGNAL( clicked() ), this, SLOT( show_all() ) );
  layout->addWidget( btn_show_all );

  //btn_unshow_all = new QPushButton(this);
  //btn_unshow_all->setText("Unshow All...");
  //connect( btn_unshow_all, SIGNAL( clicked() ), this, SLOT( unshow_all() ) );
  //layout->addWidget( btn_activate_all );

  setLayout( layout );

}

void SemapPanel::initNode()
{
  n = ros::NodeHandle();
  activate_all_srv = n.serviceClient<semap_env::ActivateAllObjects>("activate_all_objects");
  deactivate_all_srv = n.serviceClient<semap_env::DeactivateAllObjects>("deactivate_all_objects");
  show_all_srv = n.serviceClient<semap_env::ActivateAllObjects>("show_all_objects");
  get_object_instances_list_srv = n.serviceClient<semap_ros::GetObjectInstancesList>("get_object_instances_list");
}

void SemapPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void SemapPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

void SemapPanel::set_objects( )
{
  ROS_INFO("Get Objects...");
  semap_ros::GetObjectInstancesList srv;
  srv.request.types.push_back("Table");
  get_object_instances_list_srv.call(srv);
  ROS_INFO("%d objects found", srv.response.objects.size() );
  std::map<std::string, int> test;
  for (int i = 0; i< srv.response.objects.size(); i++)
  {
    //ROS_INFO("Insert %s %d",  srv.response.objects[i].name.c_str(), srv.response.objects[i].id);
    test.insert( std::pair<std::string, int>( srv.response.objects[i].name, srv.response.objects[i].id ) );
  }
   choice->setComboBoxes(test);
}

void SemapPanel::activate_all( )
{
  ROS_INFO("Activate all objects...");
  semap_env::ActivateAllObjects srv;
  activate_all_srv.call(srv);
}

void SemapPanel::deactivate_all( )
{
  ROS_INFO("Deactivate all objects...");
  semap_env::DeactivateAllObjects srv;
  deactivate_all_srv.call(srv);
}

void SemapPanel::show_all( )
{
  ROS_INFO("Show all objects...");
  semap_env::ActivateAllObjects srv;
  show_all_srv.call(srv);
}

void SemapPanel::unshow_all( )
{
  ROS_INFO("Unshows all objects...");
}

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_semap_plugin::SemapPanel,rviz::Panel )
