/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef SEMAP_PANEL_H
#define SEMAP_PANEL_H

#include <ros/ros.h>

#include <rviz/panel.h>

#include <semap_widgets.h>

#include <semap_ros/GetObjectInstancesList.h>
#include <semap_env/ActivateAllObjects.h>
#include <semap_env/DeactivateAllObjects.h>

class QLineEdit;
class QPushButton;

namespace rviz_semap_plugin
{

//class DriveWidget;

class SemapPanel: public rviz::Panel
{

Q_OBJECT
public:

  SemapPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:

protected Q_SLOTS:

  void set_objects( );
  void activate_all( );
  void deactivate_all( );
  void show_all( );
  void unshow_all( );

protected:

  ros::NodeHandle n;

private:

  void initNode();

  ObjectChoice* choice;

  QPushButton* btn_set_objects;
  QPushButton* btn_activate_all;
  QPushButton* btn_deactivate_all;
  QPushButton* btn_show_all;
  QPushButton* btn_unshow_all;

  ros::ServiceClient activate_all_srv;
  ros::ServiceClient deactivate_all_srv;
  ros::ServiceClient show_all_srv;
  ros::ServiceClient get_object_instances_list_srv;

};

} // end namespace

#endif
