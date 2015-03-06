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
 * importer_widget.h
 *
 * Author: Henning Deeken
 *
 */

#ifndef MESH_IMPORTER_WIDGET_H
#define MESH_IMPORTER_WIDGET_H

#include <string>
#include <vector>
#include <iostream>

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QFileDialog>
#include <QTreeView>

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>

#include <../src/mesh_utility.cpp>

/**
 *
 *@class MeshImporterWidget
 *
 *@brief QTreeWidget which handles the import of scans
 *
 */

namespace rviz_spatial_db_plugin
{

class MeshImporterWidget : public QDialog
{

Q_OBJECT
public:

MeshImporterWidget( QWidget* parent = 0);

virtual ~MeshImporterWidget();

bool importMesh(std::string path);


//bool (lvr_ros::Mesh& mesh, string floor_label, string room_label);

private:
  // checkbox for converting to ros format
  QCheckBox* convert_to_ros;
  QLabel* path_label;
  std::string path;

  private Q_SLOTS:

void importButtonClicked();

void chooseButtonClicked();
};

}
#endif
