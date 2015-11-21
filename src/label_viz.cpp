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
 * label_viz.cpp
 *
 *  Author: Tristan Igelbrink <tigelbri@uos.de>
 *
 */

#include <label_viz.h>

namespace rviz_semap_plugin
{

LabelViz::LabelViz(LabelTool* labelTool)
{
  label_tool = labelTool;

  QLabel* head_label = new QLabel("Store Segment:");

  QPushButton* labelButton = new QPushButton("Store");
  QPushButton* cancelButton = new QPushButton("Cancel");

  QLabel* modi_label = new QLabel("Modi: ");

  modi_edit = new QComboBox();

  QLabel* segment_name_edit_label = new QLabel("Segment Name:");
  segment_name_edit = new QComboBox();
  segment_name_edit->setEditable(true);

  QLabel* object_name_edit_label = new QLabel("Object Name:");
  object_name_edit = new QComboBox();
  object_name_edit->setEditable(true);

  QGridLayout* controlsLayout = new QGridLayout();
  controlsLayout->addWidget(head_label, 0, 0, 1, 2);
  controlsLayout->addWidget(modi_label, 1, 0);
  controlsLayout->addWidget(modi_edit, 1, 1);
  controlsLayout->addWidget(segment_name_edit_label, 2, 0);
  controlsLayout->addWidget(segment_name_edit, 2, 1);
  controlsLayout->addWidget(object_name_edit_label, 3, 0);
  controlsLayout->addWidget(object_name_edit, 3, 1);
  controlsLayout->addWidget(labelButton, 4, 0);
  controlsLayout->addWidget(cancelButton, 4, 1);

  // Set the top-level layout for this widget.
  setLayout(controlsLayout);

  // Make signal/slot connections.
  connect(labelButton, SIGNAL(clicked()), this, SLOT(labelButtonClicked()));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(cancelButtonClicked()));
}

LabelViz::~LabelViz()
{
}

void LabelViz::exec()
{
  setComboBoxes();
  QDialog::exec();
}

void LabelViz::setComboBoxes()
{
  modi_edit->clear();
  //modi_edit->addItem( "Split" );
  //modi_edit->addItem( "Remove" );
  modi_edit->addItem( "New Object" );  
  modi_edit->addItem( "Add" );

  segment_name_edit->clear();
  object_name_edit->clear();
}

void LabelViz::labelButtonClicked()
{

  status = true;
  modus = modi_edit->currentText().toStdString();
  object_name = object_name_edit->currentText().toStdString();
  segment_name = segment_name_edit->currentText().toStdString();
  this->hide();
}

void LabelViz::cancelButtonClicked()
{
  status = false;
  modus = "";
  object_name = "";
  segment_name = "";

  this->hide();
}

bool LabelViz::getStatus()
{
  return status;
}

std::string LabelViz::getModus()
{
  return modus;
}

std::string LabelViz::getObjectName()
{
  return object_name;
}

std::string LabelViz::getSegmentName()
{
  return segment_name;
}

}
