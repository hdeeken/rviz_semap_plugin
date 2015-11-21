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
 * label_viz.h
 *
 *  Author: Tristan Igelbrink <tigelbri@uos.de>
 *          Johannes Heitmann <joheitma@uos.de>
 *          Marcel Mrozinski  <mmronzs@uos.de
 *
 */

#ifndef LABEL_VIZ_H
#define LABEL_VIZ_H

#include <QDialog>
#include <QLabel>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QString>

#include <iostream>
#include <string>
#include <vector>

#include <label_tool.h>

/**
 *
 *@class LabelViz
 *
 *@brief QTreeWidget which handles labeling of given regions rooms etc
 *
 * this class implements a qtwidget which dumps the marked faces with
 * a new label into the database
 *
 */

namespace rviz_semap_plugin

{
class LabelTool;
class LabelViz: public QDialog
{

Q_OBJECT
public:

/**
 *
 *@brief constructor
 *
 *@param label_tool parent labelTool which is connected to this viz
 *
 */
LabelViz(LabelTool* label_tool = 0);

/**
 *
 *@brief destructor
 *
 */
virtual ~LabelViz();

/**
 *
 *@brief overwritten exec-method
 *
 */
virtual void exec();

bool getStatus();
std::string getModus();
std::string getObjectName();
std::string getSegmentName();

private Q_SLOTS:

/**
 *
 *@brief function which is called when the 'label' button is pressed
 *
 * QSlot function for dumping the labels into the database
 *
 */
void labelButtonClicked();

/**
 *
 *@brief function which is called when the 'cancel' button is pressed
 *
 * Q Slot function for cancelling the procedure
 *
 */
void cancelButtonClicked();

private:

/**
 *
 *@brief fills the QtComboBoxes with the room, in which the selected
 *        faces are
 *
 */
void setComboBoxes();

LabelTool* label_tool;
QComboBox* modi_edit;
QComboBox* segment_name_edit;
QComboBox* object_name_edit;

bool status;
std::string modus;
std::string object_name;
std::string segment_name;

};
}
#endif
