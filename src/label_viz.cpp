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
 *	Author: Tristan Igelbrink <tigelbri@uos.de>
 *
 */

#include <label_viz.h>
namespace rviz
{

LabelViz::LabelViz(LabelTool* labelTool)
{
    // Set given labeltool
    m_labelTool = labelTool;

    // Construct and layout labels and button
    QLabel* faceLabel = new QLabel("Label selected faces:");
    QPushButton* labelButton = new QPushButton("Label Faces");
    QPushButton* cancelButton = new QPushButton("Cancel");

    QLabel* roomLabel = new QLabel("Room: ");
    QLabel* regionLabel = new QLabel("Region: ");
    QLabel* tableLabel = new QLabel("Table: ");

    m_roomEdit = new QComboBox();
    m_regionEdit = new QComboBox();
    m_tableEdit = new QComboBox();

    m_regionEdit->setEditable(true);
    m_tableEdit->setEditable(true);

    QGridLayout* controlsLayout = new QGridLayout();
    controlsLayout->addWidget(faceLabel, 0, 0, 1, 2);
    controlsLayout->addWidget(roomLabel, 1, 0);
    controlsLayout->addWidget(m_roomEdit, 1, 1);
    controlsLayout->addWidget(regionLabel, 2, 0);
    controlsLayout->addWidget(m_regionEdit, 2, 1);
    controlsLayout->addWidget(tableLabel, 3, 0);
    controlsLayout->addWidget(m_tableEdit, 3, 1);
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
    m_roomEdit->clear();
    m_regionEdit->clear();
    m_tableEdit->clear();

    m_regionList.clear();
    m_tableList.clear();
    setComboBoxes();
    QDialog::exec();
}

void LabelViz::setComboBoxes()
{
  /*
    // get Rooms from DB
    if(!m_labelTool->getDBPanel()->getDBManager()->getAllRoomIds(m_roomMap))
    {
        LogDialog* dia = new LogDialog();
        dia->exec();
        return;
    }

    if (m_labelTool->areFacesSelected())
    {
        std::vector<std::string> roomIDs;
        m_labelTool->getSelectedRooms(roomIDs);
        for (size_t i = 0; i < roomIDs.size(); i++)
        {
            m_roomEdit->addItem(QString::fromStdString(m_roomMap.left.at(roomIDs[i])));
        }
    }

    m_regionEdit->addItem(QString::fromStdString(""));
    m_tableEdit->addItem(QString::fromStdString(""));
    */
}

void LabelViz::labelButtonClicked()
{
  /*
    if (m_labelTool->areFacesSelected())
    {
        std::string roomLabel = m_roomEdit->currentText().toStdString();
        lvr_ros::Mesh meshMsg;
        size_t goalSection = m_labelTool->getGoalsectionFromRoom(m_roomMap.right.at(roomLabel));
        m_labelTool->getSelectedFaces(goalSection, m_regionEdit->currentText().toStdString(), meshMsg);
        if (m_regionEdit->currentText().toStdString().compare("") != 0)
        {
            postgis_control::Region region;
            region.id = m_labelTool->getDBPanel()->getDBManager()->insertRegion(m_roomMap.right.at(roomLabel));
            region.label = m_regionEdit->currentText().toStdString();
            if (!m_labelTool->getDBPanel()->getDBManager()->updateRegion(region.id, region))
            {
                //TODO: Fehlerbehandlung
                return;
             }

            if (!m_labelTool->getDBPanel()->getDBManager()->labelFacesOfRegion(region.id,
                                                                               meshMsg,
                                                                               m_labelTool->getDBPanel()->getfacesDBIndicesMap(goalSection)))
            {
                //TODO: Fehlerbehandlung
                return;
            }
        }

        else if (m_tableEdit->currentText().toStdString().compare("") != 0)
        {
            postgis_control::Table table;
            table.id = m_labelTool->getDBPanel()->getDBManager()->insertTable(m_roomMap.right.at(roomLabel));
            table.label = m_tableEdit->currentText().toStdString();
            if (!m_labelTool->getDBPanel()->getDBManager()->updateTable(table.id, table))
            {
                //TODO: Fehlerbehandlung
                return;
             }

            if (!m_labelTool->getDBPanel()->getDBManager()->labelFacesOfTable(table.id,
                                                                               meshMsg,
                                                                               m_labelTool->getDBPanel()->getfacesDBIndicesMap(goalSection)))
            {
                //TODO: Fehlerbehandlung
                return;
            }
        }
    }
    m_labelTool->clearSelection();
    this->hide();
    */
}

void LabelViz::cancelButtonClicked()
{
	this->hide();
}
}
