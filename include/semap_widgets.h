#ifndef SEMAP_WIDGETS_H
#define SEMAP_WIDGETS_H

#include <QDialog>
#include <QLabel>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QString>

#include <iostream>
#include <string>
#include <vector>
#include <map>

namespace rviz_semap_plugin

{
class ObjectChoice: public QDialog
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
ObjectChoice(QWidget* parent = 0);

/**
 *
 *@brief destructor
 *
 */
virtual ~ObjectChoice();

/**
 *
 *@brief overwritten exec-method
 *
 */
virtual void exec();

/*
bool getStatus();
std::string getModus();
std::string getObjectName();
std::string getSegmentName();*/

void setComboBoxes(std::map<std::string,int> choices);

private Q_SLOTS:

/*
/ **
 *
 *@brief function which is called when the 'label' button is pressed
 *
 * QSlot function for dumping the labels into the database
 *
 * /
void labelButtonClicked();

/**
 *
 *@brief function which is called when the 'cancel' button is pressed
 *
 * Q Slot function for cancelling the procedure
 *
 * /
void cancelButtonClicked();
*/

private:


QComboBox* choice;

//std::string modus;
//std::string object_name;
//std::string segment_name;

};
}
#endif
