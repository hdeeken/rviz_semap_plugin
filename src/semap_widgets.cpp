#include <semap_widgets.h>

namespace rviz_semap_plugin
{

ObjectChoice::ObjectChoice(QWidget* parent)
{
  parent = parent;

  QLabel* label = new QLabel("Object: ");

  choice = new QComboBox();

  QGridLayout* controlsLayout = new QGridLayout();
  controlsLayout->addWidget(label , 1, 0);
  controlsLayout->addWidget(choice, 1, 1);
  setLayout(controlsLayout);

  // Make signal/slot connections.
  //connect(labelButton, SIGNAL(clicked()), this, SLOT(labelButtonClicked()));
  //connect(cancelButton, SIGNAL(clicked()), this, SLOT(cancelButtonClicked()));
}

ObjectChoice::~ObjectChoice()
{
}

void ObjectChoice::exec()
{
  QDialog::exec();
}

void ObjectChoice::setComboBoxes( std::map<std::string, int> choices )
{
  choice->clear();

  for( std::map<std::string, int>::iterator it = choices.begin(); it != choices.end(); it++)
  {
    //ROS_INFO("%s", it->first.c_str());
    choice->addItem( QString(it->first) );
  }
  
}
/*
void ObjectChoice::labelButtonClicked()
{
  status = true;
  modus = modi_edit->currentText().toStdString();
  object_name = object_name_edit->currentText().toStdString();
  segment_name = segment_name_edit->currentText().toStdString();
  this->hide();
}

void ObjectChoice::cancelButtonClicked()
{
  status = false;
  modus = "";
  object_name = "";
  segment_name = "";

  this->hide();
}*/
/*
bool ObjectChoice::getObjectID()
{
  return status;
}

std::string ObjectChoice::getObjectType()
{
  return object_name;
}*/


}
