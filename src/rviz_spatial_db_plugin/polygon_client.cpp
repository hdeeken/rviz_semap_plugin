#!/usr/bin/env python

"""
Interactive Description Marker
"""

import roslib; roslib.load_manifest("spatial_environment")
import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin

from rviz_spatial_db_plugin.object_description_marker import *
from geometry_msgs.msg import PoseStamped as ROSPoseStamped

from python_qt_binding.QtGui import QWidget

from spatial_db_ros.srv import *

#### UNABHAENGIG ###
def createTitleControl(controls, object_name):
    control = InteractiveMarkerControl()
    control.name = "Menu"
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    marker = create_title_marker(object_name)
    control.markers.append(marker)
    controls.append(control)

def createVisuControl(controls, obj, visu):
  #  print "createVisuCointrol"
    control = InteractiveMarkerControl()
    control.name = "VisuControl"
    control.description = "This is the objects visualization."
    control.interaction_mode = InteractiveMarkerControl.NONE
    control.always_visible = True
    markers = create_object_visualization_marker(obj, visu).markers
    control.markers = markers
    controls.append(control)

def updateVisuControl(controls, obj, visu):
    for control in controls:
      if control.name == "VisuControl":
        controls.remove(control)
    createVisuControl(controls, obj, visu)

 ### Motion Control

def createMotionControl(controls, fixed):
    control = InteractiveMarkerControl()
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.orientation.w = 1
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    if fixed:
      control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

def updateMotionControl(controls, fixed):
    for control in controls:
      if control.name == "MotionControl":
        controls.remove(control)
    createMotionControl(controls, fixed)

def createMenuControl(controls, object_name):
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.name = "Menu"
    control.description= object_name
    control.always_visible = True
    controls.append(control)

def listControls(controls):
    for control in controls:
      print control.name

## Util

def setTitle(menu_handler, entry_id, title):
    menu_handler.entry_contexts_[entry_id].title = title

####################
####################

def create_object_visualization_marker(obj, model_visu):

    array = MarkerArray()
    desc = obj.description

    for model in desc.point2d_models:
      pose = ROSPoseStamped()
      pose.header.frame_id = obj.name
      pose.pose.orientation.w = 1.0
      pose.pose.position.x = model.geometry.x
      pose.pose.position.y = model.geometry.y
      pose.pose.position.z = 0.0

      if model_visu[model.type].show_geo:
        geo_marker = create_point_marker("Point2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.pose2d_models:
   #   print 'pos2'
      pose = ROSPoseStamped()
      pose.header.frame_id = obj.name
      pose.pose.orientation.w = 1.0

      quat = quaternion_from_euler(0, 0, model.pose.theta)
      pose.pose.position.x = model.pose.x
      pose.pose.position.y = model.pose.y
      pose.pose.position.z = 0.0
      pose.pose.orientation.x = quat[0]
      pose.pose.orientation.y = quat[1]
      pose.pose.orientation.z = quat[2]
      pose.pose.orientation.w = quat[3]

      if model_visu[model.type].show_geo:
        geo_marker = create_pose_marker("Pose2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.polygon2d_models:
   #   print 'pol2'
      pose = ROSPoseStamped()
      pose.header.frame_id = obj.name
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_polygon_marker("Polygon2D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)
      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.point3d_models:
 #     print 'poi3'
      pose = ROSPoseStamped()
      pose.header.frame_id = obj.name
      pose.pose.orientation.w = 1.0

      pose.pose.position = model.geometry

      if model_visu[model.type].show_geo:
        geo_marker = create_point_marker("Point3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.pose3d_models:
#      print 'pos3'
      pose = ROSPoseStamped()
      pose.header.frame_id = obj.name
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_pose_marker("Pose3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)
    for model in desc.polygon3d_models:
#      print 'pol3'
      pose = ROSPoseStamped()
      pose.header.frame_id = obj.name
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_polygon_marker("Polygon3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)
    for model in desc.trianglemesh3d_models:
 #     print 'tri'
      pose = ROSPoseStamped()
      pose.header.frame_id = obj.name
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        geo_marker = create_mesh_marker("TriangleMesh3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, model.geometry)
        array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type)
        array.markers.append(text_marker)

    for model in desc.polygonmesh3d_models:
     # print 'poly'
      pose = ROSPoseStamped()
      pose.header.frame_id = obj.name
      pose.pose.orientation.w = 1.0

      pose.pose = model.pose

      if model_visu[model.type].show_geo:
        for polygon in model.geometry.polygons:
        #  print polygon
          geo_marker = create_polygon_marker("PolygonMesh3D", pose, model_visu[model.type].geo_color, model_visu[model.type].geo_scale, polygon)
          array.markers.append(geo_marker)

      if model_visu[model.type].show_text:
        text_marker = create_text_marker("Label", pose, model_visu[model.type].text_color, model_visu[model.type].text_scale, model.type, model_visu[model.type].text_offset)
        array.markers.append(text_marker)

    id = 0
    for m in array.markers:
      m.id = id
      id += 1

    return array

def lookupModelVisuConfig(desc):
  model_dict = {}

  for model in desc.point2d_models:
    type = model.type
    show_geo = True
    geo_color = [1.0, 0.0, 0.0, 1.0]
    geo_scale = [0.05, 0.05, 0.05]
    show_text = True
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)
  for model in desc.pose2d_models:
    type = model.type
    show_geo = True
    geo_color = [1.0, 1.0, 0.0, 1.0]
    geo_scale = [0.1, 0.025, 0.025]
    show_text = True
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)
  for model in desc.polygon2d_models:
    type = model.type
    show_geo = True
    geo_color = [0.0, 1.0, 1.0, 1.0]
    geo_scale = [0.05, 0.05, 0.05]
    show_text = True
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, show_geo, geo_color, geo_scale, show_text,text_color, text_scale, text_offset)
  for model in desc.point3d_models:
    type = model.type
    show_geo = True
    geo_color = [0.0, 0.0, 1.0, 1.0]
    geo_scale = [0.05, 0.05, 0.05]
    show_text = True
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)
  for model in desc.pose3d_models:
    type = model.type
    show_geo = True
    geo_color = [0.0, 0.0, 1.0, 1.0]
    geo_scale = [0.1, 0.025, 0.025]
    show_text = True
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, show_geo, geo_color, geo_scale, show_text,text_color, text_scale, text_offset)
  for model in desc.polygon3d_models:
    type = model.type
    show_geo = True
    geo_color = [0.0, 1.0, 0.0, 1.0]
    geo_scale = [0.05, 0.05, 0.05]
    show_text = True
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, show_geo, geo_color, geo_scale, show_text,text_color, text_scale, text_offset)
  for model in desc.trianglemesh3d_models:
    type = model.type
    show_geo = True
    geo_color = [0.0, 0.5, 0.5, 1.0]
    geo_scale = [1.0, 1.0, 1.0]
    show_text = True
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.1, 0.1, 0.1]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)
  for model in desc.polygonmesh3d_models:
    type = model.type
    show_geo = True
    geo_color = [0.5, 1.0, 0.5, 1.0]
    geo_scale = [0.05, 0.05, 0.05]
    show_text = True
    text_color = [1.0, 1.0, 1.0, 1.0]
    text_scale = [0.05, 0.05, 0.05]
    text_offset = [0.0, 0.0, 0.15]
    model_dict[model.type] = ModelVisu(type, show_geo, geo_color, geo_scale, show_text, text_color, text_scale, text_offset)

  return model_dict

def switchCheckState(menu_handler, handle):
  state = menu_handler.getCheckState( handle )

  if state == MenuHandler.CHECKED:
    print menu_handler.getTitle(handle), 'unchecked'
    menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
  else:
    print menu_handler.getTitle(handle), 'checked'
    menu_handler.setCheckState( handle, MenuHandler.CHECKED )

class ModelVisu:
  type = None
  show_geo = False
  geo_color = []
  geo_scale = []
  show_text = False
  text_color = []
  text_scale = []
  text_offset = []

  def __init__(self, type, \
               show_geo = False, \
               geo_color = [0,0,0,1], \
               geo_scale = [0.2,0.2,0.2], \
               show_text = False, \
               text_color = [0,0,0,1], \
               text_scale = [0.1,0.1,0.1], \
               text_offset = [0.0,0.0,0.25]):
    self.type = type
    self.show_geo = show_geo
    self.geo_color = geo_color
    self.geo_scale = geo_scale
    self.show_text = show_text
    self.text_color = text_color
    self.text_scale = text_scale
    self.text_offset = text_offset

class InteractiveDescriptionMarker():

  server = None
  obj = None
  marker = None

  model_visu = {}
  model_entry = {}
  menu_handler = None
  lock_menu_handle = None
  geometry_menu_handle = None
  movement_menu_handle = None

  current_pose = None

  def __init__(self, obj, server):
    self.obj = obj
    self.server = server
    self.menu_handler = MenuHandler()
    self.initMenus()
    self.createInteractiveMarker()

## Callbacks

  def processFeedback(self, feedback ):
      s = "Feedback from marker '" + feedback.marker_name
      s += "' / control '" + feedback.control_name + "'"

      mp = ""
      if feedback.mouse_point_valid:
          mp = " at " + str(feedback.mouse_point.x)
          mp += ", " + str(feedback.mouse_point.y)
          mp += ", " + str(feedback.mouse_point.z)
          mp += " in frame " + feedback.header.frame_id

      if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
          rospy.loginfo( s + ": button click" + mp + "." )
      elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
          rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
      elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
          rospy.loginfo( s + ": pose changed")
          rospy.loginfo( feedback.pose)
          self.current_pose = feedback.pose
      elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
          rospy.loginfo( s + ": mouse down" + mp + "." )
      elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
          rospy.loginfo( s + ": mouse up" + mp + "." )

      self.server.applyChanges()

  def lockCb(self, feedback):
      handle = feedback.menu_entry_id
      state = self.menu_handler.getCheckState( handle )

      if state == MenuHandler.CHECKED:
        self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED )
        self.menu_handler.setVisible(self.movement_menu_handle, False)
      else:
        self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
        self.menu_handler.setVisible(self.movement_menu_handle, True )

      self.menu_handler.reApply(self.server)
      self.server.applyChanges()

  def geometryCb(self, feedback):
    handle = feedback.menu_entry_id
    menu_handler = self.menu_handler
    server = self.server

    if menu_handler.getTitle(handle) in self.model_visu.keys():

      state = menu_handler.getCheckState( handle )

      if state == MenuHandler.CHECKED:
        menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
        rospy.loginfo("Uncheck %s" % menu_handler.getTitle(handle))
        self.model_visu[menu_handler.getTitle(handle)].show_geo = False
        self.model_visu[menu_handler.getTitle(handle)].show_text = False
      else:
        menu_handler.setCheckState( handle, MenuHandler.CHECKED )
        rospy.loginfo("Check %s" % menu_handler.getTitle(handle))
        self.model_visu[menu_handler.getTitle(handle)].show_geo = True
        self.model_visu[menu_handler.getTitle(handle)].show_text = True

    elif menu_handler.getTitle(handle) == "All":
      for key in self.model_visu.keys():
        self.model_visu[key].show_geo = True
        self.model_visu[key].show_text = True
        menu_handler.setCheckState( menu_handler.getTitle(key), MenuHandler.CHECKED )

    elif menu_handler.getTitle(handle) == "None":
      for key in self.model_visu.keys():
        self.model_visu[key].show_geo = False
        self.model_visu[key].show_text = False
        menu_handler.setCheckState( menu_handler.getTitle(key), MenuHandler.UNCHECKED )

    elif menu_handler.getTitle(handle) == "Inverted":
      print self.model_entry.keys()
      for key in self.model_entry.keys():
        handle = self.model_entry[key]
        state = menu_handler.getCheckState( handle)

        if state == MenuHandler.CHECKED:
          menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
          self.model_visu[menu_handler.getTitle(handle)].show_geo = False
          self.model_visu[menu_handler.getTitle(handle)].show_text = False
        else:
          menu_handler.setCheckState( handle, MenuHandler.CHECKED )
          self.model_visu[menu_handler.getTitle(handle)].show_geo = True
          self.model_visu[menu_handler.getTitle(handle)].show_text = True
    else:
      print menu_handler.getTitle(handle), 'was activated, warfoer?'

    menu_handler.reApply( server )
    updateVisuControl(self.marker.controls, self.obj, self.model_visu)
    server.applyChanges()

  def movementCb(self, feedback):
    menu_handler = self.menu_handler
    server = self.server

    switchCheckState(menu_handler, feedback.menu_entry_id)
    print 'updateMotionControl'
    menu_handler.reApply( server )
    updateMotionControl(self.marker.controls, True)
    server.applyChanges()

  def frameCb(self, feedback):
    app = QApplication( sys.argv )
    widget = ChangeReferenceWidget(self.obj.id)
    widget.resize( 50, 50 )
    widget.show()
    app.exec_()
    self.update()

  def update(self):
    print 'call UPIUPI'
    call_activate_object(self.obj.id)

  def saveCb(self, feedback):

    if self.current_pose != None:

      try:
        rospy.wait_for_service('update_transform')
        update_transform_call = rospy.ServiceProxy('update_transform', UpdateTransform)
        request = UpdateTransformRequest()
        request.id = self.obj.id
        request.pose = self.current_pose
        response = update_transform_call(request)
        rospy.loginfo('UpdateTransform service call succeeded!')
        #return True
      except rospy.ServiceException as e:
        return False, "UpdateTransform service call failed: %s" % e

    self.update()

  def resetCb(self, feedback):
    try:
      rospy.wait_for_service('set_transform')
      call = rospy.ServiceProxy('set_transform', SetTransform)
      request = SetTransformRequest()
      request.id = self.obj.id
      origin = ROSPoseStamped()
      origin.pose.orientation.w = 1.0
      print origin
      request.pose = origin.pose
      response = call(request)
      rospy.loginfo('SetTransform service call succeeded!')
      #return True
    except rospy.ServiceException as e:
      return False, "SetTransform service call failed: %s" % e

    self.update()

### Init Menus

  def initMenus(self):
    self.initLockMenu()
    self.initGeometryMenu()
    self.initMovementMenu()

  def initLockMenu(self):

    menu_handler = self.menu_handler

    status_ = menu_handler.insert("Status")

    #locked_ = menu_handler.insert("Locked", parent = status_, callback=self.lockCb)
    #menu_handler.setCheckState(locked_, MenuHandler.CHECKED )
    #menu_handler.setVisible(locked_, True)

    save_ = menu_handler.insert("Save", parent = status_, callback = self.saveCb)
    menu_handler.setVisible(save_, True)

    reset_ = menu_handler.insert("Reset To Origin", parent = status_, callback = self.resetCb)
    menu_handler.setVisible(reset_, True)

    frame_ = menu_handler.insert("Change Reference Frame", parent = status_, callback = self.frameCb)
    menu_handler.setVisible(frame_, True)

  def initGeometryMenu(self):
    self.geometry_menu_handle = self.menu_handler.insert("Show")
    self.menu_handler.setVisible(self.geometry_menu_handle, True)

    components_ = self.menu_handler.insert("Components", parent = self.geometry_menu_handle)

    self.model_visu = lookupModelVisuConfig(self.obj.description)
    for key in self.model_visu:
      self.model_entry[key]= self.menu_handler.insert(key, parent = components_, callback = self.geometryCb)
      self.menu_handler.setCheckState(self.model_entry[key], (self.model_visu[key].show_geo or self.model_visu[key].show_text))
      self.menu_handler.setVisible(self.model_entry[key], True)

    choose_ = self.menu_handler.insert("Choose", parent = self.geometry_menu_handle)

    all_ = self.menu_handler.insert("All", parent =choose_, callback = self.geometryCb)
    self.menu_handler.setVisible(all_, True)

    none_ = self.menu_handler.insert("None", parent = choose_, callback = self.geometryCb)
    self.menu_handler.setVisible(none_, True)
    invert_ = self.menu_handler.insert("Inverted", parent = choose_, callback = self.geometryCb)
    self.menu_handler.setVisible(invert_, True)

  def initMovementMenu(self):
    self.movement_menu_handle = self.menu_handler.insert( "Move" )

    #move_xy_entry = self.menu_handler.insert( "XY", parent = self.movement_menu_handle, callback = self.movementCb)
    #self.menu_handler.setCheckState(move_xy_entry, MenuHandler.UNCHECKED)
    #self.menu_handler.setVisible(move_xy_entry, True)
    move_xyz_entry = self.menu_handler.insert("XYZ", parent = self.movement_menu_handle, callback = self.movementCb)
    self.menu_handler.setCheckState(move_xyz_entry, MenuHandler.UNCHECKED)
    self.menu_handler.setVisible(move_xyz_entry, True)

  def createInteractiveMarker(self):
    self.marker = InteractiveMarker()

    self.marker.name = self.obj.name

    self.marker.header.frame_id = self.obj.name
    self.marker.scale = 1.0
    self.marker.description = "This is the interactive marker for object: " + self.obj.name

    createMenuControl(self.marker.controls, self.marker.name)
    createVisuControl(self.marker.controls, self.obj, self.model_visu)
    self.server.insert(self.marker, self.processFeedback)
    self.menu_handler.apply( self.server, self.marker.name )
    self.server.applyChanges()

### Debug Node

if __name__=="__main__":

    rospy.init_node("interactive_description_marker")
    server = InteractiveMarkerServer("interactive_description_marker")

    obj = create_object_instance()
    int_obj = InteractiveDescriptionMarker(obj, server)

    rospy.spin()
