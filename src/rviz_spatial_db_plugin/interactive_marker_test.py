#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest("rviz_spatial_db_plugin")
import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin

from object_description_marker import *

server = None
marker_pos = 0
menu_handler = MenuHandler()
br = None
counter = 0

h_first_entry = 0
h_mode_last = 0

object_description = None
int_marker = None
model_visibility = {}

class GeometryModelVisu:
  def __init__(self):  
    self.type = None
    self.entry = None
    self.

def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame" )
    counter += 1

def enableCb( feedback ):
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState( handle )

    if state == MenuHandler.CHECKED:
        menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
        rospy.loginfo("Hiding first menu entry")
        menu_handler.setVisible( h_first_entry, False )
    else:
        menu_handler.setCheckState( handle, MenuHandler.CHECKED )
        rospy.loginfo("Showing first menu entry")
        menu_handler.setVisible( h_first_entry, True )

    menu_handler.reApply( server )
    rospy.loginfo("update")
    server.applyChanges()

def setTitle(entry_id, title):
   menu_handler.entry_contexts_[entry_id].title = title

def lockCb( feedback ):
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState( handle )

    if state == MenuHandler.CHECKED:
      menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
      rospy.loginfo("Unlock Object")
      menu_handler.setVisible(h_move, False)
      setTitle(h_lock, "Lock Object") 
  
    else:
      menu_handler.setCheckState( handle, MenuHandler.CHECKED )
      rospy.loginfo("Lock Object")
      menu_handler.setVisible( h_move, True )
      setTitle(h_lock, "Unlock Object")

    menu_handler.reApply( server )
    server.applyChanges()

def modeCb(feedback):
    global h_mode_last
    menu_handler.setCheckState( h_mode_last, MenuHandler.UNCHECKED )
    h_mode_last = feedback.menu_entry_id
    menu_handler.setCheckState( h_mode_last, MenuHandler.CHECKED )

    rospy.loginfo("Switching to menu entry #" + str(h_mode_last))
    menu_handler.reApply( server )
    print "DONE"
    server.applyChanges()

def deepCb( feedback ):
    rospy.loginfo("The deep sub-menu has been found.")

def initMenu():
    global h_first_entry, h_mode_last
    global h_lock, h_move
    global geometry_menu_handle
    
    h_lock = menu_handler.insert( "Unlock", callback=lockCb )

    h_move = menu_handler.insert( "Move Object" )
    menu_handler.setVisible(h_move, False)
    move_xy_entry = menu_handler.insert( "XY", parent = h_move)
    move_myz_entry = menu_handler.insert( "XYZ", parent = h_move)
    geometry_menu_handle = menu_handler.insert( "Show" )

def initGeometryMenu():
  for key in models:
    menu_handler.setCheckState( menu_handler.insert( "Show First Entry", callback=enableCb ), MenuHandler.CHECKED )
    menu_handler.insert( "Show " + key, parent = geometry_menu_handle)

    menu_handler.setVisible(h_move, models[key])

def createGeometryModelDictionary(desc): 
  for model_visibility in desc.point2d_models:
    model_visibility[model.type] = true
  for model in desc.pose2d_models:
    model_visibility[model.type] = true
  for model in desc.polygon2d_models:
    model_visibility[model.type] = true
  print models
  #for model in desc.point3d_models:
    #array.markers.append(create_point_marker(model.geometry))
    #pose = ROSPose()
    #pose.position = model.geometry
    #array.markers.append(create_text_marker(model.type, pose))
  #for model in desc.pose3d_models:
    #array.markers.append(create_point_marker(model.pose.position))
    #array.markers.append(create_pose_marker(model.pose))
    #array.markers.append(create_text_marker(model.type, model.pose))
  #for model in desc.polygon3d_models:
    #array.markers.append(create_polygon_marker(model.geometry, model.pose))
    #array.markers.append(create_text_marker(model.type, model.pose))
  #for model in desc.trianglemesh3d_models:
     #array.markers.append(create_mesh_marker(model.geometry, model.pose))
     #array.markers.append(create_text_marker(model.type, model.pose))
  #for model in desc.polygonmesh3d_models:
    #array.markers.append(create_text_marker(model.type, model.pose))
    #for polygon in model.geometry.polygons:
      #poly = create_polygon_marker(polygon, model.pose)
      #poly.color.r = 0.0
      #poly.color.g = 1.0
      #poly.color.b = 1.0
      #poly.color.a = 1.0
      #poly.ns = 'polymesh'
      #array.markers.append(poly)  

def processFeedback( feedback ):
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
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    
    server.applyChanges()

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

#####################################################################
# Marker Creation

def create6DOFControl(controls, fixed):
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS#
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

def createMenuControl(controls):
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    controls.append(control)

def createInteractiveMarker():

    int_marker = InteractiveMarker()
    int_marker.name = 'my_interactive_marker'
    int_marker.header.frame_id = '/base_link'
    int_marker.scale = 0.5
    int_marker.description = 'this is a test'

    control = InteractiveMarkerControl()
    control.name = "button"
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    control.markers.append( create_description_name_marker(object_description) )
    int_marker.controls.append(control)
    
    control = InteractiveMarkerControl()
    control.name = "visu"
    control.description = "this is sparta"
    control.interaction_mode = InteractiveMarkerControl.NONE
    control.always_visible = True
    control.markers = create_object_description_marker(object_description).markers
    int_marker.controls.append(control)

    create6DOFControl(int_marker.controls, True)
    print '6dof',len(int_marker.controls)
    return int_marker

def listControls():
    for control in int_marker.controls:
      print control.name

if __name__=="__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()
    rospy.Timer(rospy.Duration(0.1), frameCallback)

    server = InteractiveMarkerServer("basic_controls")

    object_description = create_object_description()
    createGeometryModelDictionary(object_description)
    
    initMenu()
    initGeometryMenu()
    int_marker = createInteractiveMarker()
    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )
    server.applyChanges()

    rospy.spin()
