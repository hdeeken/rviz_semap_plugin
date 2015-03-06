def create_object_description():
  point2dmodel = Point2DModel()
  point2dmodel.type = '2dpoint'
  point2dmodel.geometry.x = -0.5
  point2dmodel.geometry.y = 0.0
  point2dmodel.geometry.z = 0.0

  pose2dmodel = Pose2DModel()
  pose2dmodel.type = '2dpose'
  pose2dmodel.pose = ROSPose2D()
  pose2dmodel.pose.x = 0.5
  pose2dmodel.pose.y = 1.5
  pose2dmodel.pose.theta = 1.5

  point0 = ROSPoint32(0, 0, 0)
  point1 = ROSPoint32(1, 0, 0)
  point2 = ROSPoint32(1, 1, 0)
  point3 = ROSPoint32(0, 1, 0)
  point4 = ROSPoint32(-1, 0, 0.5)
  point5 = ROSPoint32(-1, 1, 0.5)

  point6 = ROSPoint32(1, 1, 0)
  point7 = ROSPoint32(2, 1, 0)
  point8 = ROSPoint32(2, 2, 0)
  point9 = ROSPoint32(1, 2, 0)
  point10 = ROSPoint32(-2, 1, 0)
  point11 = ROSPoint32(-2, 2, 0)

  polygon0 = ROSPolygon()
  polygon0.points.append(point0)
  polygon0.points.append(point1)
  polygon0.points.append(point2)
  polygon0.points.append(point3)

  polygon1 = ROSPolygon()
  polygon1.points.append(point4)
  polygon1.points.append(point0)
  polygon1.points.append(point3)
  polygon1.points.append(point5)

  polygon2 = ROSPolygon()
  polygon2.points.append(point6)
  polygon2.points.append(point7)
  polygon2.points.append(point8)
  polygon2.points.append(point9)

  polygon3 = ROSPolygon()
  polygon3.points.append(point10)
  polygon3.points.append(point6)
  polygon3.points.append(point9)
  polygon3.points.append(point11)

  polygon2dmodel = Polygon2DModel()
  polygon2dmodel.type = '2dpolygon'
  polygon2dmodel.pose = ROSPose()
  polygon2dmodel.geometry = polygon0

  point3dmodel = Point3DModel()
  point3dmodel.type = '3dpoint'
  point3dmodel.geometry.x = 2.0
  point3dmodel.geometry.y = 2.0
  point3dmodel.geometry.z = 1.0

  pose3dmodel = Pose3DModel()
  pose3dmodel.type = '3dpose'
  pose3dmodel.pose = ROSPose()
  pose3dmodel.pose.position.x = -0.5
  pose3dmodel.pose.position.y = -0.5
  pose3dmodel.pose.position.z = 0.5

  rand_quat = random_quaternion()
  pose3dmodel.pose.orientation.x = rand_quat[0]
  pose3dmodel.pose.orientation.y = rand_quat[1]
  pose3dmodel.pose.orientation.z = rand_quat[2]
  pose3dmodel.pose.orientation.w = rand_quat[3]

  polygon3dmodel = Polygon3DModel()
  polygon3dmodel.type = '3dpolygon'
  polygon3dmodel.pose = ROSPose()
  polygon3dmodel.pose.position.x = 2.0
  polygon3dmodel.geometry = polygon1

  tri0 = ROSMeshTriangle()
  tri1 = ROSMeshTriangle()
  tri0.vertex_indices[0] = 0
  tri0.vertex_indices[1] = 1
  tri0.vertex_indices[2] = 2

  tri1.vertex_indices[0] = 0
  tri1.vertex_indices[1] = 2
  tri1.vertex_indices[2] = 3

  tpoint0 = ROSPoint( 0.0, 0.0, 0.0)
  tpoint1 = ROSPoint( 1.0, 0.0, 1.0)
  tpoint2 = ROSPoint( 1.0, 1.0, 0.0)
  tpoint3 = ROSPoint( 0.0, 1.0, 1.0)
  tpoint4 = ROSPoint(-1.0, 0.0, 0.0)
  tpoint5 = ROSPoint(-1.0, 1.0, 1.0)

  trianglemesh3dmodel = TriangleMesh3DModel()
  trianglemesh3dmodel.type = '3dtrianglemesh'
  trianglemesh3dmodel.pose = ROSPose()
  rand_quat = random_quaternion()
  trianglemesh3dmodel.pose.position.x = 1.0
  trianglemesh3dmodel.pose.position.y = -2.0
  trianglemesh3dmodel.pose.position.z = 0.25
  #trianglemesh3dmodel.pose.orientation.x = rand_quat[0]
  #trianglemesh3dmodel.pose.orientation.y = rand_quat[1]
  #trianglemesh3dmodel.pose.orientation.z = rand_quat[2]
  #trianglemesh3dmodel.pose.orientation.w = rand_quat[3]
  trianglemesh3dmodel.geometry.vertices.append(tpoint0)
  trianglemesh3dmodel.geometry.vertices.append(tpoint1)
  trianglemesh3dmodel.geometry.vertices.append(tpoint2)
  trianglemesh3dmodel.geometry.vertices.append(tpoint3)
  trianglemesh3dmodel.geometry.triangles.append(tri0)
  trianglemesh3dmodel.geometry.triangles.append(tri1)

  polygonmesh3dmodel = PolygonMesh3DModel()
  polygonmesh3dmodel.type = '3dpolygonmesh'
  polygonmesh3dmodel.pose = ROSPose()
  rand_quat = random_quaternion()
  #polygonmesh3dmodel.pose.position.x = 0.5
  #polygonmesh3dmodel.pose.position.y = 1.0
  #polygonmesh3dmodel.pose.position.z = 0.0
  #polygonmesh3dmodel.pose.orientation.x = rand_quat[0]
  #polygonmesh3dmodel.pose.orientation.y = rand_quat[1]
  #polygonmesh3dmodel.pose.orientation.z = rand_quat[2]
  #polygonmesh3dmodel.pose.orientation.w = rand_quat[3]
  polygonmesh3dmodel.geometry.polygons.append(polygon2)
  polygonmesh3dmodel.geometry.polygons.append(polygon3)

  desc_ros = ROSObjectDescription()
  desc_ros.type = "test_description"
  desc_ros.point2d_models.append(point2dmodel)
  desc_ros.pose2d_models.append(pose2dmodel)
  desc_ros.polygon2d_models.append(polygon2dmodel)
  desc_ros.point3d_models.append(point3dmodel)
  desc_ros.pose3d_models.append(pose3dmodel)
  desc_ros.polygon3d_models.append(polygon3dmodel)
  desc_ros.trianglemesh3d_models.append(trianglemesh3dmodel)
  desc_ros.polygonmesh3d_models.append(polygonmesh3dmodel)

  return desc_ros

def create_object_instance():

  pose = ROSPoseStamped()
  pose.header.frame_id = "world"
  pose.pose.position.x = 0.0
  pose.pose.position.y = 0.0
  pose.pose.position.z = 0.0
  pose.pose.orientation.x = 0.0
  pose.pose.orientation.y = 0.0
  pose.pose.orientation.z = 0.0
  pose.pose.orientation.w = 1.0

  inst_ros = ROSObjectInstance()
  inst_ros.alias = "mr_objecto"
  inst_ros.pose = pose
  inst_ros.description = create_object_description()
  return inst_ros

'''
####
####
## Point
#  marker.scale.x = 0.05
#  marker.scale.y = 0.05
#  marker.scale.z = 0.05
#  marker.color.g = 1.0
#  marker.color.a = 1.0

## Polygon
#  marker.scale.x = 0.05
#  marker.scale.y = 0.05
#  marker.scale.z = 0.05
#  marker.color.r = 1.0
#  marker.color.a = 1.0

## Mesh
#  marker.scale.x = 1.0
#  marker.scale.y = 1.0
#  marker.scale.z = 1.0
#  marker.color.r = 1.0
#  marker.color.g = 1.0
#  marker.color.b = 0.0
#  marker.color.a = 1.0

## Text
#  marker.scale.x = 0.15
#  marker.scale.y = 0.15
#  marker.scale.z = 0.15
#  marker.color.b = 1.0
#  marker.color.a = 1.0

## POSE
#  marker.scale.x = 0.25
#  marker.scale.y = 0.05
#  marker.scale.z = 0.05
#  marker.color.b = 1.0
#  marker.color.a = 1.0
####
####
'''
