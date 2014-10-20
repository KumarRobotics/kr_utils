# This message contains information in a key frame

std_msgs/Header header                 # header
uint32 id                              # key frame id
geometry_msgs/PoseWithCovariance pose  # pose of this key frame
sensor_msgs/Image image                # image seen from this key frame
sensor_msgs/CameraInfo cinfo           # camera info
uint32[] point_ids                     # id of points in point cloud
