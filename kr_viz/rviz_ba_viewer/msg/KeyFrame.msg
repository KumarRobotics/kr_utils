# Occurence of a KeyFrame in a bundle-adjustment graph.
# The 'id' field is expected to be unique per-graph.
# Both image and cinfo must be included if image previews are desired.

# Header
# When this keyframe is included in a BA-graph, the frame_id
# is ignored in favour of the graph's frame_id.
std_msgs/Header header

uint32 id                              # Key frame id, must be unique.

geometry_msgs/PoseWithCovariance pose  # Pose of this key frame.

sensor_msgs/Image image                # Image seen from this keyframe.
sensor_msgs/CameraInfo cinfo           # Camera info.

uint32[] point_ids                     # IDs of features visible in this keyframe.
