# A feature or point in a bundle-adjustment graph.

uint32 id                         # Unique ID of the point.
geometry_msgs/Point32 position    # World-frame position of the point.

uint32 keyframe_id                # ID of the reference keyframe.
float64 depth_sigma               # Standard deviation on depth of the feature,
                                  # in the reference keyframe.