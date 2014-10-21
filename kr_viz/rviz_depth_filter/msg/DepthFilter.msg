# DepthFilter.msg

# Each point is drawn as a line parallel to the
# vector from origin to position, with a length
# given by the uncertainty.

# Header. frame_id is used to transform the entire cloud.
std_msgs/Header header

# Viewpoints
geometry_msgs/Point[] origins

# Position of features
geometry_msgs/Point[] positions

# Standard deviations (uncertainty in depth)
float64[] sigmas
