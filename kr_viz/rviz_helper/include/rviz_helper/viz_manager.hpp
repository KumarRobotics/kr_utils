#ifndef VIZ_MANAGER_HPP
#define VIZ_MANAGER_HPP

#include <ros/ros.h>
//#include <visualization_msgs/MarkerArray.h>
#include <rviz_helper/kr_marker.hpp>
#include <map>

namespace kr {
namespace viz {

class VizManager {
public:
    static VizManager & instance();
private:
    VizManager();
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    std::map<std::string,ros::Publisher> publishers;
};



class MarkerPubHandle{
public:
    MarkerPubHandle();
    void publish();
private:
    VizManager &manager_;
};

}  // namespace viz
}  // namespace kr



#endif // VIZ_MANAGER_HPP
