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
    ros::Publisher& getPublisher(const std::string &topic);
private:
    VizManager();
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    std::map<std::string,ros::Publisher> publishers_;

};



class MarkerPubHandle{
public:
    MarkerPubHandle(const std::string &topic);



//    template <typename M>
//    void publish(const M& message) const {
//          pub_.publish(message);
//    }
    // this should be non-generic
    void publish(const visualization_msgs::MarkerArray& message) const {
          pub_.publish(message);
    }
private:
    VizManager &manager_;
    ros::Publisher &pub_;
};



}  // namespace viz
}  // namespace kr



#endif // VIZ_MANAGER_HPP
