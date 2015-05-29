#include <rviz_helper/kr_marker.hpp>
#include <rviz_helper/viz_manager.hpp>

namespace kr {
namespace viz {

// // // // // // // KR Marker // // // // // // // // // // // // // // //

Marker::getUniqueId() {
    static int id=0;
    return id++;
}

Marker::Marker(){
    //set default values
    mark_.color.a=1.0;
    mark_.color.r = kr::viz::colors::FUCHSIA.r_;
    mark_.color.g = kr::viz::colors::FUCHSIA.g_;
    mark_.color.b = kr::viz::colors::FUCHSIA.b_;

    mark_.pose.orientation.w = 1;
    mark_.ns = "viz";
    mark_.header.frame_id = "world";
    mark_.id = getUniqueId();
    mark_.action = visualization_msgs::Marker::ADD;

    mark_.scale.x = 1.0;
    mark_.scale.y = 1.0;
    mark_.scale.z = 1.0;

    mark_.type = visualization_msgs::Marker::SPHERE;
    mark_.header.stamp = ros::Time();
}



// Implementations
Marker& Marker::color(const std_msgs::ColorRGBA &col) {
    mark_.color = col;
    return *this;
}
Marker& Marker::color(const rviz::Color &col) {
    mark_.color.r = col.r_;
    mark_.color.b = col.b_;
    mark_.color.g = col.g_;
    return *this;
}
Marker& Marker::scale(const geometry_msgs::Vector3 &sca){
    mark_.scale = sca;
    return *this;
}
Marker& Marker::scale(double x, double y, double z){
    mark_.scale.x = x;
    mark_.scale.y = y;
    mark_.scale.z = z;
    return *this;
}

MarkerArray MarkerArray::operator +(const MarkerArray &array) {
    for(auto &mark:array.array_)
        array_.push_back(mark);
    return *this;
}
MarkerArray MarkerArray::operator +(const Marker &mark) {
    array_.push_back(mark);
    return *this;
}

// // // // // // // Viz Manager // // // // // // // // // // // // // // //

VizManager::VizManager() : nh_(""), nh_priv_("~") {

}
VizManager & VizManager::instance() {
    static VizManager man;
    return man;
}
MarkerPubHandle::MarkerPubHandle():  manager_(VizManager::instance()) {
}



}  // namespace viz
}  // namespace kr

