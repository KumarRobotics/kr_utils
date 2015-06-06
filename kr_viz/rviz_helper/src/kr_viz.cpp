#include <rviz_helper/kr_marker.hpp>
#include <rviz_helper/viz_manager.hpp>

namespace kr {
namespace viz {

// // // // // // // KR Marker // // // // // // // // // // // // // // //

int Marker::getUniqueId() {
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
    mark_.header.stamp = ros::Time::now();
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
Marker& Marker::position(const geometry_msgs::Point pose) {
    mark_.pose.position = pose;
    return *this;
}
Marker& Marker::scale(double s){
    return scale(s,s,s);
}
Marker& Marker::alpha(double a) {
  mark_.color.a = a;
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
MarkerArray MarkerArray::operator +=(const MarkerArray &array) {
    for(auto &mark:array.array_)
        array_.push_back(mark);
    return *this;
}
MarkerArray MarkerArray::operator +=(const Marker &marker) {
    array_.push_back(marker);
    return *this;
}
// // // // // // // Viz Manager // // // // // // // // // // // // // // //

VizManager::VizManager() : nh_(""), nh_priv_("~") {

}
VizManager & VizManager::instance() {
    static VizManager man;
    return man;
}
MarkerPubHandle::MarkerPubHandle(const std::string &topic):  manager_(VizManager::instance()), pub_(manager_.getPublisher(topic)) {
}
ros::Publisher& VizManager::getPublisher(const std::string &topic) {
    auto element = publishers_.find(topic);
    if(element == publishers_.end()) {
        publishers_[topic] = nh_priv_.advertise<visualization_msgs::MarkerArray>(topic.c_str(),10,this);
        return publishers_[topic];
    } else {
        return element->second;
    }

}
Marker::operator visualization_msgs::MarkerArray(){
    MarkerArray array;
    array.push_back(*this);
    return array;
}
MarkerArray::operator visualization_msgs::MarkerArray(){
    visualization_msgs::MarkerArray array;
    for(auto & mark: array_)
        array.markers.push_back(mark);
    return array;
}
void MarkerArray::push_back(const Marker &mark) {
    array_.push_back(mark);
}
MarkerArray::MarkerArray() {}
Marker& Marker::mesh(const std::string &resource) {
    mark_.mesh_resource = resource;
    mark_.type = visualization_msgs::Marker::MESH_RESOURCE;
    return *this;
}
Marker& Marker::type(int32_t tp) {
    mark_.type = tp;
    return *this;
}
Marker& Marker::point_push_back(const geometry_msgs::Point &pt) {
    mark_.points.push_back(pt);
    return *this;
}

MarkerArray::MarkerArray(double lx, double ly, double lz,
                         double ux, double uy, double uz, rviz::Color col) {
    kr::viz::Marker mark_lines;
    kr::viz::Marker mark_points;

    geometry_msgs::Point p000;
    geometry_msgs::Point p001;
    geometry_msgs::Point p010;
    geometry_msgs::Point p011;
    geometry_msgs::Point p100;
    geometry_msgs::Point p101;
    geometry_msgs::Point p110;
    geometry_msgs::Point p111;

    p000.x = lx; p000.y = ly; p000.z = lz;
    p001.x = lx; p001.y = ly; p001.z = uz;
    p010.x = lx; p010.y = uy; p010.z = lz;
    p011.x = lx; p011.y = uy; p011.z = uz;
    p100.x = ux; p100.y = ly; p100.z = lz;
    p101.x = ux; p101.y = ly; p101.z = uz;
    p110.x = ux; p110.y = uy; p110.z = lz;
    p111.x = ux; p111.y = uy; p111.z = uz;

    // edges
    mark_lines.type(visualization_msgs::Marker::LINE_LIST).color(col);
    mark_lines.point_push_back(p000);    mark_lines.point_push_back(p001);
    mark_lines.point_push_back(p000);    mark_lines.point_push_back(p010);
    mark_lines.point_push_back(p000);    mark_lines.point_push_back(p100);

    mark_lines.point_push_back(p001);    mark_lines.point_push_back(p101);
    mark_lines.point_push_back(p001);    mark_lines.point_push_back(p011);

    mark_lines.point_push_back(p010);    mark_lines.point_push_back(p110);
    mark_lines.point_push_back(p010);    mark_lines.point_push_back(p011);

    mark_lines.point_push_back(p100);    mark_lines.point_push_back(p110);
    mark_lines.point_push_back(p100);    mark_lines.point_push_back(p101);

    mark_lines.point_push_back(p011);    mark_lines.point_push_back(p111);
    mark_lines.point_push_back(p101);    mark_lines.point_push_back(p111);
    mark_lines.point_push_back(p110);    mark_lines.point_push_back(p111);

    // corners
    mark_points.type(visualization_msgs::Marker::SPHERE_LIST).color(col);
    mark_points.point_push_back(p000);
    mark_points.point_push_back(p001);
    mark_points.point_push_back(p010);
    mark_points.point_push_back(p011);
    mark_points.point_push_back(p100);
    mark_points.point_push_back(p101);
    mark_points.point_push_back(p110);
    mark_points.point_push_back(p111);

    mark_lines.scale(0.1);
    mark_points.scale(0.1);

    array_.push_back(mark_lines);
    array_.push_back(mark_points);
}

}  // namespace viz
}  // namespace kr

