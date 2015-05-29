#ifndef KR_MARKER_HPP_
#define KR_MARKER_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz_helper/colors.hpp>

namespace kr {
namespace viz {

// Defines visualization marker with better default syntax and coloring

class Marker {
public:
    Marker();
    Marker(const visualization_msgs::Marker &mark) {mark_ = mark;}
    operator visualization_msgs::Marker() {return mark_;}

    // set marker properties using chained structure:
    // ex: mahkah.color(kr::viz::colors::Pink).frame_id("sim")
    Marker& color(const rviz::Color &col);
    Marker& color(const std_msgs::ColorRGBA &col);
    Marker& scale(const geometry_msgs::Vector3 &sca);
    Marker& scale(double x, double y, double z);
private:
    int getUniqueId();
;
private:
    visualization_msgs::Marker mark_;
}; // class Marker

class MarkerArray {
public:
    MarkerArray();
    MarkerArray(const visualization_msgs::MarkerArray &array);

    MarkerArray operator+(const MarkerArray &array);
    MarkerArray operator+(const Marker &marker);

    operator visualization_msgs::MarkerArray();

    void push_back(const Marker &mark);
    Marker & at(int pos);
    const Marker & at(int pos) const;
private:
    std::vector<Marker> array_;
};




}  // namespace viz
}  // namespace kr





#endif // KR_MARKER_HPP
