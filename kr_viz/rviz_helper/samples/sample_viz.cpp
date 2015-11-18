#include <ros/ros.h>
#include <rviz_helper/viz_manager.hpp>

class SampleViz{
public:
    SampleViz();
    void update();
private:
    ros::NodeHandle nh_;

    kr::viz::MarkerPubHandle mpub_;

    kr::viz::Marker mark;
};
SampleViz::SampleViz() : nh_("~"), mpub_("viz") {

}
void SampleViz::update() {
    mark.color(kr::viz::colors::EGGSHELL).scale(2.0,1.0,0.5);
    mpub_.publish(mark);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample_viz");
    ros::NodeHandle nh("~");

    SampleViz sample_viz;

    ros::Rate r(1.0);

    while(nh.ok()) {
        sample_viz.update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

