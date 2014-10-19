#include <QApplication>

#include "kr_viz/qglviewer/camera_viewer.hpp"
#include "kr_viz/qglviewer/camera.hpp"

int main(int argc, char** argv) {
  // Read command lines arguments.
  QApplication application(argc, argv);

  // Instantiate the viewer.
  kr::viz::CameraViewer viewer;
  kr::viz::Camera* cam = new kr::viz::Camera();
  viewer.addCamera(cam);

  //  generate simple texture in a cv mat
  cv::Mat dummy(50,100,CV_8UC3);
  struct RGB {
    uchar r,g,b;
  } __attribute__((packed));
  for (int i=0; i < dummy.rows; i++) {
    for (int j=0; j < dummy.cols; j++) {
      RGB& rgb = dummy.at<RGB>(i,j);
      rgb.r = static_cast<uchar>(i*255.0 / dummy.rows);
      rgb.g = 0;
      rgb.b = static_cast<uchar>(j*255.0 / dummy.cols);
    }
  }
  cam->setImage(dummy);

  viewer.setWindowTitle("camera");

  // Make the viewer window visible on screen.
  viewer.show();

  // Run main loop.
  return application.exec();
}
