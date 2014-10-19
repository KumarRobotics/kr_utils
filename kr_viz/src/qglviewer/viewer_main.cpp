#include <QApplication>

#include "kr_viz/qglviewer/camera_viewer.hpp"
#include "kr_viz/qglviewer/camera.hpp"

int main(int argc, char** argv) {
  // Read command lines arguments.
  QApplication application(argc, argv);

  // Instantiate the viewer.
  kr::viz::CameraViewer viewer;
  kr::viz::Camera* cam = new kr::viz::Camera();

  viewer.set_camera(cam);

  viewer.setWindowTitle("camera");

  // Make the viewer window visible on screen.
  viewer.show();

  // Run main loop.
  return application.exec();
}
