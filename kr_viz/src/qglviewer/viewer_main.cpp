#include <QApplication>
#include "kr_viz/qglviewer/camera_viewer.hpp"

int main(int argc, char** argv) {
  // Read command lines arguments.
  QApplication application(argc, argv);

  // Instantiate the viewer.
  kr::viz::CameraViewer viewer;

  viewer.setWindowTitle("camera");

  // Make the viewer window visible on screen.
  viewer.show();

  // Run main loop.
  return application.exec();
}
