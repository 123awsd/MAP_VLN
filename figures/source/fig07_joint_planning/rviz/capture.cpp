#include <QApplication>
#include <QTimer>
#include <ros/ros.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/yaml_config_reader.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreSceneManager.h>
#include <iostream>
int main(int argc, char** argv) {
  ros::init(argc, argv, "paper_rviz_capture", ros::init_options::AnonymousName);
  QApplication app(argc, argv);
  rviz::RenderPanel panel;
  rviz::VisualizationManager manager(&panel);
  panel.initialize(manager.getSceneManager(), &manager);
  manager.initialize();
  rviz::YamlConfigReader reader; rviz::Config config;
  reader.readFile(config, QString::fromUtf8(argv[1]));
  manager.load(config.mapGetChild("Visualization Manager"));
  manager.getSceneManager()->setAmbientLight(Ogre::ColourValue(0.18,0.18,0.18));
  manager.startUpdate();
  panel.resize(1800,1200); panel.setWindowTitle("Paper MINCO preview"); panel.show();
  QTimer::singleShot(9000, [&]() {
    panel.getRenderWindow()->writeContentsToFile(argv[2]);
    std::cout << "Captured native OGRE framebuffer" << std::endl;
    if (argc > 3) app.quit();
  });
  return app.exec();
}
