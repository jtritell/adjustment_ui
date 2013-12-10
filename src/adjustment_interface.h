#ifndef ADJUSTMENT_INTERFACE_H
#define ADJUSTMENT_INTERFACE_H

#include <QWidget>
#include <ros/ros.h>
#include "ui_AdjustmentInterface.h"
#include <QMainWindow>
#include <adjustment_localization/AdjustmentParameters.h>
#include <geometry_msgs/Quaternion.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "AdjustmentInterface" implements the top level widget for this example.
class AdjustmentInterface: public QMainWindow
{
Q_OBJECT
public:
  explicit AdjustmentInterface( QWidget* parent = 0 );
  void updateParameters(const adjustment_localization::AdjustmentParameters::ConstPtr& msg);
  ~AdjustmentInterface();
  void setTilt(const geometry_msgs::Quaternion::ConstPtr& msg);

private Q_SLOTS:
  void sendAdjustment();
  void sendTilt();
  void requestPointCloud();
  void resetAdjustment();
  void changeAdjustment(int adjustment);

private:
  Ui_adjustmentinterface *ui;
  rviz::VisualizationManager* manager_;
  ros::NodeHandle n_;
  ros::Publisher adjustment_pub_;
  ros::Publisher tilt_pub_;
  ros::Publisher link_pub_;
  ros::Subscriber config_sub_;
  ros::Subscriber tilt_sub_;
  ros::ServiceClient pcd_client_;
  int rungs;
  QLabel* pointcloud_status;
};
// END_TUTORIAL
#endif // ADJUSTMENT_INTERFACE_H
