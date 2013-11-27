#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QTextFormat>

#include <yaml-cpp/node.h>
#include <yaml-cpp/parser.h>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/default_plugin/interactive_marker_display.h"
#include "rviz/properties/ros_topic_property.h"
#include "adjustment_interface.h"
#include "rviz/config.h"
#include "rviz/yaml_config_reader.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <teleop_msgs/RateControl.h>

// Constructor for Adjustment_Interface.  This does most of the work of the class.
Adjustment_Interface::Adjustment_Interface( QWidget* parent )
  : QWidget( parent )
{

  // Construct and lay out labels and slider controls.
  QPushButton* request_pointcloud = new QPushButton("Request Pointcloud");
  pointcloud_status = new QLabel("Not Requested");
  pointcloud_status->setMinimumWidth(120);
  pointcloud_status->setStyleSheet("QLabel { background-color : white; border-style : outset; border-width : 1px; border-color : black}");
  QPushButton* send_adjustment = new QPushButton("Send Adjustment");
  QPushButton* ladder_reset = new QPushButton("Reset");
  QLabel* lbl_rungs = new QLabel("Rungs #");
  QSpinBox* number_rungs = new QSpinBox();
  rungs=6;
  number_rungs->setValue(rungs);
  QHBoxLayout* controls_layout = new QHBoxLayout();
  controls_layout->addWidget( request_pointcloud);
  controls_layout->addWidget( pointcloud_status);
  controls_layout->addSpacing(30);
  //  controls_layout->addWidget( lbl_rungs);
  //controls_layout->addWidget( number_rungs );
  controls_layout->addWidget( ladder_reset);
  //controls_layout->addStretch();
  //controls_layout->addWidget( plan_motion);

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );
  setMinimumSize(640,480);

  // Set the top-level layout for this Adjustment_Interface widget.
  setLayout( main_layout );
  // Make signal/slot connections.
  //  connect( plan_motion, SIGNAL(clicked()),this, SLOT( toPlanner()));
  connect( ladder_reset, SIGNAL(clicked()),this, SLOT( resetLadder()));
  //  connect( number_rungs, SIGNAL(valueChanged(int)),this, SLOT(changeNumRungs(int)));
  connect(request_pointcloud, SIGNAL(clicked()),this,SLOT(requestPointCloud()));
  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();

  //load the configuration
  rviz::YamlConfigReader reader;
  rviz::Config config;
  reader.readFile( config, "/home/jordan/.rviz/front_view.rviz");
//  reader.readFile( config, "front_view.rviz");
  if(reader.error())
    std::cout<<"ERROR reading config\n";
  else{
    config = config.mapGetChild("Visualization Manager");
    manager_->load(config);
  }
  n_ = ros::NodeHandle();
  str_pub_ =n_.advertise<std_msgs::String>("export",1000);
  int_pub_ =n_.advertise<std_msgs::Int32>("reset",1000);
  int_pub2_ =n_.advertise<std_msgs::Int32>("rungs",1000);
  pcd_client_ =n_.serviceClient<teleop_msgs::RateControl>("relay");
}

// Destructor.
Adjustment_Interface::~Adjustment_Interface()
{
  delete manager_;
}

void Adjustment_Interface::toPlanner()
{
  std_msgs::String myMsg;
  myMsg.data = "Export";
  str_pub_.publish(myMsg);
  ros::spinOnce();
}

void Adjustment_Interface::requestPointCloud()
{
    pointcloud_status->setText("Requesting...");
    pointcloud_status->setStyleSheet("QLabel { background-color : yellow;border-style : outset; border-width : 1px; border-color : black;}");
    teleop_msgs::RateControl srv;
    srv.request.Rate = -1.0;
    if (pcd_client_.call(srv))
    {
        pointcloud_status->setText("Success");
        pointcloud_status->setStyleSheet("QLabel { background-color : green;border-style : outset; border-width : 1px; border-color : black;}");
      }
    else
    {
        pointcloud_status->setText("Service Failed");
        pointcloud_status->setStyleSheet("QLabel { background-color : red;border-style : outset; border-width : 1px; border-color : black;}");
    }
    //if(pcd_pub.call)
}

void Adjustment_Interface::resetLadder()
{
    std_msgs::Int32 myMsg;
    myMsg.data = rungs;
    int_pub_.publish(myMsg);
    ros::spinOnce();
}

void Adjustment_Interface::changeNumRungs(int num)
{
  rungs=num;
  std_msgs::Int32 myMsg;
  myMsg.data = rungs;
  int_pub2_.publish(myMsg);
  ros::spinOnce();
}
