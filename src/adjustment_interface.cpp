#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/default_plugin/interactive_marker_display.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/config.h"
#include "rviz/yaml_config_reader.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <adjustment_localization/AdjustmentParameters.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <teleop_msgs/RateControl.h>
#include "adjustment_interface.h"

#include "adjustment_interface.h"
#include <stdio.h>

void AdjustmentInterface::updateParameters(const adjustment_localization::AdjustmentParameters::ConstPtr& msg){
    ui->spn_x->setValue(msg->x);
}


void AdjustmentInterface::setTilt(const geometry_msgs::Quaternion::ConstPtr& msg){
    tf::Quaternion q;
    tf::quaternionMsgToTF(*msg,q);
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
    ui->spn_roll->setValue(roll);
    ui->spn_pitch->setValue(pitch);
    ui->spn_yaw->setValue(yaw);
}

AdjustmentInterface::AdjustmentInterface( QWidget* parent ):
    QMainWindow(parent),
    ui(new Ui_adjustmentinterface)
{
    ui->setupUi(this);
    rviz::RenderPanel*render_panel;
    render_panel = new rviz::RenderPanel();
    render_panel->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    ui->verticalLayout->addWidget(render_panel);
    manager_ = new rviz::VisualizationManager( render_panel );
    render_panel->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();
    
    //load the configuration
    rviz::YamlConfigReader reader;
    rviz::Config config;
    std::string path = ros::package::getPath("adjustment_ui");
    path += "/robot_view.rviz";
    std::cout<<path<<std::endl;
    reader.readFile(config, path.c_str());
    if(reader.error())
      std::cout<<"ERROR reading config\n";
    else{
      config = config.mapGetChild("Visualization Manager");
      manager_->load(config);
    }
    n_ = ros::NodeHandle();
    adjustment_pub_ =n_.advertise<geometry_msgs::Point>("adjustment/point",1000);
    tilt_pub_ =n_.advertise<geometry_msgs::Quaternion>("adjustment/tilt",1000);
    link_pub_ =n_.advertise<std_msgs::Int32>("adjustment/link",1000);
    pcd_client_ =n_.serviceClient<teleop_msgs::RateControl>("relay");
    tilt_sub_ = n_.subscribe("/adjustment/tilt2",1000,&AdjustmentInterface::setTilt,this);
    resetAdjustment();
}

// Destructor.
AdjustmentInterface::~AdjustmentInterface()
{
  delete manager_;
}


void AdjustmentInterface::changeAdjustment(int adjustment){
    std_msgs::Int32 msg;
    msg.data=adjustment;
    link_pub_.publish(msg);
    resetAdjustment();
}

void AdjustmentInterface::resetAdjustment(){
    geometry_msgs::Point myMsg;
    myMsg.x=myMsg.y=myMsg.z=0;
    ui->spn_x->setValue(0);
    ui->spn_y->setValue(0);
    ui->spn_z->setValue(0);
    ui->spn_roll->setValue(0);
    ui->spn_pitch->setValue(0);
    ui->spn_yaw->setValue(0);
    sendAdjustment();
}

void AdjustmentInterface::sendAdjustment()
{
    geometry_msgs::Point myMsg;
    myMsg.x=ui->spn_x->value();
    myMsg.y=ui->spn_y->value();
    myMsg.z=ui->spn_z->value();
    adjustment_pub_.publish(myMsg);
}

void AdjustmentInterface::sendTilt()
{
    geometry_msgs::Quaternion myMsg;
    tf::Quaternion q;
    q.setRPY(ui->spn_roll->value(),ui->spn_pitch->value(),ui->spn_yaw->value());
    tf::quaternionTFToMsg(q,myMsg);
    tilt_pub_.publish(myMsg);
}

void AdjustmentInterface::requestPointCloud()
{
    ui->lbl_pcd_status->setText("Requesting...");
    ui->lbl_pcd_status->setStyleSheet("QLabel { background-color : yellow;border-style : outset; border-width : 1px; border-color : black;}");
    teleop_msgs::RateControl srv;
    srv.request.Rate = -1.0;
    if (pcd_client_.call(srv))
    {
        ui->lbl_pcd_status->setText("Success");
        ui->lbl_pcd_status->setStyleSheet("QLabel { background-color : green;border-style : outset; border-width : 1px; border-color : black;}");
      }
    else
    {
        ui->lbl_pcd_status->setText("Service Failed");
        ui->lbl_pcd_status->setStyleSheet("QLabel { background-color : red;border-style : outset; border-width : 1px; border-color : black;}");
    }
}
