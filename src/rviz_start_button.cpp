#include "rviz_start_button.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>
#include <QLabel>

namespace rviz_navigation
{

NavigationButton::NavigationButton(QWidget* parent)
    :rviz::Panel(parent),
    _doing_navigation(false)
{
    _clear_pub = _nh.advertise<std_msgs::Bool>("/clear_nav_point",100);
    _label_pub = _nh.advertise<std_msgs::String>("/point_label",1);

    QVBoxLayout *root = new QVBoxLayout;
    _start_b = new QPushButton("Start Navigation");
    _delete_b = new QPushButton("Clear Goals");
    _record_b = new QPushButton("Voice Command");
    _send_b = new QPushButton("Set Label");
    _name_input = new QLineEdit;
    QLabel *name_l = new QLabel("Point Name:");
    QHBoxLayout *label_layout = new QHBoxLayout;
    label_layout->addWidget(name_l);
    label_layout->addWidget(_name_input);
    label_layout->addWidget(_send_b);

    root->addWidget(_start_b);
    root->addWidget(_delete_b);
    root->addWidget(_record_b);
    root->addLayout(label_layout);
    setLayout(root);

    connect( _start_b, SIGNAL(clicked()),this,SLOT(start()));

    connect( _delete_b, SIGNAL(clicked()),this,SLOT(clear()));

    connect( _record_b, SIGNAL(pressed()),this,SLOT(record_voice()));
    connect( _record_b, SIGNAL(released()),this,SLOT(stop_record()));

    connect( _send_b, SIGNAL(clicked()),this,SLOT(send_label()));
}

void NavigationButton::send_label()
{
    std_msgs::String msg;
    QString label = _name_input->text();
    msg.data = label.toStdString();
    if(msg.data == "")
    {
        ROS_WARN("Can be empty!");
        return;
    }
    _label_pub.publish(msg);
}

void NavigationButton::start()
{
    boost::thread thread(boost::bind(&NavigationButton::call_nav_service,this));
}

void NavigationButton::clear()
{
    std_msgs::Bool msg;
    msg.data = true;
    _clear_pub.publish(msg);
}

void NavigationButton::record_voice()
{
    boost::thread thread(boost::bind(&NavigationButton::call_voi_service,this));
}

void NavigationButton::stop_record()
{
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;
    ros::service::call("voice_service",req,resp);
    _record_b->setText("Voice Command");
}
void NavigationButton::call_voi_service()
{
    ROS_INFO("Begining record voice...");
    _record_b->setText("Recording...");
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;

    // check if the service exists
    if(!ros::service::exists("voice_service",true))
    {
        _record_b->setText("Voice Command");
        ROS_WARN("no such service!");
        return;
    }
    //call
    bool success = ros::service::call("voice_service",req,resp);
    if(success)
    {
        
        ROS_INFO("End Recording...");
        return;
    }
    else
    {

        ROS_WARN("Something error in voice recording");
    }
}

//Override
void NavigationButton::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("start_button",_start_b->text());
}

//Override
void NavigationButton::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    QString start_s;
    if(config.mapGetString("start_button",&start_s))
    {
        _start_b->setText(start_s);
    }

}

void NavigationButton::call_nav_service()
{
    ROS_INFO("Test output!");
    ROS_INFO("start navigation!");
    _start_b->setEnabled(false);
    _start_b->setText("doing navigation...");
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;
    _doing_navigation = true;
    if(!ros::service::exists("patrol_service",true))
    {
        _start_b->setEnabled(true);
        _start_b->setText("start navigation");
        _doing_navigation = false;
        ROS_WARN("no such service!");
        return;
    }
    bool success = ros::service::call("patrol_service",req,resp);
    if(success)
    {
        _start_b->setEnabled(true);
        _start_b->setText("start navigation");
        _doing_navigation = false;
    }
    else
    {
        _start_b->setEnabled(true);
        _start_b->setText("start navigation");
        _doing_navigation = false;  
    }
}

}
// A rviz plugin statement
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_navigation::NavigationButton,rviz::Panel )