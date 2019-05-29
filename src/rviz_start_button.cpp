#include "rviz_start_button.h"
#include <QVBoxLayout>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <boost/thread.hpp>

namespace rviz_navigation
{

NavigationButton::NavigationButton(QWidget* parent)
    :rviz::Panel(parent),
    _doing_navigation(false)
{
    _clear_pub = _nh.advertise<std_msgs::Bool>("/clear_nav_point",100);

    QVBoxLayout *root = new QVBoxLayout;
    _start_b = new QPushButton("Start Navigation");
    _delete_b = new QPushButton("Clear Goals");
    root->addWidget(_start_b);
    root->addWidget(_delete_b);
    setLayout(root);

    connect( _start_b, SIGNAL(clicked()),this,SLOT(start()));

    connect( _delete_b, SIGNAL(clicked()),this,SLOT(clear()));
}

void NavigationButton::start()
{
    boost::thread thread(boost::bind(&NavigationButton::test,this));
}

void NavigationButton::clear()
{
    std_msgs::Bool msg;
    msg.data = true;
    _clear_pub.publish(msg);
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

void NavigationButton::test()
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