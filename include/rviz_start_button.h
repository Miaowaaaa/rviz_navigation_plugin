#ifndef RVIZ_START_BUTTON
#define RVIZ_START_BUTTON

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>
#include <QPushButton>

namespace rviz_navigation
{

class NavigationButton: public rviz::Panel
{
Q_OBJECT

public:
    NavigationButton(QWidget *parent = 0);

    //Override
    virtual void save(rviz::Config conifg) const;

    //Override
    virtual void load(const rviz::Config &Config);

public Q_SLOTS:

    //start navigation
    void start();

    //  navigaiton goals
    void clear();

    //record voice
    void record_voice();

    //stop record
    void stop_record();

protected:

    void call_nav_service();
    void call_voi_service();

protected:

    QPushButton *_start_b;
    QPushButton *_delete_b;
    QPushButton *_record_b;
    bool _doing_navigation;
    ros::NodeHandle _nh;
    ros::Publisher _clear_pub;
};

}

#endif
