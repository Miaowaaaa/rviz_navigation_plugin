#ifndef TELEOP_PAD_H
#define TELEOP_PAD_H
 
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin base
 
class QLineEdit;
 
namespace rviz_telop_commander
{

class TeleopPanel: public rviz::Panel
{
// statement Q_OBJECT 
Q_OBJECT
public:
  // constructor, QWidget will be used for initialize GUI
  TeleopPanel( QWidget* parent = 0 );
 
  // Override for save configuration for rviz
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;
 
  // public slots
public Q_SLOTS:
  // set topic for publisher
  void setTopic( const QString& topic );
 
protected Q_SLOTS:
  void sendVel();                 // send velocity
  void update_Linear_Velocity();  // update linear velocity
  void update_Angular_Velocity(); 
  void updateTopic();             
 
protected:
  // topic name 
  QLineEdit* output_topic_editor_;
  QString output_topic_;
 
  // linear velocity
  QLineEdit* output_topic_editor_1;
  QString output_topic_1;
 
  // angular velocity
  QLineEdit* output_topic_editor_2;
  QString output_topic_2;
 
  ros::Publisher velocity_publisher_;
 
  // The ROS node handle.
  ros::NodeHandle nh_;
 
  float linear_velocity_;
  float angular_velocity_;
};
 
} // end namespace rviz_plugin_tutorials
 
#endif // TELEOP_PANEL_H