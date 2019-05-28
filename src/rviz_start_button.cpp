#include <stdio.h>
 
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
 
#include <geometry_msgs/Twist.h>
#include <QDebug>
 
#include "rviz_start_button.h"
 
namespace rviz_telop_commander
{
// constructor
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{
 
  // create a input panel
  QVBoxLayout* topic_layout = new QVBoxLayout;
  topic_layout->addWidget( new QLabel( "Teleop Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );
 
  // linear velocity input
  topic_layout->addWidget( new QLabel( "Linear Velocity:" ));
  output_topic_editor_1 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_1 );
 
  // Angular velocity input
  topic_layout->addWidget( new QLabel( "Angular Velocity:" ));
  output_topic_editor_2 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_2 );
 
  QHBoxLayout* layout = new QHBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );
 
  // Timer for publish velocity message
  QTimer* output_timer = new QTimer( this );
 
  // connect signal with slots
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));             // 输入topic命名，回车后，调用updateTopic()
  connect( output_topic_editor_1, SIGNAL( editingFinished() ), this, SLOT( update_Linear_Velocity() )); // 输入线速度值，回车后，调用update_Linear_Velocity()
  connect( output_topic_editor_2, SIGNAL( editingFinished() ), this, SLOT( update_Angular_Velocity() ));// 输入角速度值，回车后，调用update_Angular_Velocity()
 
  // connect callback function with Timer
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
 
  // set decay for Timer 100ms
  output_timer->start( 100 );
}
 
// update linear velocity
void TeleopPanel::update_Linear_Velocity()
{
    // get the input data
    QString temp_string = output_topic_editor_1->text();
 
    float lin = temp_string.toFloat();

    linear_velocity_ = lin;
}
 
// update angular velocity
void TeleopPanel::update_Angular_Velocity()
{
    QString temp_string = output_topic_editor_2->text();

    float ang = temp_string.toFloat() ;

    angular_velocity_ = ang;
}
 
// update the name of topic
void TeleopPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}
 
// set Topic name
void TeleopPanel::setTopic( const QString& new_topic )
{
  // check the change of topic name.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
 
    // null for nothing
    if( output_topic_ == "" )
    {
      velocity_publisher_.shutdown();
    }
    // reset the publisher
    else
    {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }
 
    Q_EMIT configChanged();
  }
}
 
// publish message
void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish( msg );
  }
}
 
// Override
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}
 
// Override
void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}
 
}
 
// A rviz plugin statement
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_telop_commander::TeleopPanel,rviz::Panel )