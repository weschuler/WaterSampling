#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle node_handle;

std_msgs::String sensor1_msg;
std_msgs::UInt16 sensor2_msg;

ros::Publisher sensor1_publisher("arduino_sensor1", &sensor1_msg);
ros::Publisher sensor2_publisher("arduino_sensor2", &sensor2_msg);

void setup()
{
  node_handle.initNode();
  node_handle.advertise(sensor1_publisher);
  node_handle.advertise(sensor2_publisher);
}

void loop()
{ 

  sensor2_msg.data = 5;
  sensor1_msg.data = "Hello From Arduino!";
  
  sensor1_publisher.publish( &sensor1_msg );
  sensor2_publisher.publish( &sensor2_msg );
  
  node_handle.spinOnce();
  
  delay(100);
}
