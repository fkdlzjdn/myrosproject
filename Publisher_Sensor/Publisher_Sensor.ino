

/*
 *Publisher sensor data
 */

#include <ros.h>
#include <std_msgs/Float32.h>

#define TRIG_PIN 2
#define ECHO_PIN 3

ros::NodeHandle  nh;

std_msgs::Float32 str_msg;
ros::Publisher chatter("chatter", &str_msg);




float distance = 800;


void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  
}

void loop()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(1000);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 5000);

 
  if(duration > 0)
    distance = duration * 0.17;

 
  str_msg.data = distance;
  chatter.publish( &str_msg );
  nh.spinOnce();
 
  delay(100);
  
  Serial.println(distance);
  
 
  
}
