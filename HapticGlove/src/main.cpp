#include <functions.h>
#include <variables.h>

void Callback(const std_msgs::Float64& msg){
  float timediff = (micros() - msg.data);
  timesSum += timediff;
  timesCount++;
  // String str = (String)timediff;
  if(millis() - Update >= 1000){
    Update = millis();
    float average = timesSum / timesCount;
    latency.data = average;
    latency_pub.publish(&latency);
    // String str = (String)average;
    // nh.loginfo(str.c_str());
    timesSum = 0;
    timesCount = 0;
  }
  // nh.loginfo(str.c_str());
}
ros::Subscriber<std_msgs::Float64> sub("ruckflug", &Callback);
void setup(void){
  //Begin ROS node and set baud rate
  nh.getHardware()->setBaud(850000);
  nh.initNode();
  //Advertise ROS topics
  nh.advertise(indexMCP_pub);
  nh.advertise(indexPIP_pub);
  nh.advertise(middleMCP_pub);
  nh.advertise(middlePIP_pub);
  nh.advertise(ringMCP_pub);
  nh.advertise(ringPIP_pub);
  nh.advertise(thumbMCP_pub);
  nh.advertise(thumbPIP_pub);
  nh.advertise(palm_pub);
  nh.advertise(float_pub);
  nh.advertise(latency_pub);
  nh.subscribe(sub);
  //Begin I^2C connection 
  Wire.begin();
  //Set I^2C data rate to the maximum possible rate
  Wire.setClock(3400000UL);
  delay(1000);
  //Begin communication with the IMUs
  setupFingers();  
  setupThumb();
  setupPalm();
}

void loop()
{ 
  if(millis() - lastUpdate >= 10){
    lastUpdate = millis();
    // before = micros();
    // readOutIMUData();
    // publishIMUData();
    // after = micros();
    // timediff = after - before;
    // timesSum += timediff;
    // timesCount++;
    // if(millis() - Update >= 1000){
    //   Update = millis();
    //   float average = timesSum / timesCount;
    //   latency.data = average;
    //   latency_pub.publish(&latency);
    //   // String str = (String)average;
    //   // nh.loginfo(str.c_str());
    //   timesSum = 0;
    //   timesCount = 0;
    // }
    // String str = (String)timediff;
    float_msg.data = micros();
    float_pub.publish(&float_msg);
    // nh.loginfo(str.c_str());
  }
  nh.spinOnce();
}