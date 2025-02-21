#include <variables.h>
#include <functions.h>

//Adresses of the Multiplexers
#define TCAADDR 0x70

//Define the IMUs and their adress
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29);

// Declare ROS node handle
ros::NodeHandle nh;

// Declare ROS messages and publishers
geometry_msgs::Quaternion indexMCP_msg;
geometry_msgs::Quaternion  indexPIP_msg;
geometry_msgs::Quaternion  middleMCP_msg;
geometry_msgs::Quaternion  middlePIP_msg;
geometry_msgs::Quaternion  ringMCP_msg;
geometry_msgs::Quaternion  ringPIP_msg;
geometry_msgs::Quaternion  thumbMCP_msg;
geometry_msgs::Quaternion  thumbPIP_msg;
geometry_msgs::Quaternion  palm_msg;
std_msgs::Float64 latency;
std_msgs::Float64 float_msg;
ros::Publisher latency_pub("latency", &latency);
ros::Publisher float_pub("hinflug", &float_msg);
ros::Publisher indexMCP_pub("indexMCP", &indexMCP_msg);
ros::Publisher indexPIP_pub("indexPIP", &indexPIP_msg);
ros::Publisher middleMCP_pub("middleMCP", &middleMCP_msg);
ros::Publisher middlePIP_pub("middlePIP", &middlePIP_msg);
ros::Publisher ringMCP_pub("ringMCP", &ringMCP_msg);
ros::Publisher ringPIP_pub("ringPIP", &indexPIP_msg);
ros::Publisher thumbMCP_pub("thumbMCP", &thumbMCP_msg);
ros::Publisher thumbPIP_pub("thumbPIP", &thumbPIP_msg);
ros::Publisher palm_pub("palm_angle", &palm_msg);

//event variables to save the data from the sensors
sensors_event_t event;
imu::Quaternion quat[12];
int fingersAmount = 3;

// external variables for the main loop
unsigned long lastUpdate = 0;
uint32_t sequence = 0;
unsigned long before = 0;
unsigned long after = 0;
unsigned long timediff = 0;
float timesSum = 0;
int timesCount = 0;
unsigned long Update = 0;

