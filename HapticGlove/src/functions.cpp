#include <functions.h>
#include <variables.h>

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void setupFingers(){
  for(uint8_t i = 0; i < fingersAmount; i++){
    delay(20);
    tcaselect(i);  
    uint8_t a = 2 * i;
    uint8_t b = 2 * i + 1;
    if(!bno.begin()){
      String log = "Sensor " + (String)a + " not detected";
      nh.logerror(log.c_str());
      i = -1;
    }
    else{
      String log = "Sensor " + (String)a + " detected";
      nh.loginfo(log.c_str());
    }
    if(!bno2.begin()){
      String log = "Sensor " + (String)b + " not detected";
      nh.logerror(log.c_str());
      i = -1;
    }
    else{
      String log = "Sensor " + (String)b + " detected";
      nh.loginfo(log.c_str());
    }
    delay(20); 
    bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
    bno2.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
    bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);
    bno2.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);
    bno.setExtCrystalUse(true);  
    bno2.setExtCrystalUse(true);
    // bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS); 
    // bno2.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);     
  }
}
void setupThumb(){
  for(uint8_t i = 0; i < 2; i++){
    uint8_t a = 4 + i;
    tcaselect(a);  
    while(!bno.begin()){
      String log = "Sensor T" + (String)a + " not detected";
      nh.logerror(log.c_str());
      delay(50);
    }
    String log = "Sensor T" + (String)a + " detected";
    nh.loginfo(log.c_str());
    delay(20); 
    bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
    bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);
    bno.setExtCrystalUse(true);  
    // bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
  }   
}
void setupPalm(){
  tcaselect(3);  
  while(!bno.begin()){
    nh.logerror("Palm sensor not detected");
    delay(50);
  }
  nh.loginfo("Palm sensor detected");
  delay(20); 
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);
  bno.setExtCrystalUse(true);  
  // bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
}
void readOutIMUData(){
  for(uint8_t i = 0; i < fingersAmount; i++){
    tcaselect(i);      
    bno.getEvent(&event);    
    quat[2*i] = bno.getQuat();    
    //second IMU
    bno2.getEvent(&event);    
    quat[2*i+1] = bno2.getQuat();       
  }   
  readOutThumb();
  tcaselect(3);
  bno.getEvent(&event);
  quat[11] = bno.getQuat();
  // nh.loginfo("error");
}
void readOutThumb(){
  for(uint8_t i = 0; i < 2; i++){
    tcaselect(4 + i);    
    bno.getEvent(&event);    
    quat[8+i] = bno.getQuat();    
  }  
}
void publishIMUData(){
    indexMCP_msg.w = quat[0].w();
    indexMCP_msg.x = quat[0].x();
    indexMCP_msg.y = quat[0].y();
    indexMCP_msg.z = quat[0].z();
    indexMCP_pub.publish(&indexMCP_msg);
    indexPIP_msg.w = quat[1].w();
    indexPIP_msg.x = quat[1].x();
    indexPIP_msg.y = quat[1].y();
    indexPIP_msg.z = quat[1].z();
    indexPIP_pub.publish(&indexPIP_msg);
    middleMCP_msg.w = quat[2].w();
    middleMCP_msg.x = quat[2].x();
    middleMCP_msg.y = quat[2].y();
    middleMCP_msg.z = quat[2].z();
    middleMCP_pub.publish(&middleMCP_msg);
    middlePIP_msg.w = quat[3].w();
    middlePIP_msg.x = quat[3].x();
    middlePIP_msg.y = quat[3].y();
    middlePIP_msg.z = quat[3].z();
    middlePIP_pub.publish(&middlePIP_msg);
    ringMCP_msg.w = quat[4].w();
    ringMCP_msg.x = quat[4].x();
    ringMCP_msg.y = quat[4].y();
    ringMCP_msg.z = quat[4].z();
    ringMCP_pub.publish(&ringMCP_msg);
    ringPIP_msg.w = quat[5].w();
    ringPIP_msg.x = quat[5].x();
    ringPIP_msg.y = quat[5].y();
    ringPIP_msg.z = quat[5].z();
    ringPIP_pub.publish(&ringPIP_msg);
    thumbMCP_msg.w = quat[8].w();
    thumbMCP_msg.x = quat[8].x();
    thumbMCP_msg.y = quat[8].y();
    thumbMCP_msg.z = quat[8].z();
    thumbMCP_pub.publish(&thumbMCP_msg);
    thumbPIP_msg.w = quat[9].w();
    thumbPIP_msg.x = quat[9].x();
    thumbPIP_msg.y = quat[9].y();
    thumbPIP_msg.z = quat[9].z();
    thumbPIP_pub.publish(&thumbPIP_msg);
    palm_msg.w = quat[11].w();
    palm_msg.x = quat[11].x();
    palm_msg.y = quat[11].y();
    palm_msg.z = quat[11].z();
    palm_pub.publish(&palm_msg);
}
