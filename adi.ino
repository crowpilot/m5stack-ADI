#include <M5Stack.h>
#include<TinyGPS++.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

MPU9250 IMU;
HardwareSerial GPSRaw(2);

TinyGPSPlus gps;


void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Wire.begin();
  
  GPSRaw.begin(9600);
  
  M5.Lcd.setTextSize(2);
  randomSeed(analogRead(0));
  //IMU.calibrateMPU9250(IMU.gyroBias,IMU.accelBias);
  IMU.initMPU9250();
  IMU.initAK8963(IMU.magCalibration);


}

float lastroll,lastpitch;

void loop() {
    M5.Lcd.drawCircle(160,120,110,TFT_GREEN);

    M5.Lcd.fillRect(160-2,0,4,10,TFT_GREEN);
   
    M5.Lcd.drawLine(160-21,120-118,160-19,120-108,TFT_GREEN);
    M5.Lcd.drawLine(160-41,120-113,160-38,120-103,TFT_GREEN);
    M5.Lcd.drawLine(160-60,120-104,160-55,120-95,TFT_GREEN);
    M5.Lcd.drawLine(160-104,120-60,160-95,120-55,TFT_GREEN);
    M5.Lcd.drawLine(160+21,120-118,160+19,120-108,TFT_GREEN);
    M5.Lcd.drawLine(160+41,120-113,160+38,120-103,TFT_GREEN);
    M5.Lcd.drawLine(160+60,120-104,160+55,120-95,TFT_GREEN);
    M5.Lcd.drawLine(160+104,120-60,160+95,120-55,TFT_GREEN); 
  // put your main code here, to run repeatedly:
  float acx,acy,acz,gyx,gyy,gyz,magx,magy,magz;
  if(IMU.readByte(MPU9250_ADDRESS,INT_STATUS) & 0x01){
    IMU.readAccelData(IMU.accelCount);
    IMU.readGyroData(IMU.gyroCount);
    IMU.readMagData(IMU.magCount);
    
    IMU.getAres();
    IMU.getGres();
    IMU.getMres();

    IMU.magbias[0] = +470;
    IMU.magbias[1] =+120;
    IMU.magbias[2] = +125;
    
    acx = IMU.accelCount[0]*IMU.aRes;
    acy = IMU.accelCount[1]*IMU.aRes;
    acz = IMU.accelCount[2]*IMU.aRes;

    gyx = IMU.gyroCount[0]*IMU.gRes;
    gyy = IMU.gyroCount[1]*IMU.gRes;
    gyz = IMU.gyroCount[2]*IMU.gRes;

    magx = (float)IMU.magCount[0]*IMU.mRes*IMU.magCalibration[0]-IMU.magbias[0];
    magy = (float)IMU.magCount[1]*IMU.mRes*IMU.magCalibration[1] -IMU.magbias[1];
    magz = (float)IMU.magCount[2]*IMU.mRes*IMU.magCalibration[2] -IMU.magbias[2];

   IMU.updateTime();
    
    MahonyQuaternionUpdate(acz,acy,acx,gyz*DEG_TO_RAD,gyy*DEG_TO_RAD,gyx*DEG_TO_RAD,magz,magy,magx,IMU.deltat);
  }
 
  M5.Lcd.setCursor(0,0);
//  M5.Lcd.printf("ac x:%+2.2f y:%+2.2f z:%+2.2f\n",acx,acy,acz);
//  M5.Lcd.printf("gy x:%+4.f y:%+4.f z:%+4.f\n",gyx,gyy,gyz);
  if(M5.BtnA.wasPressed()){
      M5.Lcd.drawRect(random(0,320),random(0,240),random(0,320),random(0,240),TFT_BLUE);
    }
    IMU.delt_t = millis() - IMU.count;

    IMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() **(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)- *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    IMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) **(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)- *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    IMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() **(getQ()+2)))*RAD_TO_DEG;

    
//    M5.Lcd.printf("%.+04.0f,%+04.0f,%+04.0f",IMU.pitch,IMU.yaw*RAD_TO_DEG,IMU.roll*RAD_TO_DEG);

    float lastbar =  sqrt(abs(12100-lastpitch*lastpitch*16));
    float pitchbar = sqrt(abs(12100-IMU.pitch*IMU.pitch*16));
    //horizontal line
    
    M5.Lcd.drawLine(160-lastpitch*4*cos(lastroll)-lastbar*sin(lastroll),
                    120+lastpitch*4*sin(lastroll)-lastbar*cos(lastroll),
                    160-lastpitch*4*cos(lastroll)+lastbar*sin(lastroll),
                    120+lastpitch*4*sin(lastroll)+lastbar*cos(lastroll),TFT_BLACK);
    if(abs(IMU.pitch*4)<100){
    M5.Lcd.drawLine(160-IMU.pitch*4*cos(IMU.roll)-pitchbar*sin(IMU.roll),
                    120+IMU.pitch*4*sin(IMU.roll)-pitchbar*cos(IMU.roll),
                    160-IMU.pitch*4*cos(IMU.roll)+pitchbar*sin(IMU.roll),
                    120+IMU.pitch*4*sin(IMU.roll)+pitchbar*cos(IMU.roll),TFT_GREEN);
    }
    for(int i=-9;i<=9;i++){
      M5.Lcd.drawLine(160-30,120+40*i+lastpitch*4,160+30,120+40*i+lastpitch*4,TFT_BLACK);
      M5.Lcd.drawLine(160-15,120-20+40*i+lastpitch*4,160+15,120-20+40*i+lastpitch*4,TFT_BLACK);
      M5.Lcd.setCursor(160-(lastpitch*4-40*i)*cos(lastroll)+30*sin(lastroll)+5,
                        120+(lastpitch*4-40*i)*sin(lastroll)+30*cos(lastroll)-5);
      M5.Lcd.setTextColor(TFT_BLACK);
      M5.Lcd.printf("%d",i*10);

     M5.Lcd.drawLine(160-(lastpitch*4-40*i)*cos(lastroll)-30*sin(lastroll),
                      120+(lastpitch*4-40*i)*sin(lastroll)-30*cos(lastroll),
                      160-(lastpitch*4-40*i)*cos(lastroll)+30*sin(lastroll),
                      120+(lastpitch*4-40*i)*sin(lastroll)+30*cos(lastroll),
                      TFT_BLACK);
     M5.Lcd.drawLine(160-(lastpitch*4-40*i-20)*cos(lastroll)-15*sin(lastroll),
                      120+(lastpitch*4-40*i-20)*sin(lastroll)-15*cos(lastroll),
                      160-(lastpitch*4-40*i-20)*cos(lastroll)+15*sin(lastroll),
                      120+(lastpitch*4-40*i-20)*sin(lastroll)+15*cos(lastroll),
                      TFT_BLACK);
      
      if(abs(IMU.pitch*4-i*40)<100){
        
         M5.Lcd.drawLine(160-(IMU.pitch*4-40*i)*cos(IMU.roll)-30*sin(IMU.roll),
                          120+(IMU.pitch*4-40*i)*sin(IMU.roll)-30*cos(IMU.roll),
                          160-(IMU.pitch*4-40*i)*cos(IMU.roll)+30*sin(IMU.roll),
                          120+(IMU.pitch*4-40*i)*sin(IMU.roll)+30*cos(IMU.roll),
                          TFT_GREEN);
         M5.Lcd.drawLine(160-(IMU.pitch*4-40*i-20)*cos(IMU.roll)-15*sin(IMU.roll),
                          120+(IMU.pitch*4-40*i-20)*sin(IMU.roll)-15*cos(IMU.roll),
                          160-(IMU.pitch*4-40*i-20)*cos(IMU.roll)+15*sin(IMU.roll),
                          120+(IMU.pitch*4-40*i-20)*sin(IMU.roll)+15*cos(IMU.roll),
                          TFT_GREEN);
         M5.Lcd.setCursor(160-(IMU.pitch*4-40*i)*cos(IMU.roll)+30*sin(IMU.roll)+5,
                          120+(IMU.pitch*4-40*i)*sin(IMU.roll)+30*cos(IMU.roll)-5);
         M5.Lcd.setTextColor(TFT_GREEN);
         M5.Lcd.printf("%d",i*10);
        }
        
      }

    //mini aircraft
    //M5.Lcd.drawLine(160-80*sin(lastroll),120+80*cos(lastroll),160+80*sin(lastroll),120-80*cos(lastroll),TFT_BLACK);
    M5.Lcd.drawLine(160-50,120,160+50,120,TFT_ORANGE);
    //bank bar
    M5.Lcd.drawLine(160+110*cos(lastroll),120-110*sin(lastroll),160+80*cos(lastroll),120-80*sin(lastroll),TFT_BLACK);
    M5.Lcd.drawLine(160+110*cos(IMU.roll),120-110*sin(IMU.roll),160+80*cos(IMU.roll),120-80*sin(IMU.roll),TFT_GREEN);

    M5.Lcd.setCursor(0,0);
    M5.Lcd.setTextColor(TFT_GREEN,TFT_BLACK);
    M5.Lcd.println(IMU.yaw*RAD_TO_DEG);

    //GPS 
    //while(!gps.location.isUpdated()){

      while(GPSRaw.available()>0){
        int rawdata = GPSRaw.read();
        Serial.write(rawdata);
        if(gps.encode(rawdata)){
          M5.Lcd.print("break");
          break;
        }
      }
   //   }
    
    M5.Lcd.print(gps.satellites.value());

    M5.update();

    lastroll= IMU.roll;
    lastpitch = IMU.pitch;

    float g = sqrt(acx*acx+acy*acy+acz*acz);
    M5.Lcd.setCursor(0,210);
    M5.Lcd.setTextColor(TFT_GREEN,TFT_BLACK);
    M5.Lcd.printf("%+1.2f",g);
}
