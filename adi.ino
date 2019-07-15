#include <M5Stack.h>
//#include<TinyGPS++.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

MPU9250 IMU;
//HardwareSerial GPSRaw(2);

//TinyGPSPlus gps;


void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Wire.begin();
  
//  GPSRaw.begin(9600);
  
  M5.Lcd.setTextSize(2);
  randomSeed(analogRead(0));
  //IMU.calibrateMPU9250(IMU.gyroBias,IMU.accelBias);
  IMU.initMPU9250();
  IMU.initAK8963(IMU.magCalibration);


}

float lastroll,lastpitch,lastacx;

void loop() {
    M5.Lcd.drawCircle(160,120,110,TFT_GREEN);

    M5.Lcd.fillRect(158,0,4,10,TFT_GREEN);
   
    M5.Lcd.drawLine(139,2,141,12,TFT_GREEN);
    M5.Lcd.drawLine(119,7,122,17,TFT_GREEN);
    M5.Lcd.drawLine(100,16,105,25,TFT_GREEN);
    M5.Lcd.drawLine(56,60,65,65,TFT_GREEN);
    M5.Lcd.drawLine(181,2,179,12,TFT_GREEN);
    M5.Lcd.drawLine(201,7,198,17,TFT_GREEN);
    M5.Lcd.drawLine(220,16,215,25,TFT_GREEN);
    M5.Lcd.drawLine(264,60,255,65,TFT_GREEN); 
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
    magy = (float)IMU.magCount[1]*IMU.mRes*IMU.magCalibration[1]-IMU.magbias[1];
    magz = (float)IMU.magCount[2]*IMU.mRes*IMU.magCalibration[2]-IMU.magbias[2];

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

    IMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() **(getQ()+3)), 
                      *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)- *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
                      
    IMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) **(getQ()+3)), 
                      *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)- *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
                      
    IMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() **(getQ()+2)))*RAD_TO_DEG;

    
//    M5.Lcd.printf("%.+04.0f,%+04.0f,%+04.0f",IMU.pitch,IMU.yaw*RAD_TO_DEG,IMU.roll*RAD_TO_DEG);

    float lastbar =  sqrt(abs(12100-lastpitch*lastpitch*16));
    float pitchbar = sqrt(abs(12100-IMU.pitch*IMU.pitch*16));
    //yoku tsukau
    float coslroll=cos(lastroll);
    float sinlroll=sin(lastroll);

    float cosroll = cos(IMU.roll);
    float sinroll = sin(IMU.roll);

    
//Horizon bar
    M5.Lcd.drawLine(160-lastpitch*4*coslroll-lastbar*sinlroll,
                    120+lastpitch*4*sinlroll-lastbar*coslroll,
                    160-lastpitch*4*coslroll+lastbar*sinlroll,
                    120+lastpitch*4*sinlroll+lastbar*coslroll,TFT_BLACK);
    if(abs(IMU.pitch*4)<100){
    M5.Lcd.drawLine(160-IMU.pitch*4*cosroll-pitchbar*sinroll,
                    120+IMU.pitch*4*sinroll-pitchbar*cosroll,
                    160-IMU.pitch*4*cosroll+pitchbar*sinroll,
                    120+IMU.pitch*4*sinroll+pitchbar*cosroll,TFT_GREEN);
    }
    for(int i=-9;i<=9;i++){
      //M5.Lcd.drawLine(160-30,120+40*i+lastpitch*4,160+30,120+40*i+lastpitch*4,TFT_BLACK);
      //M5.Lcd.drawLine(160-15,120-20+40*i+lastpitch*4,160+15,120-20+40*i+lastpitch*4,TFT_BLACK);
      M5.Lcd.setCursor(160-(lastpitch*4-40*i)*coslroll+30*sinlroll+5,
                        120+(lastpitch*4-40*i)*sinlroll+30*coslroll-5);
      M5.Lcd.setTextColor(TFT_BLACK);
      M5.Lcd.printf("%d",i*10);

    //Pitch scale 10deg 60px 5deg 30px
     M5.Lcd.drawLine(160-(lastpitch*4-40*i)*coslroll-30*sinlroll,
                      120+(lastpitch*4-40*i)*sinlroll-30*coslroll,
                      160-(lastpitch*4-40*i)*coslroll+30*sinlroll,
                      120+(lastpitch*4-40*i)*sinlroll+30*coslroll,
                      TFT_BLACK);
     M5.Lcd.drawLine(160-(lastpitch*4-40*i-20)*coslroll-15*sinlroll,
                      120+(lastpitch*4-40*i-20)*sinlroll-15*coslroll,
                      160-(lastpitch*4-40*i-20)*coslroll+15*sinlroll,
                      120+(lastpitch*4-40*i-20)*sinlroll+15*coslroll,
                      TFT_BLACK);
      
      if(abs(IMU.pitch*4-i*40)<100){
        
         M5.Lcd.drawLine(160-(IMU.pitch*4-40*i)*cosroll-30*sinroll,
                          120+(IMU.pitch*4-40*i)*sinroll-30*cosroll,
                          160-(IMU.pitch*4-40*i)*cosroll+30*sinroll,
                          120+(IMU.pitch*4-40*i)*sinroll+30*cosroll,
                          TFT_GREEN);
         M5.Lcd.drawLine(160-(IMU.pitch*4-40*i-20)*cosroll-15*sinroll,
                          120+(IMU.pitch*4-40*i-20)*sinroll-15*cosroll,
                          160-(IMU.pitch*4-40*i-20)*cosroll+15*sinroll,
                          120+(IMU.pitch*4-40*i-20)*sinroll+15*cosroll,
                          TFT_GREEN);
         M5.Lcd.setCursor(160-(IMU.pitch*4-40*i)*cosroll+30*sinroll+5,
                          120+(IMU.pitch*4-40*i)*sinroll+30*cosroll-5);
         M5.Lcd.setTextColor(TFT_GREEN);
         M5.Lcd.printf("%d",i*10);
        }
        
      }

    //mini aircraft
    //M5.Lcd.drawLine(160-80*sin(lastroll),120+80*cos(lastroll),160+80*sin(lastroll),120-80*cos(lastroll),TFT_BLACK);
    M5.Lcd.drawLine(160-50,120,160+50,120,TFT_ORANGE);
    M5.Lcd.drawRect(158,118,5,5,TFT_ORANGE);
    //bank bar
    M5.Lcd.drawLine(160+110*coslroll,120-110*sinlroll,160+80*coslroll,120-80*sinlroll,TFT_BLACK);
    M5.Lcd.drawLine(160+110*cosroll,120-110*sinroll,160+80*cosroll,120-80*sinroll,TFT_GREEN);

    M5.Lcd.drawLine(160-100*sin(lastacx/2),120+100*cos(lastacx/2),160-110*sin(lastacx/2),120+110*cos(lastacx/2),TFT_BLACK);
    M5.Lcd.drawLine(160-100*sin(acx/2),120+100*cos(acx/2),160-110*sin(acx/2),120+110*cos(acx/2),TFT_GREEN);
    
    //M5.Lcd.setCursor(0,0);
   // M5.Lcd.setTextColor(TFT_GREEN,TFT_BLACK);
  //  M5.Lcd.printf("%+2.2f  %+2.2f\n",acx,acz);

    //GPS 
    //while(!gps.location.isUpdated()){
/*
      while(GPSRaw.available()>0){
        int rawdata = GPSRaw.read();
        Serial.write(rawdata);
        if(gps.encode(rawdata)){
          //M5.Lcd.print("break");
          break;
        }
      }
  */ //   }
    
   // M5.Lcd.print(gps.location.lat());

    M5.update();

    lastroll= IMU.roll;
    lastpitch = IMU.pitch;

    lastacx = acx;

    float g = sqrt(acx*acx+acy*acy+acz*acz);
    M5.Lcd.setCursor(0,210);
    M5.Lcd.setTextColor(TFT_GREEN,TFT_BLACK);
    M5.Lcd.printf("%+1.2f",g);
}
