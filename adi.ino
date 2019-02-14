#include <M5Stack.h>
//#include <TinyGPS++.h>
#include <VL53L0X.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

MPU9250 IMU;
HardwareSerial GPSRaw(2);

//TinyGPSPlus gps;
VL53L0X vl53;

int fullstroke = 1000;
int stroke = 0;
int count=0;
TaskHandle_t th[1];

/*
void getTof(void *pvParameters){
  //Wire.begin();
  //vl53.init();
  while(1){
   //int stroke = 1;//vl53.readRangeSingleMillimeters();
    delay(1000);
  }
}*/

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Wire.begin();
  
//  GPSRaw.begin(9600);

  vl53.init();
 
vl53.setMeasurementTimingBudget(20000);
  //vl53.setTimeout(500);
  
  M5.Lcd.setTextSize(2);
 // randomSeed(analogRead(0));
  //IMU.calibrateMPU9250(IMU.gyroBias,IMU.accelBias);
  IMU.initMPU9250();
  IMU.initAK8963(IMU.magCalibration);

//  xTaskCreatePinnedToCore(getTof,"getTof",4096,NULL,10,&th[0],0);

}

int biasflag = 0;
float lastroll,lastpitch,lastacx,lastacy;
float accelbias[3] = {0,0,0};
float mag_max[3] = {0,0,0};
float mag_min[3] = {0,0,0};
float pitchbias = 0;

void loop() {

  //atitude indicator outline
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

    
    acx = IMU.accelCount[0]*IMU.aRes;
    acy = IMU.accelCount[1]*IMU.aRes;
    acz = IMU.accelCount[2]*IMU.aRes;

    gyx = IMU.gyroCount[0]*IMU.gRes;
    gyy = IMU.gyroCount[1]*IMU.gRes;
    gyz = IMU.gyroCount[2]*IMU.gRes;

  
    magx = (float)IMU.magCount[0]*IMU.mRes*IMU.magCalibration[0];
    magy = (float)IMU.magCount[1]*IMU.mRes*IMU.magCalibration[1];
    magz = (float)IMU.magCount[2]*IMU.mRes*IMU.magCalibration[2];

    if(magx>mag_max[0]){
    mag_max[0]=magx;
  }
  if(magy>mag_max[1]){
    mag_max[1]=magy;
  }
  if(magz>mag_max[2]){
    mag_max[2]=magz;
  }
  if(magx<mag_min[0]){
    mag_min[0]=magx;
  }
  if(magy<mag_min[1]){
    mag_min[1]=magy;
  }
  if(magz<mag_min[2]){
    mag_min[2]=magz;
  }
    IMU.magbias[0] =(mag_max[0]+mag_min[0])/2;
    IMU.magbias[1] =(mag_max[1]+mag_min[1])/2;
    IMU.magbias[2] =(mag_max[2]+mag_min[2])/2;

  magx -= IMU.magbias[0];
  magy -= IMU.magbias[1];
  magz -= IMU.magbias[2];
    
  
   IMU.updateTime();
    
    MahonyQuaternionUpdate(acz,acy,acx,
                            gyz*DEG_TO_RAD,gyy*DEG_TO_RAD,gyx*DEG_TO_RAD,
                            magz,magy,magx
                            ,IMU.deltat);

//    IMU.count = milllis();
  }
 
  M5.Lcd.setCursor(0,0);
//  M5.Lcd.printf("ac x:%+2.2f y:%+2.2f z:%+2.2f\n",acx,acy,acz);
//  M5.Lcd.printf("gy x:%+4.f y:%+4.f z:%+4.f\n",gyx,gyy,gyz);
  if(M5.BtnA.wasPressed()){
      //M5.Lcd.drawRect(random(0,320),random(0,240),random(0,320),random(0,240),TFT_BLUE);
      biasflag = 1;
    }
  if(M5.BtnB.wasPressed()){
    biasflag = 1;
  }
  if(M5.BtnB.pressedFor(3000)){
    M5.Lcd.clear();
    M5.Lcd.println("calibration mode");
    M5.Lcd.println("\nset display side up.");
    M5.Lcd.println("keep level");
    M5.Lcd.println("push left button");
    M5.Lcd.setCursor(0,220);
    M5.Lcd.println("calibrate");
    M5.Lcd.setCursor(220,220);
    M5.Lcd.println("cancel");  
    while(1){
      delay(1);
      M5.update();
      if(M5.BtnA.wasPressed()){
        IMU.calibrateMPU9250(IMU.gyroBias,IMU.accelBias);
        M5.Lcd.clear();
        break;
      }
      if(M5.BtnC.wasPressed()){
        M5.Lcd.clear();
        break;
      }
    }
    biasflag = 1;
  }
    IMU.delt_t = millis() - IMU.count;

    //IMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() **(getQ()+3)), 
    //                *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)- *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3))
    //                *RAD_TO_DEG;
    IMU.yaw = atan2(2.0f*(*(getQ())* *(getQ()+3)+*(getQ()+1)* *(getQ()+2)),1.0f-2.0f*(*(getQ()+2)**(getQ()+2)+*(getQ()+3)**(getQ()+3)))*RAD_TO_DEG;
                      
    IMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) **(getQ()+3)), 
                      *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)- *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
                      
    IMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() **(getQ()+2)))*RAD_TO_DEG;


    if(biasflag){
      accelbias[0] = acx;
      accelbias[1] = acy;
      accelbias[2] = acz;
      pitchbias = IMU.pitch;
      biasflag = 0;
      }
      
      IMU.pitch-=pitchbias;
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
    
    M5.Lcd.setCursor(0,0);
    M5.Lcd.setTextColor(TFT_GREEN,TFT_BLACK);

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
      }*/
  // }

   //G Bowl
   M5.Lcd.drawCircle(280,200,35,TFT_GREEN);
   M5.Lcd.drawLine(245,200,315,200,TFT_GREEN);
   M5.Lcd.drawLine(280,165,280,235,TFT_GREEN);

  int bowlx = (acx-accelbias[0])*20;
  int bowly = (acy-accelbias[1])*20;
   M5.Lcd.fillRect(277-lastacx,197+lastacy,6,6,TFT_BLACK);
   M5.Lcd.fillRect(277-bowlx,197+bowly,6,6,TFT_ORANGE);
//stroke sensor
/*
    int stroke_gage = stroke*148/fullstroke;
    if(stroke < fullstroke){
    M5.Lcd.fillRect(301,21,18,stroke_gage-1,TFT_BLACK);
    M5.Lcd.drawRect(300,20,20,150,TFT_GREEN);  
    M5.Lcd.fillRect(301,20+stroke_gage,18,149-stroke_gage,TFT_GREEN);
    }
    else{
      M5.Lcd.fillRect(301,21,18,148,TFT_BLACK);
    }
    M5.Lcd.printf("%f\n%f\n%f\n%f\n%f",gps.location.lat(),IMU.yaw,magx,magy,magz);
    if(count == 5){
    stroke = vl53.readRangeSingleMillimeters();
    count = 0;
    }
    count++;*/
//g meter
    float g = sqrt(acx*acx+acy*acy+acz*acz);
    M5.Lcd.setCursor(0,210);
    M5.Lcd.setTextColor(TFT_GREEN,TFT_BLACK);
    M5.Lcd.printf("%+1.2f",g);   

    M5.update();

    lastroll= IMU.roll;
    lastpitch = IMU.pitch;

    lastacx = bowlx;
    lastacy = bowly;

}
