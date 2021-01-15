#include <vs-rc202.h>
#include <Arduino.h>
#include <FS.h>
#include <math.h>
#include <SoftwareSerial.h>
SoftwareSerial Lidar(5, 14);//, false, 256); // RX 5 (IC2 SCL), TX 14 (Sonic TRIG)


int decode(const char code[], int byt)
{
  int value = 0;
  int i;
  for (i = 0; i < byt; ++i) {
    value <<= 6;
    value &= ~0x3f;
    value |= code[i] - 0x30;
  }
  return value;
}

//Send a scan command
void scan()
{
  /*Prepare the MD scan command:
        start step:0044 (-120 deg)
        end step:0725 (120 deg)
        clustering:29 約10 deg間隔
        scan skip:0
        scans:01　*/

  uint8_t data[] = "GS0044072529\n";
  Lidar.write(data, 13); /* send　to the sensor */
  Lidar.flush();
  delay(200);
}


void setup()
{
  //Init robot lib
  initLib();
  delay(10);

  Serial.begin(9600);
  Lidar.begin(9600);

  delay(100);
  uint8_t data[] = "BM\n";
//  Lidar.write(data, 3);   /* send　to the sensor */
//  Lidar.flush();

  String line[2]; // String line
  line[0] = "99";
  line[1] = "99";
  
  while(1){
    Lidar.write(data, 3);   /* send　to the sensor */
    Lidar.flush();
    delay(200);
    
    // LRFからデータを読み込む
    if (Lidar.available()) {
      line[0] = Lidar.readStringUntil('\n');  // Response
      line[1] = Lidar.readStringUntil('\n');  // Status
    }    

    // Clear serial buffer
    while (Lidar.available()) {
      Lidar.read();
    }
//    Serial.println(line[0]);
//    Serial.println(line[1]);
//    Serial.flush();
    if (line[1].startsWith("00") || line[1].startsWith("02")) {
      break;
    }    
  }  
  delay(100);
}

void loop()
{
  char buf[128], buf2[3];   //  Buffer
  char message[256];
  String line[4];
  int lrf[24] = {0};        //  距離情報　[mm]
  int i, j, n;

  scan();

  if (Lidar.available() > 0) {
    line[0] = Lidar.readStringUntil('\n');  // Response
    line[1] = Lidar.readStringUntil('\n');  // Status
    line[2] = Lidar.readStringUntil('\n');  // Time stamp [ms]
    line[3] = Lidar.readStringUntil('\n');  // Data

    line[3].toCharArray(buf, 128);  // charに変換

    for (i = 0; i < 24; i++) {
      for (j = 0; j < 2; j++) {
        buf2[j] = buf[2 * i + j];
      }
      lrf[i] = decode(buf2, 2);
    }

    // Clear serial buffer
    while (Lidar.available()) {
      Lidar.read();
    }

    //    for (i = 0; i < 4; i++) {
    //      Serial.println(line[i]);
    //    }

    for (i = 0; i < 24; i++) {
      sprintf(message, "%4d - %4d deg: %4d ", (10 * i - 120), (10 * (i + 1) - 120), lrf[i]);
      Serial.print(message);
      n = lrf[i] / 50;
      for (j = 0; j < n; j++) {
        Serial.print('=');
      }
      Serial.print('\n');
    }
    Serial.println("---------------------------");
    Serial.flush();

  }
}
