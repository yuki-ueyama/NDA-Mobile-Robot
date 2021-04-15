#include <vs-rc202.h>
#include <math.h>
#include "HTML_Controller.h"

// #define INTERVAL  100    //moveOmuni3での移動時の制御周期


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
  uint8_t data[] = "GS0044072529\n";
  int bytesSent = Serial.write(data, 13);
  Serial.flush();
}

void setup()
{
  //Init robot lib
  initLib();
  delay(10);

  buzzerEnable(1);

  //SV1-3 servo mode
  servoEnable(1, 1);        //Enable SV1 PWM
  servoEnable(2, 1);        //Enable SV2 PWM
  servoEnable(3, 1);        //Enable SV3 PWM

  //Offset
  setServoOffset(1, 0);  //Offset range:-500 to 500
  setServoOffset(2, 0);  //モータに合わせて設定してください
  setServoOffset(3, 0);

  setServoMode(1);    // 動作をオーバーライド

  Serial.begin(9600);
  delay(200);

  setup_wifi();

  uint8_t data[] = "BM\n";
  String line[2]; // String line
  line[0] = "99";
  line[1] = "99";

  while (1) {
    Serial.write(data, 3);   /* send　to the sensor */
    Serial.flush();
    delay(200);

    // LRFからデータを読み込む
    if (Serial.available()) {
      line[0] = Serial.readStringUntil('\n');  // Response
      line[1] = Serial.readStringUntil('\n');  // Status
    }

    // Clear serial buffer
    while (Serial.available()) {
      Serial.read();
    }

    if (line[1].startsWith("00") || line[1].startsWith("02")) {
      break;
    }
  }

  delay(500);
}

void loop()
{
  char buf[128], buf2[3];  // Buffer
  String line[4]; // String line
  int lrf[24] = {0}; //Data from laser rangefinder　[mm]
  static int lrf_[24] = {0};
  char message[128];
  int i, j;

  scan();

  // LRFからデータを読み込む
  if (Serial.available()) {
    line[0] = Serial.readStringUntil('\n');  // Response
    line[1] = Serial.readStringUntil('\n');  // Status
    line[2] = Serial.readStringUntil('\n');  // Time stamp [ms]
    line[3] = Serial.readStringUntil('\n');  //  距離情報

    line[3].toCharArray(buf, 128);  // charに変換

    for (i = 0; i < 24; i++) {
      for (j = 0; j < 2; j++) {
        buf2[j] = buf[2 * i + j];
      }
      lrf[i] = decode(buf2, 2);
      st_data[i] = String(lrf[i]);    //Convert numeric to string
    }

    // Clear serial buffer
    while (Serial.available()) {
      Serial.read();
    }
  }
  HTML_CMD();
  if (getMotionNumber() == CTRL) {
    
    // ここからプログラムを作成 ----------------------------------------

    int velocity = 0;
    int axis = 0;
    int omega = 0;

    for (i = 0; i < 24; i++) {
      if (lrf[i] > 1000 || lrf[i] == 0 ) {
        lrf[i] = 1000;
      }
    }

    if (lrf[11]  > 500 ) {
      velocity += 500;

      if (lrf[3] < 500) {
        omega = -100;
      }

      if (lrf[20] < 500) {
        omega += 100;
      }

    }
    else {
      omega = -320;
    }

    moveOmuni3(velocity, axis, omega);


    // ここまで -------------------------------------------------------
    
  }
  delay(200);

}
