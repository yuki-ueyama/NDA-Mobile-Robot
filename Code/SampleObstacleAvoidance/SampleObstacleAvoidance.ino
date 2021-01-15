#include <vs-rc202.h>
#include <math.h>

#define INTERVAL  100    //moveOmuni3での移動時の制御周期

//サーボモータ回転速度補正係数
double alpha = -1.33096 * pow(10, -6);
double beta = -6.94394 * pow(10, -5);
double gam = 1.263727114;
double delta = -0.6045050505;

//角度->ラジアン変換
double deg2rad(int deg)
{
  double rad;
  rad = deg * 3.14 / 180;

  return rad;
}

//速度(velocity -600～600)、移動方向[°](axis 0～360)、旋回量(omega -600～600)から各サーボの回転速度を算出する
void moveOmuni3(int velocity, int axis, int omega)
{
  int v1, v2, v3;
  double vx, vy, rad;

  rad = deg2rad(axis);
  vy = velocity * cos(rad);
  vx = velocity * sin(rad);

  v1 = vx + omega;
  v1 =  alpha * pow(v1, 3) + beta * pow(v1, 2) + gam * v1 + delta; //サーボモータ回転速度補正
  v2 = -0.5 * vx + 0.865 * vy + omega;
  v2 =  alpha * pow(v2, 3) + beta * pow(v2, 2) + gam * v2 + delta; //サーボモータ回転速度補正
  v3 = -0.5 * vx - 0.865 * vy + omega;
  v3 =  alpha * pow(v3, 3) + beta * pow(v3, 2) + gam * v3 + delta; //サーボモータ回転速度補正

  setServoDeg(1, v1);
  setServoDeg(2, v2);
  setServoDeg(3, v3);
  setServoMovingTime(INTERVAL); //Set moving time
  moveServo();

  return;
}

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
  /* send　to the sensor */
  int bytesSent = Serial.write(data, 13);
  Serial.flush();
}


void setup()
{
  //Init robot lib
  initLib();
  delay(10);

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

  //  // Clear serial buffer
  //  while (Serial.available()) {
  //    Serial.read(); // Clear serial buffer
  //  }
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
    }

    // Clear serial buffer
    while (Serial.available()) {
      Serial.read();
    }
  }

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

  delay(200);

}
