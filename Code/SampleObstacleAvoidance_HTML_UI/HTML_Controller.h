#include <vs-rc202.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <FS.h>
#include <math.h>

#define GO 0
#define LEFT 1
#define RIGHT 2
#define BACK 3
#define STOP 4
#define FUNC1 5
#define FUNC2 6
#define CTRL 7

#define INTERVAL  100    //moveOmuni3での移動時の制御周期

const char* ui_path = "/index.html";
const char* ssid = "Aterm_WM3800R";
const char* password = "jikkenjikken";
#define BUF_SIZE 10240
uint8_t buf[BUF_SIZE];
int led_onoff_flag = 0;
String st_data[24];

ESP8266WebServer server(80);

//サーボモータ回転速度補正係数
double alpha = -1.33096 * pow(10,-6);
double beta = -6.94394 * pow(10,-5);
double gam = 1.263727114;
double delta = -0.6045050505;

//角度->ラジアン変換
double deg2rad(int deg){
  double rad;
  rad = deg * 3.14 /180;
  
  return rad;
}

//速度(velocity -600～600)、移動方向[°](axis 0～360)、旋回量(omega -600～600)から各サーボの回転速度を算出する
void moveOmuni3(int velocity, int axis, int omega){
  int v1, v2, v3;
  double vx, vy, rad;
  
  rad = deg2rad(axis);
  vy = velocity*cos(rad);
  vx = velocity*sin(rad);

  v1 = vx+omega;
  v1 =  alpha * pow(v1,3) + beta * pow(v1,2) + gam * v1 + delta;  //サーボモータ回転速度補正
  v2 = -0.5*vx + 0.865*vy + omega;
  v2 =  alpha * pow(v2,3) + beta * pow(v2,2) + gam * v2 + delta;  //サーボモータ回転速度補正
  v3 = -0.5*vx - 0.865*vy + omega;
  v3 =  alpha * pow(v3,3) + beta * pow(v3,2) + gam * v3 + delta;  //サーボモータ回転速度補正
  
  setServoDeg(1, v1);
  setServoDeg(2, v2);
  setServoDeg(3, v3);
  setServoMovingTime(INTERVAL); //Set moving time
  moveServo();

  return;
}

int loadUI(){
  File ui = SPIFFS.open(ui_path, "r");
  if (!ui) {
    Serial.println("index.html does not exist.");
    return 0;
  }else if(ui.size() >= BUF_SIZE){
    Serial.print("Could not load file. Make file smaller.");
    return 0;
  }else{
    ui.read(buf,ui.size());
    ui.close();
    return 1;
  }
}

void selectMotion(){
  switch(getMotionNumber()){
    case GO:
      moveOmuni3(600,0,0);
      break;
    case LEFT:
      moveOmuni3(600,90,0);
      break;
    case RIGHT:
      moveOmuni3(600,270,0);
      break;
    case BACK:
      moveOmuni3(600,180,0);
      break;
    case STOP:
      moveOmuni3(0,0,0);
      break;
   case FUNC1:
      moveOmuni3(0,0,-600);
      break;
   case FUNC2:
      moveOmuni3(0,0,600);
      break;
  }
}

//Send UI page
void handleRoot(){
  server.send(200, "text/html", (char *)buf);
}

void Go(){
  setMotionNumber(GO);
  //Serial.println("Go");
  server.send(200, "text/html","Go");
}

void Left(){
  setMotionNumber(LEFT);
  //Serial.println("Left");
  server.send(200, "text/html","Left");
}

void Right(){
  setMotionNumber(RIGHT);
  //Serial.println("Right");
  server.send(200, "text/html","Right");
}

void Back(){
  setMotionNumber(BACK);
  //Serial.println("Back");
  server.send(200, "text/html","Back");
}

void Stop(){
  setMotionNumber(STOP);
  //Serial.println("Stop");
  server.send(200, "text/html","Stop");
}

void F1(){
  setMotionNumber(FUNC1);
  //Serial.println("F1");
  server.send(200, "text/html","F1");
}

void F2(){
  setMotionNumber(FUNC2);
  //Serial.println("F2");
  server.send(200, "text/html","F2");
}

// Autonomous control
void F3(){
  setMotionNumber(CTRL);
//  setMotionNumber(STOP);
//  setBuzzer(PC5, BEAT4, TANG);
//  setBuzzer(PC5, BEAT4, TANG);
//  setBuzzer(PG5, BEAT4, TANG);
//  setBuzzer(PG5, BEAT4, TANG);
//  setBuzzer(PA5, BEAT4, TANG);
//  setBuzzer(PA5, BEAT4, TANG);
//  setBuzzer(PG5, BEAT2, TANG);
//
//  setBuzzer(PF5, BEAT4, TANG);
//  setBuzzer(PF5, BEAT4, TANG);
//  setBuzzer(PE5, BEAT4, TANG);
//  setBuzzer(PE5, BEAT4, TANG);
//  setBuzzer(PD5, BEAT4, TANG);
//  setBuzzer(PD5, BEAT4, TANG);
//  setBuzzer(PC5, BEAT2, TANG);
//
//  setBuzzer(PG5, BEAT4, TANG);
//  setBuzzer(PG5, BEAT4, TANG);
//  setBuzzer(PF5, BEAT4, TANG);
//  setBuzzer(PF5, BEAT4, TANG);
//  setBuzzer(PE5, BEAT4, TANG);
//  setBuzzer(PE5, BEAT4, TANG);
//  setBuzzer(PD5, BEAT2, TANG);
//
//  setBuzzer(PG5, BEAT4, TANG);
//  setBuzzer(PG5, BEAT4, TANG);
//  setBuzzer(PF5, BEAT4, TANG);
//  setBuzzer(PF5, BEAT4, TANG);
//  setBuzzer(PE5, BEAT4, TANG);
//  setBuzzer(PE5, BEAT4, TANG);
//  setBuzzer(PD5, BEAT2, TANG);
//
//  setBuzzer(PC5, BEAT4, TANG);
//  setBuzzer(PC5, BEAT4, TANG);
//  setBuzzer(PG5, BEAT4, TANG);
//  setBuzzer(PG5, BEAT4, TANG);
//  setBuzzer(PA5, BEAT4, TANG);
//  setBuzzer(PA5, BEAT4, TANG);
//  setBuzzer(PG5, BEAT2, TANG);
//
//  setBuzzer(PF5, BEAT4, TANG);
//  setBuzzer(PF5, BEAT4, TANG);
//  setBuzzer(PE5, BEAT4, TANG);
//  setBuzzer(PE5, BEAT4, TANG);
//  setBuzzer(PD5, BEAT4, TANG);
//  setBuzzer(PD5, BEAT4, TANG);
//  setBuzzer(PC5, BEAT2, TANG);
  
  server.send(200, "text/html","F3");
}

void F4(){
  //Serial.println("F4");
  server.send(200, "text/html","F4");
  powerOff();
}

void Sens(){

  //int sens_data[3];

  //Sensor data from lpc
  //int data1 = readSens(1);
  //int data2 = readSens(2);
  //int data3 = readSens(3);
 
  //Sonic sensor data
 // double dist = getDist();

 // Power supply voltage
  int power = readPow();

  //Convert numeric to string
//  String st_data1 = String(data1);
//  String st_data2 = String(data2);
//  String st_data3 = String(data3);  
//  String st_data4 = String(dist);
    String st_data5 = String(power);
  
//  String res = String(st_data1+","+st_data2+","+st_data3+","+st_data4+","+st_data5);

    String res = String(st_data[0]+","+st_data[1]+","+st_data[2]+","+st_data[3]+","+st_data[4]+","+
                st_data[5]+","+st_data[6]+","+st_data[7]+","+st_data[8]+","+st_data[9]+","+
                st_data[10]+","+st_data[11]+","+st_data[12]+","+st_data[13]+","+st_data[14]+","+
                st_data[15]+","+st_data[16]+","+st_data[17]+","+st_data[18]+","+st_data[19]+","+
                st_data[20]+","+st_data[21]+","+st_data[22]+","+st_data[23]+","+st_data5);

  server.send(200, "text/html",res);
}

void setup_wifi() {

  //Read html file
  SPIFFS.begin();
  if(!loadUI()){
    return;
  }

  //Connect to AP
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);
//  Serial.println("Connecting to Access Point");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    Serial.print(".");
  }
//  Serial.println();
//  Serial.print("Connected to ");
//  Serial.println(WiFi.SSID()); 
//  Serial.print("IP address: ");
//  Serial.println(WiFi.localIP());

  //Set function
  server.on("/", handleRoot);
  server.on("/go/", Go);
  server.on("/left/", Left);
  server.on("/right/", Right);
  server.on("/back/", Back);
  server.on("/stop/", Stop);
  server.on("/f1/", F1);
  server.on("/f2/", F2);
  server.on("/f3/", F3);
  server.on("/f4/", F4);
  server.on("/sens/", Sens);

  setMotionNumber(CTRL);

  //Start server
  server.begin();
}

void HTML_CMD() {
  server.handleClient();
  //Play motion
  if(getMotionNumber()!= CTRL){
    selectMotion();
  }
}
