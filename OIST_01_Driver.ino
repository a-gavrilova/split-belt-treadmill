// ______________________________________Correct here______________________________________

// PID parameters

float kp1 = 2.5, ki1 = 2.0, kd1 = 0.0;
float kp2 = 2.5, ki2 = 2.0, kd2 = 0.0;

// Wi-Fi SSID and password
const char ssid[]     = "Driver";
const char password[] = "12345678903";

// ________________________________________________________________________________________


#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "GyverPID.h"

// General
#define UPD_TIME 25
#define PID_UPD_TIME 1

bool stop_flag = false;
unsigned long previousMillis, pidMillis;

// Motors setup
#define M1_PIN 2
#define M2_PIN 14
#define PWM_FREQ 40000
#define PWM_RANGE 255

#define PWM_PID_MAX 255
#define RPM_MAX 1500
int M1_rpm_target, M2_rpm_target;

// Add parameters UART update
GyverPID M1PID(kp1, ki1, kd1, PID_UPD_TIME); // Kp, Ki, Kd, update time
GyverPID M2PID(kp2, ki2, kd2, PID_UPD_TIME); // Kp, Ki, Kd, update time

// Encoder setup
#define ENC1_PIN 5
#define ENC2_PIN 13

#define ENC_COUNT_REV 360.0
volatile unsigned int Enc1_read, Enc2_read;
unsigned int M1_rpm_current, M2_rpm_current;
float rot2rpm = 60000.0 / (float)UPD_TIME; // 1 min/measuring period

// WiFi setup
String webpage = "";
IPAddress local_IP(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 2);
IPAddress subnet(255, 255, 255, 0);
ESP8266WebServer server(80);


IRAM_ATTR void EncoderRead1() {
  Enc1_read++;
}

IRAM_ATTR void EncoderRead2() {
  Enc2_read++;
}

int SetM1RPM(String speed_val) {
  //  From 0 to 9999
  //  Speed format: M1_XXXX_M2_YYYY
  int M1_speed = 1000 * (speed_val[3] - 48) + 100 * (speed_val[4] - 48) + 10 * (speed_val[5] - 48) + (speed_val[6] - 48);
  if (M1_speed > RPM_MAX) {
    M1_speed = RPM_MAX;
  }
  M1PID.setpoint = M1_speed;
  return M1_speed;
}

int SetM2RPM(String speed_val) {
  //  From 0 to 9999
  //  Speed format: M1_XXXX_M2_YYYY
  int M2_speed = 1000 * (speed_val[11] - 48) + 100 * (speed_val[12] - 48) + 10 * (speed_val[13] - 48) + (speed_val[14] - 48);
  if (M2_speed > RPM_MAX) {
    M2_speed = RPM_MAX;
  }
  M2PID.setpoint = M2_speed;
  return M2_speed;
}

void append_page_header() {
  webpage = F("<!DOCTYPE html><html><head><title>Motor driver v1.0</title></head><body>");
}

void append_page_footer() {
  webpage += F("</body></html>");
}

void HomePage() {

  append_page_header();
  webpage = webpage + F("<h3> Motor driver v1.0 </h3>");
  webpage += "<p> M1current: " + (String)M1_rpm_current + " RPM </p>";
  webpage += "<p> M1target: "  + (String)M1_rpm_target + " RPM </p>";
  webpage += "<p>   </p>";
  webpage += "<p> M2current: " + (String)M2_rpm_current + " RPM </p>";
  webpage += "<p> M2target: "  + (String)M2_rpm_target + " RPM </p>";

  webpage += F("<p> <a href='/setspeed'> Set motors speed </a></p>");
  webpage += F("<h3> <a href='/stop'> STOP </a></h3>");

  append_page_footer();
  server.sendContent(webpage);
  webpage = "";
  server.sendContent(webpage);
  server.client().stop();
}

void SetSpeed() {
  if ((server.args() > 0) && (server.hasArg("setspeed"))) {
    String data_read = server.arg(0);
    append_page_header();
    if (data_read.length() == 15) {
      M1_rpm_target = SetM1RPM(data_read);
      M2_rpm_target = SetM2RPM(data_read);
      webpage += "<p>Speed has been setted</p>";
      webpage += "<p> M1target: " + (String)M1_rpm_target + " RPM </p>";
      webpage += "<p> M2target: " + (String)M2_rpm_target + " RPM </p>";
      stop_flag = false;
    }
    else {
      webpage += "<p style='color:red;'>Wrong speed value!!!</p>";
    }
    webpage += F("<a href='/setspeed'>Set speed page</a><br><br>");
    webpage += F("<a href='/'>Start page</a><br><br>");
    append_page_footer();
    server.sendContent(webpage);
    webpage = "";
    server.sendContent(webpage);
    server.client().stop();
  }
  else {
    append_page_header();
    webpage += F("<p>Speed format: M1_XXXX_M2_YYYY </p>");
    webpage += F("<p>Example:  Motor1 = 8 RPM, Motor2 = 850 RPM</p>");
    webpage += F("<p><i>M1_0008_M2_0850</i></p>");
    webpage += F("<form action='/setspeed' method='post'><label for='setspeed'>Set speed: </label><input type='text' name='setspeed' value=''><br><br>");
    webpage += F("<h3> \t </h3>");
    webpage += F("<a href='/'>Start page</a><br><br>");
    append_page_footer();
    server.sendContent(webpage);
    webpage = "";
    server.sendContent(webpage);
    server.client().stop();
  }
}

void DriverStop() {
  append_page_header();
  webpage += "<h1> Driver was stopped </h1>";
  webpage += F("<a href='/'>Start page</a><br><br>");
  append_page_footer();
  server.sendContent(webpage);
  webpage = "";
  server.sendContent(webpage);
  server.client().stop();
  analogWrite(M1_PIN, 0);
  analogWrite(M2_PIN, 0);
  M1_rpm_target = 0;
  M2_rpm_target = 0;
  M1PID.setpoint = 0;
  M2PID.setpoint = 0;
  stop_flag = true;
}

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println();
  Serial.println("M1_rpm_current:,M1_rpm_target:,M2_rpm_current:,M2_rpm_target:");

  // Encoder setup
  pinMode(ENC1_PIN, INPUT);
  pinMode(ENC2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_PIN), EncoderRead1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_PIN), EncoderRead2, RISING);

  // Motors setup
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  analogWrite(M1_PIN, 0);
  analogWrite(M2_PIN, 0);

  M1PID.setDirection(NORMAL);
  M1PID.setLimits(0, PWM_PID_MAX);
  M1PID.setpoint = 0;
  //  M1PID.setDt(PID_UPD_TIME);

  M2PID.setDirection(NORMAL);
  M2PID.setLimits(0, PWM_PID_MAX);
  M2PID.setpoint = 0;
  //  M2PID.setDt(PID_UPD_TIME);

  // WiFi setup
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  server.on("/",          HomePage);
  server.on("/setspeed",  SetSpeed);
  server.on("/stop",      DriverStop);
  server.begin();
  previousMillis = millis();
  pidMillis = millis();

}

void PIDparsing() {
  if (Serial.available() > 0) {
    String serialData = Serial.readStringUntil('\n');

    //   PIDX_pp.pp_ii.ii_dd.dd

    if ((serialData.indexOf("PID1") >= 0 ) && (serialData.length() == 22)) {
      kp1 = 10 * (serialData[5] - 48)  +  (serialData[6] - 48) + 0.1 * (serialData[8] - 48)  +  0.01 * (serialData[9] - 48);
      M1PID.Kp = kp1;
      ki1 = 10 * (serialData[11] - 48) + (serialData[12] - 48) + 0.1 * (serialData[14] - 48) +  0.01 * (serialData[15] - 48);
      M1PID.Ki = ki1;
      kd1 = 10 * (serialData[17] - 48) + (serialData[18] - 48) + 0.1 * (serialData[20] - 48) +  0.01 * (serialData[21] - 48);
      M1PID.Kd = kd1;
    }
    else if ((serialData.indexOf("PID2") >= 0 ) && (serialData.length() == 22)) {
      kp2 = 10 * (serialData[5] - 48)  +  (serialData[6] - 48) + 0.1 * (serialData[8] - 48)  +  0.01 * (serialData[9] - 48);
      M2PID.Kp = kp2;
      ki2 = 10 * (serialData[11] - 48) + (serialData[12] - 48) + 0.1 * (serialData[14] - 48) +  0.01 * (serialData[15] - 48);
      M2PID.Ki = ki2;
      kd2 = 10 * (serialData[17] - 48) + (serialData[18] - 48) + 0.1 * (serialData[20] - 48) +  0.01 * (serialData[21] - 48);
      M2PID.Kd = kd2;
    }
  }
}


void loop() {

  server.handleClient();

  if (stop_flag == false) {
    M1PID.input = M1_rpm_current;
    M2PID.input = M2_rpm_current;
    analogWrite(M1_PIN, M1PID.getResultTimer());
    analogWrite(M2_PIN, M2PID.getResultTimer());

    if ((millis() - previousMillis) > UPD_TIME) {
      M1_rpm_current = Enc1_read / ENC_COUNT_REV * rot2rpm;
      M2_rpm_current = Enc2_read / ENC_COUNT_REV * rot2rpm;
      Serial.print(M1_rpm_current);
      Serial.print(',');
      Serial.print(M1_rpm_target);
      Serial.print(',');
      Serial.print(M2_rpm_current);
      Serial.print(',');
      Serial.println(M2_rpm_target);
      Enc1_read = 0;
      Enc2_read = 0;
      previousMillis = millis();
    }
  }

  PIDparsing();
}
