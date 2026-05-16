#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

const char* ssid = "UAV_Dung_HUST"; //192.168.4.1
const char* password = "123456789";

AsyncWebServer server(80);

const int M1=17, M2=16, M3=26, M4=18; 
const int SDA_PIN=22, SCL_PIN=21;

Adafruit_MPU6050 mpu;
Servo motors[4];

int throttle = 1000;
float roll, pitch, yaw, r_off, p_off;
float manual_r_bias = 0, manual_p_bias = 0; 
float r_i, p_i, y_i, last_r_e, last_p_e, last_y_e;
float Kp = 5.2, Ki = 0.01, Kd = 1.2;
float Kp_y = 2, Ki_y = 0.001, Kd_y = 0.05; 


int mOut[4] = {1000, 1000, 1000, 1000};
unsigned long pTime;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { text-align: center; font-family: sans-serif; background: #1a1a1a; color: white; margin: 0; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; padding: 10px; }
  .card { background: #333; padding: 10px; border-radius: 10px; border: 1px solid #444; }
  .val { font-size: 1.2em; color: #00ff00; font-family: monospace; font-weight: bold; }
  .thr-container { background: #222; margin: 10px; padding: 15px; border-radius: 12px; border: 2px solid #28a745; }
  .thr-val { font-size: 2em; color: #28a745; font-family: monospace; font-weight: bold; }
  .motor-container { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; padding: 10px; background: #222; margin: 10px; border-radius: 12px; border: 1px solid #555; }
  .m-label { font-size: 0.7em; color: #aaa; }
  .m-val { font-size: 1.2em; color: #00d4ff; font-family: monospace; font-weight: bold; }
  .pid-box { background: #222; padding: 10px; margin: 5px; border-radius: 10px; font-size: 0.85em; }
  input { width: 42px; padding: 5px; text-align: center; background: #444; color: white; border: 1px solid #666; margin-bottom: 5px;}
  .btn { padding: 20px; font-weight: bold; border-radius: 15px; border: none; width: 45%; font-size: 1.1em; margin: 5px;}
  .up { background: #28a745; color: white; } .down { background: #ffc107; }
  .reset-btn { background: #00bcd4; color: white; width: 93%; padding: 12px; border-radius: 10px; border: none; font-weight: bold; }
  .stop { background: #ff4444; color: white; width: 93%; padding: 15px; border-radius: 10px; font-weight: bold; margin-top: 5px; }
</style></head>
<body>
  <div class="thr-container">
    <div style="font-size: 0.8em; color: #888;">GA TỔNG (THROTTLE)</div>
    <div id="thr" class="thr-val">1000</div>
  </div>

  <div class="motor-container">
    <div class="card"><div class="m-label">M1 (FR)</div><div id="m1" class="m-val">1000</div></div>
    <div class="card"><div class="m-label">M2 (FL)</div><div id="m2" class="m-val">1000</div></div>
    <div class="card"><div class="m-label">M3 (RL)</div><div id="m3" class="m-val">1000</div></div>
    <div class="card"><div class="m-label">M4 (RR)</div><div id="m4" class="m-val">1000</div></div>
  </div>

  <div class="grid">
    <div class="card">ROLL: <span id="r" class="val">0.0</span>°</div>
    <div class="card">PITCH: <span id="p" class="val">0.0</span>°</div>
  </div>

  <button class="reset-btn" onclick="fetch('/reset_gyro')">RESET GÓC (VỀ 0)</button>

  <div class="pid-box">
    <strong>R/P PID:</strong> P:<input id="kp" value="0.65"> I:<input id="ki" value="0.005"> D:<input id="kd" value="0.18"><br>
    <strong>YAW PID:</strong> P:<input id="kpy" value="1.2"> I:<input id="kiy" value="0.001"> D:<input id="kdy" value="0.05"><br>
    <button style="width:90%; background:#9c27b0; color:white; border:none; padding:8px; border-radius:5px; font-weight:bold" onclick="updatePID()">CẬP NHẬT 3 TRỤC</button>
  </div>

  <button class="btn up" onclick="fetch('/t?v=up')">GA +10</button>
  <button class="btn down" onclick="fetch('/t?v=down')">GA -10</button>
  <button class="stop" onclick="fetch('/s')">STOP</button>

  <script>
    function updatePID() {
      let p=document.getElementById('kp').value, i=document.getElementById('ki').value, d=document.getElementById('kd').value;
      let py=document.getElementById('kpy').value, iy=document.getElementById('kiy').value, dy=document.getElementById('kdy').value;
      fetch(`/set_pid?p=${p}&i=${i}&d=${d}&py=${py}&iy=${iy}&dy=${dy}`);
    }
    setInterval(() => {
      fetch('/data').then(res => res.json()).then(d => {
        document.getElementById('thr').innerText = d.t;
        document.getElementById('r').innerText = d.r.toFixed(1);
        document.getElementById('p').innerText = d.p.toFixed(1);
        for(let i=1;i<=4;i++) document.getElementById('m'+i).innerText = d['m'+i];
      });
    }, 200);
  </script>
</body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.begin();
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  for(int i=0; i<200; i++){
    sensors_event_t a, g, temp; mpu.getEvent(&a, &g, &temp);
    r_off += (atan2(a.acceleration.y, a.acceleration.z) * 57.3);
    p_off += (atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 57.3);
    delay(5);
  }
  r_off /= 200; p_off /= 200;

  WiFi.softAP(ssid, password);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *f){ f->send_P(200, "text/html", index_html); });
  server.on("/reset_gyro", HTTP_GET, [](AsyncWebServerRequest *f){ manual_r_bias = roll; manual_p_bias = pitch; f->send(200); });
  
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *f){
    String json = "{\"t\":"+String(throttle)+",\"r\":"+String(roll-manual_r_bias,1)+",\"p\":"+String(pitch-manual_p_bias,1)+
                  ",\"m1\":"+String(mOut[0])+",\"m2\":"+String(mOut[1])+",\"m3\":"+String(mOut[2])+",\"m4\":"+String(mOut[3])+"}";
    f->send(200, "application/json", json);
  });

  server.on("/set_pid", HTTP_GET, [](AsyncWebServerRequest *f){
    if(f->hasParam("p")) Kp = f->getParam("p")->value().toFloat();
    if(f->hasParam("i")) Ki = f->getParam("i")->value().toFloat();
    if(f->hasParam("d")) Kd = f->getParam("d")->value().toFloat();
    if(f->hasParam("py")) Kp_y = f->getParam("py")->value().toFloat();
    if(f->hasParam("iy")) Ki_y = f->getParam("iy")->value().toFloat();
    if(f->hasParam("dy")) Kd_y = f->getParam("dy")->value().toFloat();
    Serial.printf("Update PID: R/P(%.2f, %.3f, %.2f) | Yaw(%.2f, %.3f, %.2f)\n", Kp, Ki, Kd, Kp_y, Ki_y, Kd_y);
    f->send(200);
  });

  server.on("/t", HTTP_GET, [](AsyncWebServerRequest *f){
    if(f->getParam("v")->value() == "up") throttle += 10; else throttle -= 10;
    throttle = constrain(throttle, 1000, 1850); f->send(200);
  });

  server.on("/s", HTTP_GET, [](AsyncWebServerRequest *f){ 
    throttle = 1000; for(int i=0; i<4; i++) { mOut[i]=1000; motors[i].writeMicroseconds(1000); } f->send(200); 
  });

  server.begin();

  int pins[] = {M1, M2, M3, M4};
  ESP32PWM::allocateTimer(0);
  for(int i=0; i<4; i++) {
    motors[i].attach(pins[i], 1000, 2000); 
    motors[i].writeMicroseconds(1000); 
  }
  pTime = micros();
}

void loop() {
  sensors_event_t a, g, temp; mpu.getEvent(&a, &g, &temp);
  float dt = (micros() - pTime) / 1000000.0; if (dt <= 0 || dt > 0.1) dt = 0.01; pTime = micros();
 
  roll = 0.98 * (roll + g.gyro.x * 57.3 * dt) + 0.02 * ((atan2(a.acceleration.y, a.acceleration.z) * 57.3) - r_off);
  pitch = 0.98 * (pitch + g.gyro.y * 57.3 * dt) + 0.02 * ((atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 57.3) - p_off);
  yaw = g.gyro.z * 57.3;

  float f_r = roll - manual_r_bias;
  float f_p = pitch - manual_p_bias;

  if (throttle < 1120) {
    for(int i=0; i<4; i++) { mOut[i] = 1000; motors[i].writeMicroseconds(1000); }
    r_i = p_i = y_i = last_r_e = last_p_e = last_y_e = 0;
  } else {
    float rE = 0 - f_r, pE = 0 - f_p, yE = 0 - yaw;
    r_i = constrain(r_i + rE * dt, -50, 50); 
    p_i = constrain(p_i + pE * dt, -50, 50);
    y_i = constrain(y_i + yE * dt, -50, 50);
    
    float rO = (Kp * rE) + (Ki * r_i) + (Kd * (rE - last_r_e)/dt);
    float pO = (Kp * pE) + (Ki * p_i) + (Kd * (pE - last_p_e)/dt);
    float yO = (Kp_y * yE) + (Ki_y * y_i) + (Kd_y * (yE - last_y_e)/dt);
    
    last_r_e = rE; last_p_e = pE; last_y_e = yE;

    int base = throttle+100 ;
    mOut[0] = constrain(base - pO + rO - yO +40 , 1100, 1950); 
    mOut[1] = constrain(base - pO - rO + yO+ 80   , 1100, 1950); 
    mOut[2] = constrain(base + pO - rO - yO   , 1100, 1950); 
    mOut[3] = constrain(base + pO + rO + yO, 1100, 1950); 

    for(int i=0; i<4; i++) motors[i].writeMicroseconds(mOut[i]);
  }
}