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
float gx_off, gy_off, gz_off;           // gyro bias offsets
float manual_r_bias = 0, manual_p_bias = 0; 

// --- Cascaded PID gains ---
// Outer loop: angle → rate setpoint
float Kp_out = 4.5, Ki_out = 0.0, Kd_out = 0.0;   // typically D=0 for outer loop

// Inner loop: rate → motor output
float Kp_in = 1.5, Ki_in = 0.01, Kd_in = 0.05;

// Yaw: rate controller only (gyro rate → output)
float Kp_y = 2.0, Ki_y = 0.001, Kd_y = 0.05;

// Inner loop integrators
float ri_in, pi_in, yi_in;
// Outer loop integrators
float ri_out, pi_out;

// Previous errors
float last_rE_in, last_pE_in, last_yE;
float last_rE_out, last_pE_out;

// D-term EMA filtered values (inner loop)
float d_roll_filtered, d_pitch_filtered, d_yaw_filtered;
const float D_ALPHA = 0.4; // EMA coefficient for D-term LPF (0=more filtering, 1=no filtering)

int mOut[4] = {1000, 1000, 1000, 1000};

// Shared variable protection
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile int throttle_shared = 1000;
volatile float Kp_out_s, Ki_out_s, Kd_out_s;
volatile float Kp_in_s,  Ki_in_s,  Kd_in_s;
volatile float Kp_y_s,   Ki_y_s,   Kd_y_s;
volatile bool  gains_updated = false;

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
  .pid-section { margin-bottom: 8px; border-bottom: 1px solid #444; padding-bottom: 8px; }
  input { width: 42px; padding: 5px; text-align: center; background: #444; color: white; border: 1px solid #666; margin-bottom: 5px;}
  .btn { padding: 20px; font-weight: bold; border-radius: 15px; border: none; width: 45%; font-size: 1.1em; margin: 5px;}
  .up { background: #28a745; color: white; } .down { background: #ffc107; }
  .reset-btn { background: #00bcd4; color: white; width: 93%; padding: 12px; border-radius: 10px; border: none; font-weight: bold; }
  .stop { background: #ff4444; color: white; width: 93%; padding: 15px; border-radius: 10px; font-weight: bold; margin-top: 5px; }
  .loop-badge { font-size: 0.7em; color: #888; margin-top: 4px; }
</style></head>
<body>
  <div class="thr-container">
    <div style="font-size: 0.8em; color: #888;">GA TỔNG (THROTTLE)</div>
    <div id="thr" class="thr-val">1000</div>
    <div class="loop-badge">Loop dt: <span id="dt">--</span> ms</div>
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
    <div class="pid-section">
      <strong>OUTER LOOP (Angle → Rate):</strong><br>
      P:<input id="kp_out" value="4.5"> I:<input id="ki_out" value="0.0"> D:<input id="kd_out" value="0.0">
    </div>
    <div class="pid-section">
      <strong>INNER LOOP R/P (Rate → Motor):</strong><br>
      P:<input id="kp_in" value="1.5"> I:<input id="ki_in" value="0.01"> D:<input id="kd_in" value="0.05">
    </div>
    <div class="pid-section">
      <strong>YAW RATE:</strong><br>
      P:<input id="kpy" value="2.0"> I:<input id="kiy" value="0.001"> D:<input id="kdy" value="0.05">
    </div>
    <button style="width:90%; background:#9c27b0; color:white; border:none; padding:8px; border-radius:5px; font-weight:bold" onclick="updatePID()">CẬP NHẬT PID</button>
  </div>

  <button class="btn up" onclick="fetch('/t?v=up')">GA +10</button>
  <button class="btn down" onclick="fetch('/t?v=down')">GA -10</button>
  <button class="stop" onclick="fetch('/s')">STOP</button>

  <script>
    function updatePID() {
      let p_out=document.getElementById('kp_out').value, i_out=document.getElementById('ki_out').value, d_out=document.getElementById('kd_out').value;
      let p_in=document.getElementById('kp_in').value,  i_in=document.getElementById('ki_in').value,  d_in=document.getElementById('kd_in').value;
      let py=document.getElementById('kpy').value, iy=document.getElementById('kiy').value, dy=document.getElementById('kdy').value;
      fetch(`/set_pid?p_out=${p_out}&i_out=${i_out}&d_out=${d_out}&p_in=${p_in}&i_in=${i_in}&d_in=${d_in}&py=${py}&iy=${iy}&dy=${dy}`);
    }
    setInterval(() => {
      fetch('/data').then(res => res.json()).then(d => {
        document.getElementById('thr').innerText = d.t;
        document.getElementById('r').innerText = d.r.toFixed(1);
        document.getElementById('p').innerText = d.p.toFixed(1);
        document.getElementById('dt').innerText = d.dt.toFixed(2);
        for(let i=1;i<=4;i++) document.getElementById('m'+i).innerText = d['m'+i];
      });
    }, 200);
  </script>
</body></html>)rawliteral";

// ─── PID Task pinned to Core 1 ───────────────────────────────────────────────
void pidTask(void *param) {
  pTime = micros();
  for (;;) {
    // Pull in any gain updates from WiFi core safely
    if (gains_updated) {
      portENTER_CRITICAL(&mux);
      Kp_out = Kp_out_s; Ki_out = Ki_out_s; Kd_out = Kd_out_s;
      Kp_in  = Kp_in_s;  Ki_in  = Ki_in_s;  Kd_in  = Kd_in_s;
      Kp_y   = Kp_y_s;   Ki_y   = Ki_y_s;   Kd_y   = Kd_y_s;
      gains_updated = false;
      portEXIT_CRITICAL(&mux);
    }

    portENTER_CRITICAL(&mux);
    int thr = throttle_shared;
    portEXIT_CRITICAL(&mux);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = micros();
    float dt = (now - pTime) / 1000000.0;
    if (dt <= 0 || dt > 0.05) dt = 0.004; // guard: clamp to 4ms default (250Hz)
    pTime = now;

    // ── Gyro rates (deg/s), bias-corrected ──
    float gx = g.gyro.x * 57.3 - gx_off;
    float gy = g.gyro.y * 57.3 - gy_off;
    float gz = g.gyro.z * 57.3 - gz_off;

    // ── Complementary filter for angle estimate (outer loop only) ──
    float tau   = 0.5;
    float alpha = tau / (tau + dt);   // dt-adaptive coefficient
    float accel_roll  = atan2(a.acceleration.y, a.acceleration.z) * 57.3 - r_off;
    float accel_pitch = atan2(-a.acceleration.x,
                        sqrt(a.acceleration.y*a.acceleration.y +
                             a.acceleration.z*a.acceleration.z)) * 57.3 - p_off;
    roll  = alpha * (roll  + gx * dt) + (1.0 - alpha) * accel_roll;
    pitch = alpha * (pitch + gy * dt) + (1.0 - alpha) * accel_pitch;

    float f_r = roll  - manual_r_bias;
    float f_p = pitch - manual_p_bias;

    if (thr < 1120) {
      for (int i = 0; i < 4; i++) { mOut[i] = 1000; motors[i].writeMicroseconds(1000); }
      ri_in = pi_in = yi_in = 0;
      ri_out = pi_out = 0;
      last_rE_in = last_pE_in = last_yE = 0;
      last_rE_out = last_pE_out = 0;
      d_roll_filtered = d_pitch_filtered = d_yaw_filtered = 0;
    } else {

      // ── OUTER LOOP: angle error → desired rate (deg/s) ──
      float rE_out = 0 - f_r;   // target angle = 0°
      float pE_out = 0 - f_p;

      ri_out = constrain(ri_out + rE_out * dt, -30, 30);
      pi_out = constrain(pi_out + pE_out * dt, -30, 30);

      float d_out_r_raw = (rE_out - last_rE_out) / dt;
      float d_out_p_raw = (pE_out - last_pE_out) / dt;
      last_rE_out = rE_out; last_pE_out = pE_out;

      float roll_rate_sp  = (Kp_out * rE_out) + (Ki_out * ri_out) + (Kd_out * d_out_r_raw);
      float pitch_rate_sp = (Kp_out * pE_out) + (Ki_out * pi_out) + (Kd_out * d_out_p_raw);

      // Clamp rate setpoint to safe range (deg/s)
      roll_rate_sp  = constrain(roll_rate_sp,  -200, 200);
      pitch_rate_sp = constrain(pitch_rate_sp, -200, 200);

      // ── INNER LOOP: rate error → motor output ──
      // Uses raw gyro directly — no complementary filter phase lag
      float rE_in = roll_rate_sp  - gx;
      float pE_in = pitch_rate_sp - gy;
      float yE    = 0             - gz;   // yaw: hold zero rotation rate

      ri_in = constrain(ri_in + rE_in * dt, -50, 50);
      pi_in = constrain(pi_in + pE_in * dt, -50, 50);
      yi_in = constrain(yi_in + yE    * dt, -50, 50);

      // D-term with EMA low-pass filter on inner loop
      float d_r_raw = (rE_in - last_rE_in) / dt;
      float d_p_raw = (pE_in - last_pE_in) / dt;
      float d_y_raw = (yE    - last_yE)    / dt;
      d_roll_filtered  = D_ALPHA * d_r_raw + (1.0 - D_ALPHA) * d_roll_filtered;
      d_pitch_filtered = D_ALPHA * d_p_raw + (1.0 - D_ALPHA) * d_pitch_filtered;
      d_yaw_filtered   = D_ALPHA * d_y_raw + (1.0 - D_ALPHA) * d_yaw_filtered;
      last_rE_in = rE_in; last_pE_in = pE_in; last_yE = yE;

      float rO = (Kp_in * rE_in) + (Ki_in * ri_in) + (Kd_in * d_roll_filtered);
      float pO = (Kp_in * pE_in) + (Ki_in * pi_in) + (Kd_in * d_pitch_filtered);
      float yO = (Kp_y  * yE)    + (Ki_y  * yi_in) + (Kd_y  * d_yaw_filtered);

      int base = thr + 100;
      mOut[0] = constrain(base - pO + rO - yO, 1100, 1950); // FR
      mOut[1] = constrain(base - pO - rO + yO, 1100, 1950); // FL
      mOut[2] = constrain(base + pO - rO - yO, 1100, 1950); // RL
      mOut[3] = constrain(base + pO + rO + yO, 1100, 1950); // RR

      for (int i = 0; i < 4; i++) motors[i].writeMicroseconds(mOut[i]);
    }

    vTaskDelay(pdMS_TO_TICKS(4)); // 250 Hz target
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.begin();
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ── Calibration: accel offsets + gyro bias ──
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp; mpu.getEvent(&a, &g, &temp);
    r_off  += (atan2(a.acceleration.y, a.acceleration.z) * 57.3);
    p_off  += (atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 57.3);
    gx_off += g.gyro.x * 57.3;
    gy_off += g.gyro.y * 57.3;
    gz_off += g.gyro.z * 57.3;
    delay(5);
  }
  r_off  /= 200; p_off  /= 200;
  gx_off /= 200; gy_off /= 200; gz_off /= 200;

  WiFi.softAP(ssid, password);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *f){ f->send_P(200, "text/html", index_html); });
  server.on("/reset_gyro", HTTP_GET, [](AsyncWebServerRequest *f){ manual_r_bias = roll; manual_p_bias = pitch; f->send(200); });
  
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *f){
    unsigned long now = micros();
    float dt_ms = (now - pTime) / 1000.0;
    String json = "{\"t\":"+String(throttle_shared)+
                  ",\"r\":"+String(roll-manual_r_bias,1)+
                  ",\"p\":"+String(pitch-manual_p_bias,1)+
                  ",\"dt\":"+String(dt_ms,2)+
                  ",\"m1\":"+String(mOut[0])+
                  ",\"m2\":"+String(mOut[1])+
                  ",\"m3\":"+String(mOut[2])+
                  ",\"m4\":"+String(mOut[3])+"}";
    f->send(200, "application/json", json);
  });

  server.on("/set_pid", HTTP_GET, [](AsyncWebServerRequest *f){
    portENTER_CRITICAL(&mux);
    if(f->hasParam("p_out")) Kp_out_s = f->getParam("p_out")->value().toFloat();
    if(f->hasParam("i_out")) Ki_out_s = f->getParam("i_out")->value().toFloat();
    if(f->hasParam("d_out")) Kd_out_s = f->getParam("d_out")->value().toFloat();
    if(f->hasParam("p_in"))  Kp_in_s  = f->getParam("p_in")->value().toFloat();
    if(f->hasParam("i_in"))  Ki_in_s  = f->getParam("i_in")->value().toFloat();
    if(f->hasParam("d_in"))  Kd_in_s  = f->getParam("d_in")->value().toFloat();
    if(f->hasParam("py"))    Kp_y_s   = f->getParam("py")->value().toFloat();
    if(f->hasParam("iy"))    Ki_y_s   = f->getParam("iy")->value().toFloat();
    if(f->hasParam("dy"))    Kd_y_s   = f->getParam("dy")->value().toFloat();
    gains_updated = true;
    portEXIT_CRITICAL(&mux);
    Serial.printf("PID update queued: Outer(%.2f,%.3f,%.2f) Inner(%.2f,%.3f,%.2f) Yaw(%.2f,%.3f,%.2f)\n",
      Kp_out_s, Ki_out_s, Kd_out_s, Kp_in_s, Ki_in_s, Kd_in_s, Kp_y_s, Ki_y_s, Kd_y_s);
    f->send(200);
  });

  server.on("/t", HTTP_GET, [](AsyncWebServerRequest *f){
    portENTER_CRITICAL(&mux);
    if(f->getParam("v")->value() == "up") throttle_shared += 10; else throttle_shared -= 10;
    throttle_shared = constrain(throttle_shared, 1000, 1850);
    portEXIT_CRITICAL(&mux);
    f->send(200);
  });

  server.on("/s", HTTP_GET, [](AsyncWebServerRequest *f){ 
    portENTER_CRITICAL(&mux);
    throttle_shared = 1000;
    portEXIT_CRITICAL(&mux);
    f->send(200); 
  });

  server.begin();

  int pins[] = {M1, M2, M3, M4};
  ESP32PWM::allocateTimer(0);
  for(int i=0; i<4; i++) {
    motors[i].attach(pins[i], 1000, 2000); 
    motors[i].writeMicroseconds(1000); 
  }

  // Launch PID loop on Core 1; WiFi stays on Core 0
  xTaskCreatePinnedToCore(pidTask, "PID", 8192, NULL, 1, NULL, 1);
}

void loop() {
  // Intentionally empty — all flight control runs in pidTask on Core 1
  vTaskDelay(pdMS_TO_TICKS(1000));
}
