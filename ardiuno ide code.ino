// esp32_robot_web_ui.ino
// Modern Web UI + Adaptive analogWrite speed using two front ultrasonics.
// Uses WiFi + WebServer. Adjust pins if your wiring differs.

#include <WiFi.h>
#include <WebServer.h>

// ---------- WiFi ----------
const char* ssid = "Dhanush's M34 5G";
const char* password = "7569576743";

// ---------- Motor pins ----------
#define IN1 32
#define IN2 33
#define EN_A 25
#define IN3 18
#define IN4 19
#define EN_B 26

// ---------- Ultrasonic pins (front-left, front-right) ----------
#define TRIG_LEFT 12
#define ECHO_LEFT 13
#define TRIG_RIGHT 4
#define ECHO_RIGHT 5

// ---------- Movement timing ----------
unsigned long movementEndMs = 0;
bool movementActive = false;
char movementCmd = 'S';

const unsigned long MOVE_FORWARD_MS = 500;
const unsigned long MOVE_TURN_MS = 420;
const unsigned long MOVE_BACK_MS = 500;

// ---------- Speed ----------
int currentSpeed = 200; // 0..255
int lastSpeedSent = -1;

// ---------- Web server ----------
WebServer server(80);

// ---------- Helpers ----------
void motorStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(EN_A, 0);
  analogWrite(EN_B, 0);
}

void motorForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN_A, currentSpeed);
  analogWrite(EN_B, currentSpeed);
}

void motorBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(EN_A, currentSpeed);
  analogWrite(EN_B, currentSpeed);
}

void motorTurnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN_A, currentSpeed);
  analogWrite(EN_B, currentSpeed);
}

void motorTurnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(EN_A, currentSpeed);
  analogWrite(EN_B, currentSpeed);
}

void startMovement(char cmd) {
  movementCmd = cmd;
  if (cmd == 'F') {
    motorForward();
    movementEndMs = millis() + MOVE_FORWARD_MS;
    movementActive = true;
  } else if (cmd == 'B') {
    motorBackward();
    movementEndMs = millis() + MOVE_BACK_MS;
    movementActive = true;
  } else if (cmd == 'L') {
    motorTurnLeft();
    movementEndMs = millis() + MOVE_TURN_MS;
    movementActive = true;
  } else if (cmd == 'R') {
    motorTurnRight();
    movementEndMs = millis() + MOVE_TURN_MS;
    movementActive = true;
  } else {
    motorStop();
    movementActive = false;
    movementCmd = 'S';
  }
}

// ultrasonic reading (robust median of 3)
float getDistanceRaw(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 23000); // 23ms max (~4m)
  if (dur == 0) return 400.0;
  float d = dur * 0.0343 / 2.0;
  if (d <= 0 || d > 400) return 400.0;
  return d;
}

float getDistanceFiltered(int trigPin, int echoPin) {
  float a = getDistanceRaw(trigPin, echoPin);
  delay(5);
  float b = getDistanceRaw(trigPin, echoPin);
  delay(5);
  float c = getDistanceRaw(trigPin, echoPin);
  float arr[3] = {a,b,c};
  // median
  for (int i=0;i<2;i++){
    for (int j=i+1;j<3;j++){
      if (arr[j] < arr[i]){
        float t = arr[i]; arr[i] = arr[j]; arr[j] = t;
      }
    }
  }
  return arr[1];
}

// adaptive speed mapping (nearest cm -> PWM 0..255)
int computeAdaptiveSpeed(float leftCm, float rightCm) {
  float nearest = min(leftCm, rightCm);
  // fallback safe mapping: far -> high speed; near -> low speed
  if (nearest > 150.0) return 255;
  if (nearest > 80.0) return 220;
  if (nearest > 50.0) return 180;
  if (nearest > 30.0) return 140;
  if (nearest > 20.0) return 100;
  return 70; // very close -> crawl
}

// ---------- HTML UI (compact, dark, with sensor bars) ----------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Control</title>
<style>
:root{--bg:#0b1220;--card:#0f1724;--accent:#2dd4bf;--muted:#9ca3af;--red:#ef4444}
*{box-sizing:border-box;font-family:Arial,Helvetica,sans-serif}
body{margin:12px;background:linear-gradient(135deg,#071029 0%,#0b1220 100%);color:#e6eef5}
.container{max-width:820px;margin:0 auto}
.header{display:flex;align-items:center;gap:12px;justify-content:space-between}
.h-title{font-size:18px;font-weight:700}
.grid{display:grid;grid-template-columns:1fr 320px;gap:12px;margin-top:12px}
.card{background:var(--card);padding:12px;border-radius:10px;box-shadow:0 6px 18px rgba(2,6,23,0.6)}
.sensors{display:flex;flex-direction:column;gap:8px}
.sensor{display:flex;align-items:center;gap:8px}
.label{width:60px;color:var(--muted);font-size:13px}
.bar{flex:1;height:14px;background:#06202a;border-radius:8px;overflow:hidden}
.fill{height:100%;background:linear-gradient(90deg,#06b6d4,#60a5fa);width:10%;transition:width .25s}
.val{width:60px;text-align:right;font-weight:700}
.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:8px}
.btn{background:linear-gradient(90deg,#0ea5a7,#2563eb);color:#021124;padding:12px;border-radius:10px;border:none;font-weight:700;cursor:pointer}
.btn.big{grid-column:1/span3;background:#ef4444;color:white}
.pad{display:grid;grid-template-areas:". u ." "l o r" ". d .";gap:6px;justify-items:center;align-items:center}
.pad button{width:64px;height:44px;border-radius:8px;border:none;background:#0b1220;color:#e6eef5;font-weight:800}
.status{margin-top:8px;font-size:14px}
.small{font-size:13px;color:var(--muted)}
.speed-row{display:flex;align-items:center;gap:10px;margin-top:8px}
.slider{flex:1}
.footer{margin-top:12px;text-align:center;color:var(--muted);font-size:12px}
</style></head><body>
<div class="container">
  <div class="header">
    <div class="h-title">🤖 Robot Control — Adaptive Speed (analogWrite)</div>
    <div class="small">Status: <span id="stat">Ready</span></div>
  </div>

  <div class="grid">
    <div>
      <div class="card">
        <div style="font-weight:700;margin-bottom:8px">Sensors</div>
        <div class="sensors">
          <div class="sensor"><div class="label">LEFT</div><div class="bar"><div id="leftFill" class="fill"></div></div><div class="val" id="leftVal">--</div></div>
          <div class="sensor"><div class="label">RIGHT</div><div class="bar"><div id="rightFill" class="fill"></div></div><div class="val" id="rightVal">--</div></div>
        </div>
        <div class="small" style="margin-top:10px">Nearest sensor controls adaptive speed</div>
      </div>

      <div class="card" style="margin-top:10px">
        <div style="font-weight:700;margin-bottom:6px">Controls</div>
        <div class="pad" style="margin-top:6px">
          <div></div>
          <button onclick="send('F')">▲</button>
          <div></div>
          <button onclick="send('L')">◀</button>
          <button id="stopBtn" onclick="send('S')">■</button>
          <button onclick="send('R')">▶</button>
          <div></div>
          <button onclick="send('B')">▼</button>
        </div>
        <div style="margin-top:10px" class="speed-row">
          <div class="small">Speed</div>
          <input id="speed" type="range" min="0" max="255" value="200" class="slider" oninput="updateSpeedVal(this.value)">
          <div id="speedVal" style="width:44px;text-align:right;font-weight:700">200</div>
        </div>
        <button class="btn big" onclick="stopNow()" style="margin-top:10px">EMERGENCY STOP</button>
      </div>
    </div>

    <div>
      <div class="card">
        <div style="font-weight:700;margin-bottom:8px">Live</div>
        <div id="log" style="height:240px;overflow:auto;background:#061022;border-radius:6px;padding:8px;color:#a7f3d0;font-family:monospace;font-size:13px"></div>
        <div style="margin-top:8px" class="small">Open this page from any device on the same WiFi. The UI pulls /ultrasonic and /status regularly.</div>
      </div>
    </div>
  </div>

  <div class="footer">ESP32 Robot — Adaptive analogWrite speed | Two front ultrasonics</div>
</div>

<script>
let logEl = document.getElementById('log');
function log(s){ let t = '['+new Date().toLocaleTimeString()+'] '+s; logEl.innerHTML = t + '<br>' + logEl.innerHTML; }
async function updateSensors(){
  try{
    const r = await fetch('/ultrasonic');
    const j = await r.json();
    const left = j.left, right = j.right;
    document.getElementById('leftVal').innerText = left.toFixed(1)+'cm';
    document.getElementById('rightVal').innerText = right.toFixed(1)+'cm';
    document.getElementById('leftFill').style.width = Math.min(left/4,100) + '%';
    document.getElementById('rightFill').style.width = Math.min(right/4,100) + '%';
    // log short entry
    log('Sensors L:'+left.toFixed(1)+' R:'+right.toFixed(1));
  }catch(e){}
}
async function updateStatus(){
  try{
    const r = await fetch('/status'); const s = await r.text();
    document.getElementById('stat').innerText = s;
  }catch(e){}
}
async function send(c){
  try{
    await fetch('/move?dir='+c);
    document.getElementById('stopBtn').innerText = c==='S'?'■':'STOP';
    log('Sent: '+c);
  }catch(e){ alert('Failed to send'); }
}
async function stopNow(){ try{ await fetch('/stop'); log('STOPPED'); }catch(e){} }
function updateSpeedVal(v){ document.getElementById('speedVal').innerText = v; }
document.getElementById('speed').addEventListener('change', async (ev)=> {
  const v = ev.target.value;
  try{ await fetch('/speed?val='+v); log('Speed set '+v); }catch(e){ log('Speed set failed'); }
});
setInterval(updateSensors, 450);
setInterval(updateStatus, 1200);
updateSensors(); updateStatus();
</script>
</body></html>
)rawliteral";

// ---------- Handlers ----------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleMove() {
  String dir = server.arg("dir");
  if (dir.length() == 0) { server.send(400, "text/plain", "Missing dir"); return; }
  char d = dir.charAt(0);
  if (d == 'F' || d == 'B' || d == 'L' || d == 'R' || d == 'S') {
    startMovement(d);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad dir");
  }
}

void handleSpeed() {
  String val = server.arg("val");
  if (val.length() == 0) { server.send(400, "text/plain", "Missing val"); return; }
  int v = val.toInt();
  if (v < 0) v = 0; if (v > 255) v = 255;
  currentSpeed = v;
  analogWrite(EN_A, currentSpeed);
  analogWrite(EN_B, currentSpeed);
  server.send(200, "text/plain", String(currentSpeed));
}

void handleUltrasonic() {
  float l = getDistanceFiltered(TRIG_LEFT, ECHO_LEFT);
  float r = getDistanceFiltered(TRIG_RIGHT, ECHO_RIGHT);
  String json = "{\"left\":"+String(l,1)+",\"right\":"+String(r,1)+"}";
  server.send(200, "application/json", json);
}

void handleStatus() {
  server.send(200, "text/plain", movementActive ? "BUSY" : "READY");
}

void handleStop() {
  motorStop();
  movementActive = false;
  movementCmd = 'S';
  movementEndMs = 0;
  server.send(200, "text/plain", "STOPPED");
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(EN_A, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(EN_B, OUTPUT);

  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  motorStop();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(300); Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected. IP: "+WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi failed.");
  }

  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/speed", handleSpeed);
  server.on("/ultrasonic", handleUltrasonic);
  server.on("/status", handleStatus);
  server.on("/stop", handleStop);

  server.begin();
  Serial.println("Server started");
}

// ---------- Loop ----------
void loop() {
  server.handleClient();

  // compute adaptive speed and update periodically (even when not moving)
  float leftCm = getDistanceFiltered(TRIG_LEFT, ECHO_LEFT);
  float rightCm = getDistanceFiltered(TRIG_RIGHT, ECHO_RIGHT);
  int adaptive = computeAdaptiveSpeed(leftCm, rightCm);
  // smooth changes toward adaptive value
  if (abs(adaptive - currentSpeed) > 2) {
    if (adaptive > currentSpeed) currentSpeed += 2;
    else currentSpeed -= 2;
  } else {
    currentSpeed = adaptive;
  }
  // push new speed to motors (if they are running, set immediately)
  analogWrite(EN_A, currentSpeed);
  analogWrite(EN_B, currentSpeed);

  // if movement time expired, stop motors
  if (movementActive && (long)(millis() - movementEndMs) >= 0) {
    motorStop();
    movementActive = false;
    movementCmd = 'S';
  }

  delay(25);
}