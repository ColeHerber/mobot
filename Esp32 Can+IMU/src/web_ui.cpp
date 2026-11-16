#include "web_ui.h"
#include "pwm_out.h"
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <stdarg.h>

// ---------------------------------------------------------------------------
// Debug log ring buffer
// ---------------------------------------------------------------------------

static constexpr uint8_t LOG_MAX = 50;
static String   logBuf[LOG_MAX];
static uint32_t logSeqBuf[LOG_MAX];
static uint8_t  logHead = 0;
static uint8_t  logCount = 0;
static uint32_t logNextSeq = 1;

void dbgLog(const char* fmt, ...) {
    char tmp[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    Serial.println(tmp);
    Serial0.println(tmp);
    logBuf[logHead]    = String(tmp);
    logSeqBuf[logHead] = logNextSeq++;
    logHead = (logHead + 1) % LOG_MAX;
    if (logCount < LOG_MAX) logCount++;
}

// ---------------------------------------------------------------------------
// HTML dashboard (served from flash)
// ---------------------------------------------------------------------------

static const char HTML_PAGE[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 CAN+IMU</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d0d0d;color:#ddd;font:12px/1.5 monospace;padding:8px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:8px}
.wide{grid-column:span 2}
@media(max-width:640px){.wide{grid-column:span 1}}
.card{background:#181818;border:1px solid #2a2a2a;border-radius:6px;padding:10px}
h2{font-size:11px;color:#666;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px;padding-bottom:5px;border-bottom:1px solid #2a2a2a}
.row{display:flex;justify-content:space-between;align-items:center;padding:2px 0}
.lbl{color:#777;font-size:11px}
.val{color:#3df;font-weight:bold}
canvas{display:block;width:100%;background:#0d0d0d;border-radius:3px;margin-top:6px}
input[type=range]{width:100%;accent-color:#3df;margin:3px 0}
.btn{background:#222;border:1px solid #444;color:#ccc;padding:4px 12px;border-radius:4px;cursor:pointer;font:inherit;font-size:11px}
.btn:hover{background:#333}
input[type=text]{background:#0d0d0d;border:1px solid #333;color:#ddd;padding:3px 6px;border-radius:3px;font:inherit;font-size:11px;width:100%;margin:2px 0}
.clog{height:130px;overflow-y:auto;font-size:10px;background:#0d0d0d;border-radius:3px;padding:4px;border:1px solid #222}
.cline{color:#3df;border-bottom:1px solid #111;padding:1px 0}
.bwrap{background:#0d0d0d;border-radius:2px;height:13px;margin:3px 0;overflow:hidden;border:1px solid #1a1a1a}
.bar{height:100%;background:linear-gradient(90deg,#0a3028,#3df);transition:width .15s}
.dot{display:inline-block;width:7px;height:7px;border-radius:50%;margin-right:4px;vertical-align:middle}
.gr{background:#3c4}.re{background:#c33}.ye{background:#ca3}
.prow{margin:7px 0}
</style>
</head>
<body>
<div class="grid">

<div class="card">
<h2>&#9881; System</h2>
<div class="row"><span class="lbl">IMU</span><span class="val"><span class="dot ye" id="idot"></span><span id="itxt">—</span></span></div>
<div class="row"><span class="lbl">WHO_AM_I</span><span class="val" id="iwho">—</span></div>
<div class="row"><span class="lbl">WiFi RSSI</span><span class="val" id="srssi">—</span></div>
<div class="row"><span class="lbl">Free heap</span><span class="val" id="sheap">—</span></div>
<div class="row"><span class="lbl">Uptime</span><span class="val" id="sup">—</span></div>
<div class="row"><span class="lbl">CAN TX / TX err</span><span class="val" id="scantx">—</span></div>
<div class="row"><span class="lbl">CAN RX / RX err</span><span class="val" id="scanrx">—</span></div>
<div class="row"><span class="lbl">SSE rate</span><span class="val" id="srate">—</span></div>
</div>

	<div class="card wide">
	<h2>&#128200; IMU - <span id="imodel">—</span> (SPI2)</h2>
<div style="display:grid;grid-template-columns:1fr 1fr;gap:12px">
<div>
<div class="row"><span class="lbl">Accel X</span><span class="val" id="iax">—</span></div>
<div class="row"><span class="lbl">Accel Y</span><span class="val" id="iay">—</span></div>
<div class="row"><span class="lbl">Accel Z</span><span class="val" id="iaz">—</span></div>
</div>
<div>
<div class="row"><span class="lbl">Gyro X</span><span class="val" id="igx">—</span></div>
<div class="row"><span class="lbl">Gyro Y</span><span class="val" id="igy">—</span></div>
<div class="row"><span class="lbl">Gyro Z</span><span class="val" id="igz">—</span></div>
</div>
</div>
<canvas id="cv" height="120"></canvas>
<div style="font-size:10px;color:#555;margin-top:3px">
<span style="color:#f44">&#9632;</span>aX&nbsp;
<span style="color:#fa4">&#9632;</span>aY&nbsp;
<span style="color:#4f4">&#9632;</span>aZ&nbsp;&nbsp;
<span style="color:#48f">&#9632;</span>gX&nbsp;
<span style="color:#d4f">&#9632;</span>gY&nbsp;
<span style="color:#4ff">&#9632;</span>gZ&nbsp;&nbsp;
	(accel graph &#177;2&nbsp;g, sensor FS &#177;16&nbsp;g / gyro graph &#177;500&nbsp;&#176;/s)
	</div>
	</div>
	
	<div class="card wide">
	<h2>&#9889; High-G Accelerometer</h2>
	<div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px">
	<div class="row"><span class="lbl">High-G X</span><span class="val" id="ihgx">—</span></div>
	<div class="row"><span class="lbl">High-G Y</span><span class="val" id="ihgy">—</span></div>
	<div class="row"><span class="lbl">High-G Z</span><span class="val" id="ihgz">—</span></div>
	<div class="row"><span class="lbl">Raw X</span><span class="val" id="irhgx">—</span></div>
	<div class="row"><span class="lbl">Raw Y</span><span class="val" id="irhgy">—</span></div>
	<div class="row"><span class="lbl">Raw Z</span><span class="val" id="irhgz">—</span></div>
	</div>
	<canvas id="hcv" height="120"></canvas>
	<div style="font-size:10px;color:#555;margin-top:3px">
	<span style="color:#f55">&#9632;</span>hX&nbsp;
	<span style="color:#5f5">&#9632;</span>hY&nbsp;
	<span style="color:#59f">&#9632;</span>hZ&nbsp;&nbsp;
	(graph &#177;16&nbsp;g zoom; sensor FS &#177;256&nbsp;g)
	</div>
	</div>
	
	<div class="card">
	<h2>&#128295; PWM Outputs (50 Hz servo)</h2>
<div class="prow">
<div class="row"><span class="lbl">CH1 &mdash; J7 (IO14)</span><span class="val" id="pv0">1500 µs</span></div>
<input type="range" id="ps0" min="500" max="2500" value="1500">
</div>
<div class="prow">
<div class="row"><span class="lbl">CH2 &mdash; J8 (IO13)</span><span class="val" id="pv1">1500 µs</span></div>
<input type="range" id="ps1" min="500" max="2500" value="1500">
</div>
<div class="prow">
<div class="row"><span class="lbl">CH3 &mdash; J9 (IO12)</span><span class="val" id="pv2">1500 µs</span></div>
<input type="range" id="ps2" min="500" max="2500" value="1500">
</div>
<div class="prow">
<div class="row"><span class="lbl">CH4 &mdash; J10 (IO11)</span><span class="val" id="pv3">1500 µs</span></div>
<input type="range" id="ps3" min="500" max="2500" value="1500">
</div>
</div>

<div class="card">
<h2>&#128268; Analog Inputs (0&ndash;9 V)</h2>
<div class="row"><span class="lbl">CH1 &mdash; J12 (IO4)</span><span class="val" id="av0">—</span></div>
<div class="bwrap"><div class="bar" id="ab0" style="width:0%"></div></div>
<div class="row"><span class="lbl">CH2 &mdash; J13 (IO5)</span><span class="val" id="av1">—</span></div>
<div class="bwrap"><div class="bar" id="ab1" style="width:0%"></div></div>
<div class="row"><span class="lbl">CH3 &mdash; J14 (IO6)</span><span class="val" id="av2">—</span></div>
<div class="bwrap"><div class="bar" id="ab2" style="width:0%"></div></div>
<div class="row"><span class="lbl">CH4 &mdash; J15 (IO7)</span><span class="val" id="av3">—</span></div>
<div class="bwrap"><div class="bar" id="ab3" style="width:0%"></div></div>
<div style="font-size:10px;color:#555;margin-top:6px">3.9k/2.2k divider &mdash; 0&ndash;9.1 V full scale</div>
</div>

<div class="card wide">
<h2>&#128225; CAN Bus &mdash; TWAI 1 Mbps (IO16/17)</h2>
<div style="display:grid;grid-template-columns:1fr 1fr;gap:12px">
<div>
<div style="font-size:10px;color:#666;margin-bottom:4px">Received frames (newest first)</div>
<div class="clog" id="clog"></div>
</div>
<div>
<div style="font-size:10px;color:#666;margin-bottom:4px">Send frame</div>
<input type="text" id="cid" placeholder="ID hex (e.g. 7FF or 1FFFFFFF)">
<input type="text" id="cdat" placeholder="Data bytes hex (e.g. 01 02 03 04)">
<div class="row" style="margin-top:5px">
<label style="font-size:11px;cursor:pointer"><input type="checkbox" id="cext"> Extended (29-bit)</label>
<button class="btn" onclick="doSendCan()">Send</button>
</div>
</div>
</div>
</div>

<div class="card wide">
<h2>&#128421; Debug Log</h2>
<div class="clog" id="dlog" style="height:180px"></div>
</div>

	</div>
	<script>
	const N=200,CH={ax:'#f44',ay:'#fa4',az:'#4f4',gx:'#48f',gy:'#d4f',gz:'#4ff'};
	const buf={ax:[],ay:[],az:[],gx:[],gy:[],gz:[]};
	const cv=document.getElementById('cv'),cx=cv.getContext('2d');
	const HCH={hgx:'#f55',hgy:'#5f5',hgz:'#59f'};
	const hbuf={hgx:[],hgy:[],hgz:[]};
	const hcv=document.getElementById('hcv'),hcx=hcv.getContext('2d');
	const HGS=16;
	let fc=0,ft=Date.now();
	
	function push(im){for(const k in buf){buf[k].push(im[k]);if(buf[k].length>N)buf[k].shift();}}
	function pushHg(im){for(const k in hbuf){hbuf[k].push(im[k]);if(hbuf[k].length>N)hbuf[k].shift();}}
	function draw(){
	  const W=cv.width=cv.offsetWidth,H=120;cv.height=H;
	  cx.fillStyle='#0d0d0d';cx.fillRect(0,0,W,H);
  cx.strokeStyle='#1a1a1a';cx.lineWidth=1;
  [1,2,3].forEach(i=>{cx.beginPath();cx.moveTo(0,H*i/4);cx.lineTo(W,H*i/4);cx.stroke();});
  cx.strokeStyle='#2a2a2a';cx.beginPath();cx.moveTo(0,H/2);cx.lineTo(W,H/2);cx.stroke();
  for(const[k,arr] of Object.entries(buf)){
    if(!arr.length)continue;
    const sc=k[0]==='a'?2:500;
    cx.strokeStyle=CH[k];cx.lineWidth=1.5;cx.beginPath();
    arr.forEach((v,i)=>{const x=i/(N-1)*W,y=H/2-(v/sc)*(H/2)*.9;i?cx.lineTo(x,y):cx.moveTo(x,y);});
	    cx.stroke();
	  }
	}
	function drawHg(){
	  const W=hcv.width=hcv.offsetWidth,H=120;hcv.height=H;
	  hcx.fillStyle='#0d0d0d';hcx.fillRect(0,0,W,H);
	  hcx.strokeStyle='#1a1a1a';hcx.lineWidth=1;
	  [1,2,3].forEach(i=>{hcx.beginPath();hcx.moveTo(0,H*i/4);hcx.lineTo(W,H*i/4);hcx.stroke();});
	  hcx.strokeStyle='#2a2a2a';hcx.beginPath();hcx.moveTo(0,H/2);hcx.lineTo(W,H/2);hcx.stroke();
	  for(const[k,arr] of Object.entries(hbuf)){
	    if(!arr.length)continue;
	    hcx.strokeStyle=HCH[k];hcx.lineWidth=1.5;hcx.beginPath();
	    arr.forEach((v,i)=>{const x=i/(N-1)*W,y=H/2-(Math.max(-HGS,Math.min(HGS,v))/HGS)*(H/2)*.9;i?hcx.lineTo(x,y):hcx.moveTo(x,y);});
	    hcx.stroke();
	  }
	}
	
	const es=new EventSource('/events');
	es.onmessage=ev=>{
  const d=JSON.parse(ev.data);
  fc++;if(Date.now()-ft>=1000){document.getElementById('srate').textContent=fc+' Hz';fc=0;ft=Date.now();}
  const im=d.imu;
  document.getElementById('iax').textContent=im.ax.toFixed(4)+' g';
  document.getElementById('iay').textContent=im.ay.toFixed(4)+' g';
  document.getElementById('iaz').textContent=im.az.toFixed(4)+' g';
	  document.getElementById('igx').textContent=im.gx.toFixed(2)+' °/s';
	  document.getElementById('igy').textContent=im.gy.toFixed(2)+' °/s';
	  document.getElementById('igz').textContent=im.gz.toFixed(2)+' °/s';
	  document.getElementById('ihgx').textContent=im.hgx.toFixed(3)+' g';
	  document.getElementById('ihgy').textContent=im.hgy.toFixed(3)+' g';
	  document.getElementById('ihgz').textContent=im.hgz.toFixed(3)+' g';
	  document.getElementById('irhgx').textContent=im.rawHgx;
	  document.getElementById('irhgy').textContent=im.rawHgy;
	  document.getElementById('irhgz').textContent=im.rawHgz;
	  push(im);draw();
	  pushHg(im);drawHg();
	  d.adc.forEach((v,i)=>{
	    document.getElementById('av'+i).textContent=v.toFixed(3)+' V';
    document.getElementById('ab'+i).style.width=Math.min(100,v/9.1*100).toFixed(1)+'%';
  });
  d.pwm.forEach((us,i)=>{
    const s=document.getElementById('ps'+i);
    if(document.activeElement!==s){s.value=us;document.getElementById('pv'+i).textContent=us+' µs';}
  });
  const sy=d.sys;
  const ok=sy.imu_ok;
  document.getElementById('idot').className='dot '+(ok?'gr':'re');
  document.getElementById('itxt').textContent=ok?'OK':'ERROR';
  document.getElementById('iwho').textContent='0x'+sy.imu_who.toString(16).toUpperCase().padStart(2,'0');
  document.getElementById('imodel').textContent=sy.imu_model;
  document.getElementById('srssi').textContent=sy.rssi+' dBm';
  document.getElementById('sheap').textContent=(sy.heap/1024).toFixed(0)+' kB';
  const u=sy.uptime;
  document.getElementById('sup').textContent=Math.floor(u/3600)+'h '+Math.floor(u/60%60)+'m '+(u%60)+'s';
  document.getElementById('scantx').textContent=sy.can_tx_ok+' / '+sy.can_tx_err;
  document.getElementById('scanrx').textContent=sy.can_rx_ok+' / '+sy.can_rx_err;
  if(d.can&&d.can.length){
    const log=document.getElementById('clog');
    d.can.forEach(f=>{
      const hex=Array.from({length:f.len},(_,i)=>f.data[i].toString(16).padStart(2,'0').toUpperCase()).join(' ');
      const el=document.createElement('div');el.className='cline';
      el.textContent=(f.ext?'EXT ':'STD ')+f.id.toString(16).toUpperCase().padStart(f.ext?8:3,'0')+' ['+f.len+'] '+hex;
      log.insertBefore(el,log.firstChild);
      if(log.children.length>25)log.removeChild(log.lastChild);
    });
  }
};
es.onerror=()=>{document.getElementById('idot').className='dot ye';};

for(let i=0;i<4;i++){
  document.getElementById('ps'+i).addEventListener('input',function(){
    const us=+this.value;
    document.getElementById('pv'+i).textContent=us+' µs';
    fetch('/api/pwm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ch:i,us})});
  });
}
let lastLogSeq=0;
function pollLog(){
  fetch('/api/log?since='+lastLogSeq).then(r=>r.json()).then(arr=>{
    if(!arr.length)return;
    const log=document.getElementById('dlog');
    const atBottom=log.scrollTop+log.clientHeight>=log.scrollHeight-4;
    arr.forEach(e=>{
      lastLogSeq=Math.max(lastLogSeq,e.seq);
      const el=document.createElement('div');el.className='cline';el.textContent=e.msg;
      log.appendChild(el);
      while(log.children.length>200)log.removeChild(log.firstChild);
    });
    if(atBottom)log.scrollTop=log.scrollHeight;
  }).catch(()=>{});
}
setInterval(pollLog,1000);
pollLog();
function doSendCan(){
  const id=parseInt(document.getElementById('cid').value.trim(),16);
  const raw=document.getElementById('cdat').value.trim();
  const data=raw?raw.split(/\s+/).map(x=>parseInt(x,16)).filter(n=>!isNaN(n)&&n>=0&&n<=255):[];
  const ext=document.getElementById('cext').checked;
  if(isNaN(id)){alert('Invalid CAN ID');return;}
  fetch('/api/can',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id,ext,data})});
}
</script>
</body>
</html>
)rawhtml";

// ---------------------------------------------------------------------------
// Server + SSE
// ---------------------------------------------------------------------------

static AsyncWebServer server(80);
static AsyncEventSource events("/events");

// Shared state written by webUiPushSse, read by /api/status
static WebUiState lastState = {};
static uint8_t    imuWho    = 0x00;  // set by main

void webUiSetImuWhoAmI(uint8_t b) { imuWho = b; }

void webUiInit() {
    // Root — serve dashboard
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", HTML_PAGE);
    });

    // SSE stream
    events.onConnect([](AsyncEventSourceClient* client) {
        if (client->lastId()) {
            Serial.printf("[WEB] SSE reconnect, last id=%u\n", client->lastId());
        }
        client->send("connected", nullptr, millis(), 0);
    });
    server.addHandler(&events);

    // One-shot JSON status
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        JsonDocument doc;
        auto imu = doc["imu"].to<JsonObject>();
        imu["ax"] = lastState.imu.ax;
        imu["ay"] = lastState.imu.ay;
	        imu["az"] = lastState.imu.az;
	        imu["gx"] = lastState.imu.gx;
	        imu["gy"] = lastState.imu.gy;
	        imu["gz"] = lastState.imu.gz;
	        imu["hgx"] = lastState.imu.hgx;
	        imu["hgy"] = lastState.imu.hgy;
	        imu["hgz"] = lastState.imu.hgz;
	        imu["rawHgx"] = lastState.imu.rawHgx;
	        imu["rawHgy"] = lastState.imu.rawHgy;
	        imu["rawHgz"] = lastState.imu.rawHgz;
	        auto adc = doc["adc"].to<JsonArray>();
        for (float v : lastState.adc) adc.add(v);
        auto pwm = doc["pwm"].to<JsonArray>();
        for (uint16_t u : lastState.pwm) pwm.add(u);
        auto sys = doc["sys"].to<JsonObject>();
        sys["imu_ok"]    = lastState.imuOk;
        sys["imu_who"]   = imuWho;
        sys["imu_model"] = imuModelName();
        sys["rssi"]      = lastState.rssi;
        sys["heap"]      = lastState.freeHeap;
        sys["uptime"]    = millis() / 1000;
        sys["can_tx_ok"] = lastState.canTxOk;
        sys["can_tx_err"]= lastState.canTxErr;
        sys["can_rx_ok"] = lastState.canRxOk;
        sys["can_rx_err"]= lastState.canRxErr;
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // PWM command: POST {"ch":0,"us":1500}
    server.on("/api/pwm", HTTP_POST, [](AsyncWebServerRequest* req) {
        req->send(400, "text/plain", "send JSON body");
    }, nullptr, [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
        JsonDocument doc;
        if (deserializeJson(doc, data, len) == DeserializationError::Ok) {
            const uint8_t  ch = doc["ch"] | 255;
            const uint16_t us = doc["us"] | 1500;
            if (ch < 4) {
                pwmSetUs(ch, us);
                req->send(200, "text/plain", "ok");
                return;
            }
        }
        req->send(400, "text/plain", "bad json");
    });

    // CAN send: POST {"id":0x7FF,"ext":false,"data":[1,2,3]}
    server.on("/api/can", HTTP_POST, [](AsyncWebServerRequest* req) {
        req->send(400, "text/plain", "send JSON body");
    }, nullptr, [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
        JsonDocument doc;
        if (deserializeJson(doc, data, len) == DeserializationError::Ok) {
            const uint32_t id  = doc["id"] | 0u;
            const bool     ext = doc["ext"] | false;
            JsonArray      arr = doc["data"].as<JsonArray>();
            uint8_t bytes[8];
            uint8_t byteLen = 0;
            for (JsonVariant v : arr) {
                if (byteLen >= 8) break;
                bytes[byteLen++] = v.as<uint8_t>();
            }
            canSend(id, bytes, byteLen, ext);
            req->send(200, "text/plain", "ok");
            return;
        }
        req->send(400, "text/plain", "bad json");
    });

    // Debug log — ?since=<seq> returns only entries newer than that seq
    server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* req) {
        const uint32_t since = req->hasParam("since") ? req->getParam("since")->value().toInt() : 0;
        JsonDocument doc;
        auto arr = doc.to<JsonArray>();
        // Walk from oldest to newest
        const uint8_t start = (logCount < LOG_MAX) ? 0 : logHead;
        for (uint8_t i = 0; i < logCount; i++) {
            const uint8_t idx = (start + i) % LOG_MAX;
            if (logSeqBuf[idx] <= since) continue;
            auto e = arr.add<JsonObject>();
            e["seq"] = logSeqBuf[idx];
            e["msg"] = logBuf[idx];
        }
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // 404 fallback
    server.onNotFound([](AsyncWebServerRequest* req) {
        req->send(404, "text/plain", "not found");
    });

    server.begin();
    Serial.println("[WEB] Server started on port 80");
    Serial0.println("[WEB] Server started on port 80");
}

void webUiPushSse(const WebUiState& state, const CanFrame* frames, uint8_t frameCount) {
    lastState = state;

    JsonDocument doc;
    doc["t"] = millis();

    auto imu = doc["imu"].to<JsonObject>();
    imu["ax"] = serialized(String(state.imu.ax, 4));
    imu["ay"] = serialized(String(state.imu.ay, 4));
    imu["az"] = serialized(String(state.imu.az, 4));
	    imu["gx"] = serialized(String(state.imu.gx, 2));
	    imu["gy"] = serialized(String(state.imu.gy, 2));
	    imu["gz"] = serialized(String(state.imu.gz, 2));
	    imu["hgx"] = serialized(String(state.imu.hgx, 3));
	    imu["hgy"] = serialized(String(state.imu.hgy, 3));
	    imu["hgz"] = serialized(String(state.imu.hgz, 3));
	    imu["rawHgx"] = state.imu.rawHgx;
	    imu["rawHgy"] = state.imu.rawHgy;
	    imu["rawHgz"] = state.imu.rawHgz;

    auto adc = doc["adc"].to<JsonArray>();
    for (float v : state.adc) adc.add(serialized(String(v, 3)));

    auto pwm = doc["pwm"].to<JsonArray>();
    for (uint16_t u : state.pwm) pwm.add(u);

    auto sys = doc["sys"].to<JsonObject>();
    sys["imu_ok"]    = state.imuOk;
    sys["imu_who"]   = imuWho;
    sys["imu_model"] = imuModelName();
    sys["rssi"]      = state.rssi;
    sys["heap"]      = state.freeHeap;
    sys["uptime"]    = millis() / 1000;
    sys["can_tx_ok"] = state.canTxOk;
    sys["can_tx_err"]= state.canTxErr;
    sys["can_rx_ok"] = state.canRxOk;
    sys["can_rx_err"]= state.canRxErr;

    auto can = doc["can"].to<JsonArray>();
    for (uint8_t i = 0; i < frameCount; i++) {
        auto f = can.add<JsonObject>();
        f["id"]  = frames[i].id;
        f["len"] = frames[i].len;
        f["ext"] = frames[i].extended;
        auto d = f["data"].to<JsonArray>();
        for (uint8_t j = 0; j < frames[i].len; j++) d.add(frames[i].data[j]);
    }

    String out;
    serializeJson(doc, out);
    events.send(out.c_str(), "message", millis());
}
