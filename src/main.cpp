/* ============================================================
 *  ESP32-S3 多线程环境与生命体征监测系统  —— 最终版
 * ============================================================
 *  需求：
 *    线程1 (dhtTask)   : 读取 DHT11 温度 / 湿度
 *    线程2 (radarTask) : 读取 C4001 雷达 —— 呼吸频率 + 身体移动
 *                        (速度模式 + 自实现 FFT，估算呼吸次数/分钟)
 *    线程3 (wifiTask)  : 监控 WiFi 连接状态
 *    线程4 (netTask)   : 上报 温度+湿度+呼吸频率+身体移动 至 Mac
 *
 *  接线：
 *    ESP32-S3  GPIO4  -> C4001 mmWave TX   (ESP32 串口 RX)
 *    ESP32-S3  GPIO5  -> C4001 mmWave RX   (ESP32 串口 TX)
 *    ESP32-S3  GPIO6  -> C4001 mmWave OUT  (数字输出，身体移动检测)
 *    ESP32-S3  GPIO7  -> DHT11 signal
 *    C4001    VIN -> 3V3   GND -> GND
 *    DHT11    vcc -> 3V3   ground -> GND
 *
 *  WiFi : SSID=XiiPhone  PASS=yuxi831qwert!
 *  上报 : GET http://172.20.10.6:8088/log?temp=..&hum=..&resp=..&move=..
 *
 *  呼吸频率估算：
 *    实测 C4001 的 getTargetSpeed() 测速下限 0.1m/s，而呼吸胸廓微动
 *    速度约 0.005~0.02 m/s，速度信号测不到呼吸。
 *    故改用观测 getTargetEnergy()（反射能量随胸廓起伏周期变化）。
 *    Fs=10Hz 采 256 点(25.6s)，radix-2 FFT + Hann 窗 + 抛物线插值，
 *    在 11~36 BPM 区间找主峰。
 * ============================================================ */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "DFRobot_C4001.h"
#include <DHT.h>
#include <math.h>

/* ---------- 接线引脚定义 ---------- */
#define PIN_C4001_RX   4     // ESP32 串口RX，接 C4001 TX
#define PIN_C4001_TX   5     // ESP32 串口TX，接 C4001 RX
#define PIN_C4001_OUT  6     // C4001 数字输出（身体移动检测）
#define PIN_DHT11      7     // DHT11 数据线

/* ---------- C4001 毫米波雷达（UART, 9600） ---------- */
DFRobot_C4001_UART radar(&Serial1, 9600, PIN_C4001_RX, PIN_C4001_TX);

/* ---------- DHT11 温湿度 ---------- */
DHT dht(PIN_DHT11, DHT11);

/* ---------- WiFi 配置 ---------- */
const char* WIFI_SSID = "XiiPhone";
const char* WIFI_PASS = "yuxi831qwert!";

/* ---------- Mac 端接收服务器配置 ---------- */
const char* HOST = "172.20.10.6";
const int   PORT = 8088;
const char* PATH = "/log";

/* ---------- 呼吸频率 FFT 参数 ---------- */
#define FFT_N        256        // FFT 点数（radix-2）
#define Fs           10.0f      // 采样率 Hz（每 100ms 采一次）
#define SAMPLE_MS    100        // 采样间隔毫秒
// 呼吸频率搜索区间（Hz）: 11~36 BPM
#define RESP_LO_F    0.18f
#define RESP_HI_F    0.60f

/* ---------- 多线程间共享数据 ---------- */
volatile float  g_tempC    = 0.0f;
volatile float  g_humidity = 0.0f;
volatile float  g_respBPM  = 0.0f;   // 呼吸频率（次/分钟）
volatile bool   g_moving   = false;  // 身体移动
volatile bool   g_radarOk  = false;
volatile bool   g_dhtOk    = false;
char            g_wifiStatus[64];

/* ============================================================
 *  自实现 radix-2 迭代 FFT（原地，复数被 real/imag 交错存储，
 *  in[] 长度 = 2*N，real[i]=in[2i], imag[i]=in[2i+1]）
 * ============================================================ */
void radix2FFT(float* in, uint16_t n) {
  // 位反转
  for (uint16_t i = 1, j = 0; i < n; i++) {
    uint16_t bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float tr = in[2*i];   in[2*i]   = in[2*j];   in[2*j]   = tr;
      float ti = in[2*i+1]; in[2*i+1] = in[2*j+1]; in[2*j+1] = ti;
    }
  }
  // 蝶形运算
  for (uint16_t len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * M_PI / len;
    float wr = cosf(ang), wi = sinf(ang);
    for (uint16_t i = 0; i < n; i += len) {
      float cr = 1.0f, ci = 0.0f;
      for (uint16_t k = 0; k < len/2; k++) {
        uint16_t a = i + k, b = i + k + len/2;
        float tr = cr*in[2*b] - ci*in[2*b+1];
        float ti = cr*in[2*b+1] + ci*in[2*b];
        in[2*b]   = in[2*a]   - tr;
        in[2*b+1] = in[2*a+1] - ti;
        in[2*a]  += tr;
        in[2*a+1]+= ti;
        float nwr = cr*wr - ci*wi;
        ci = cr*wi + ci*wr;
        cr = nwr;
      }
    }
  }
}

/* ============================================================
 *  呼吸频率估算：对能量信号做 FFT，提取呼吸主频(Hz)
 * ============================================================ */
float estimateRespirationHz(float* samples) {
  static float buf[2*FFT_N];   // 静态缓冲，避免栈溢出

  // 1) 去均值 + Hann 窗 + 填复数
  float mean = 0.0f;
  for (int i = 0; i < FFT_N; i++) mean += samples[i];
  mean /= FFT_N;

  for (int i = 0; i < FFT_N; i++) {
    float w = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_N - 1))); // Hann
    buf[2*i]   = (samples[i] - mean) * w;
    buf[2*i+1] = 0.0f;
  }

  radix2FFT(buf, FFT_N);

  // 3) 幅值谱，在搜索区间找主峰
  float binW  = Fs / FFT_N;
  int   loBin = (int)(RESP_LO_F / binW);
  int   hiBin = (int)(RESP_HI_F / binW);
  if (loBin < 1) loBin = 1;
  if (hiBin > FFT_N/2) hiBin = FFT_N/2;

  int   peakBin = loBin;
  float peakMag = -1.0f;
  for (int k = loBin; k <= hiBin; k++) {
    float mag = sqrtf(buf[2*k]*buf[2*k] + buf[2*k+1]*buf[2*k+1]);
    if (mag > peakMag) { peakMag = mag; peakBin = k; }
  }

  // 4) 抛物线插值提高频率精度
  float freq = peakBin * binW;
  if (peakBin > loBin && peakBin < hiBin) {
    auto magAt = [&](int kk) {
      return sqrtf(buf[2*kk]*buf[2*kk] + buf[2*kk+1]*buf[2*kk+1]);
    };
    float ym1 = magAt(peakBin-1), y0 = peakMag, yp1 = magAt(peakBin+1);
    float d = 0.5f * (ym1 - yp1) / (ym1 - 2.0f*y0 + yp1 + 1e-12f);
    if (d > -0.5f && d < 0.5f) freq = (peakBin + d) * binW;
  }
  return freq;
}

/* ============================================================
 *  线程1：DHT11 温湿度采集
 * ============================================================ */
void dhtTask(void* pvParam) {
  for (;;) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      g_tempC = t; g_humidity = h; g_dhtOk = true;
      Serial.printf("[DHT] 温度=%.1f°C 湿度=%.1f%%\n", t, h);
    } else {
      Serial.println("[DHT] 读取失败(检查接线/上拉电阻)");
    }
    delay(2000);
  }
}

/* ============================================================
 *  线程2：C4001 雷达 —— 呼吸频率(能量信号FFT) + 身体移动
 * ============================================================ */
void radarTask(void* pvParam) {
  static float samples[FFT_N];
  static int   idx = 0;
  unsigned long last = 0;

  for (;;) {
    if (!g_radarOk) { delay(300); continue; }

    unsigned long now = millis();
    if (now - last >= SAMPLE_MS) {
      last = now;
      bool hasTarget = (radar.getTargetNumber() > 0);
      float en = radar.getTargetEnergy();
      // 有目标记录能量，无人置 0 抑制环境噪声
      samples[idx] = (hasTarget && !isnan(en)) ? en : 0.0f;
      idx++;

      if (idx >= FFT_N) {
        float hz  = estimateRespirationHz(samples);
        float bpm = hz * 60.0f;
        g_respBPM = (bpm >= 8.0f && bpm <= 40.0f) ? bpm : 0.0f;
        Serial.printf("[Radar] 呼吸频率=%.1f 次/分钟 (峰%.2fHz)\n",
                      (double)g_respBPM, (double)hz);
        // 滑动窗：保留后一半
        for (int i = 0; i < FFT_N/2; i++) samples[i] = samples[i + FFT_N/2];
        idx = FFT_N/2;
      }
    }

    // 身体移动检测（OUT 引脚高电平=有活动）
    g_moving = (digitalRead(PIN_C4001_OUT) == HIGH);
  }
}

/* ============================================================
 *  线程3：WiFi 状态监控
 * ============================================================ */
void wifiTask(void* pvParam) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      snprintf(g_wifiStatus, sizeof(g_wifiStatus),
               "已连接 %s IP=%s", WIFI_SSID, WiFi.localIP().toString().c_str());
    } else {
      snprintf(g_wifiStatus, sizeof(g_wifiStatus),
               "WiFi未连接(status=%d)", (int)WiFi.status());
    }
    Serial.printf("[WiFi] %s\n", g_wifiStatus);
    delay(3000);
  }
}

/* ============================================================
 *  线程4：向 Mac 上报 温度+湿度+呼吸频率+身体移动
 * ============================================================ */
void netSendTask(void* pvParam) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      char url[200];
      snprintf(url, sizeof(url),
               "http://%s:%d%s?temp=%.1f&hum=%.1f&resp=%.1f&move=%d",
               HOST, PORT, PATH,
               (double)g_tempC, (double)g_humidity,
               (double)g_respBPM, g_moving ? 1 : 0);

      http.begin(url);
      http.setTimeout(3000);
      int code = http.GET();
      if (code > 0) {
        Serial.printf("[Net] HTTP=%d 温度=%.1f°C 湿度=%.1f%% 呼吸=%.1f 移动=%s\n",
                      code, (double)g_tempC, (double)g_humidity,
                      (double)g_respBPM, g_moving ? "YES" : "NO");
      } else {
        Serial.printf("[Net] 发送失败 code=%d（确认Mac已运行server.py且端口=%d）\n", code, PORT);
      }
      http.end();
    } else {
      Serial.println("[Net] WiFi 未连接，跳过发送");
    }
    delay(5000);
  }
}

/* ============================================================
 *  setup
 * ============================================================ */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n===== ESP32-S3 环境与生命体征监测系统 =====");

  // --- C4001 初始化（速度模式，用于读取能量/呼吸 FFT）---
  pinMode(PIN_C4001_OUT, INPUT);
  radar.begin();
  delay(300);
  if (radar.setSensorMode(eSpeedMode)) {
    radar.setDetectThres(30, 1200, 10);
    radar.setFrettingDetection(eON);   // 启用微动(呼吸)检测
    g_radarOk = true;
    Serial.println("[Radar] C4001 初始化成功（速度模式+微动检测）");
  } else {
    Serial.println("[Radar] C4001 模式设置失败！请检查接线/供电");
  }

  // --- DHT11 初始化 ---
  dht.begin();
  delay(200);
  float t = dht.readTemperature();
  if (!isnan(t)) {
    g_dhtOk = true;
    Serial.printf("[DHT] 初始化成功 当前温度=%.1f°C\n", t);
  } else {
    Serial.println("[DHT] 初始化失败，请检查接线");
  }

  // --- WiFi 连接 ---
  Serial.printf("[WiFi] 连接 %s ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(400);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(g_wifiStatus, sizeof(g_wifiStatus),
             "已连接 %s IP=%s", WIFI_SSID, WiFi.localIP().toString().c_str());
    Serial.printf("\n[WiFi] 连接成功 IP=%s\n", WiFi.localIP().toString().c_str());
  } else {
    snprintf(g_wifiStatus, sizeof(g_wifiStatus),
             "WiFi连接失败(status=%d)", (int)WiFi.status());
    Serial.printf("\n[WiFi] 连接失败（请检查热点/密码/范围）\n");
  }

  // --- 创建四个 FreeRTOS 任务（多线程） ---
  xTaskCreatePinnedToCore(dhtTask,    "dhtTask",    4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(radarTask,  "radarTask", 16384, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(wifiTask,   "wifiTask",   4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(netSendTask,"netTask",    8192, NULL, 1, NULL, 1);
}

/* ============================================================
 *  loop —— 主循环（周期摘要）
 * ============================================================ */
void loop() {
  delay(10000);
  Serial.println("----------------------------------------");
  Serial.printf("[摘要] 温度=%.1f°C 湿度=%.1f%% 呼吸=%.1fBPM 移动=%s\n",
                g_tempC, g_humidity,
                (double)g_respBPM, g_moving ? "YES" : "NO");
}
