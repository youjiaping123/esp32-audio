#include <Arduino.h>
#include <WiFi.h>
#include <Audio.h>

// MAX98357A I2S 引脚定义
#define I2S_BCLK      26  // 位时钟
#define I2S_LRC       25  // 字时钟
#define I2S_DOUT      22  // 数据输出

Audio audio;

void setup() {
    Serial.begin(115200);
    
    // 配置 I2S 引脚
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    
    // WiFi 连接设置
    WiFi.begin("roefruit", "1234567890");
    
    // 等待 WiFi 连接
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nWiFi 已连接");
    Serial.println("IP 地址: " + WiFi.localIP().toString());
    
    // 设置音量 (0-21)
    audio.setVolume(21);
    
    // 连接到音频流
    audio.connecttohost("http://doc.itprojects.cn/0006.zhishi.esp32/01.download/audio/maifu.wav");  // 替换为你的音频URL
}

void loop() {
    audio.loop();
}

// 音频信息回调函数
void audio_info(const char *info) {
    Serial.print("info: ");
    Serial.println(info);
}

void audio_showstation(const char *info) {
    Serial.print("station: ");
    Serial.println(info);
}

void audio_showstreamtitle(const char *info) {
    Serial.print("streamtitle: ");
    Serial.println(info);
}