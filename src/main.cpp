#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

// WiFi设置
const char* ssid = "roefruit";
const char* password = "1234567890";

// MQTT设置
const char* mqtt_server = "198.89.125.170";
const int mqtt_port = 1883;
String client_id = "voice_client_" + String((uint32_t)ESP.getEfuseMac() & 0xFFFFFFFF, HEX);
String stream_topic = "voice/stream/" + client_id;
String response_topic = "voice/response/" + client_id;

// 按钮设置
#define BUTTON_PIN 19        // 按钮引脚
#define DEBOUNCE_TIME 50     // 消抖时间(ms)
bool is_recording = false;   // 录音状态
bool last_button_state = HIGH;   // 上一次按钮状态
unsigned long last_debounce_time = 0;  // 上次消抖时间

// 调试标志
#define DEBUG_AUDIO true
#define DEBUG_MQTT true   
unsigned long last_debug_time = 0;
const unsigned long DEBUG_INTERVAL = 5000;

// 发送结束标记
const uint8_t END_MARKER[] = "END_OF_STREAM";

// I2S引脚定义
#define I2S_BCLK_OUT      26
#define I2S_LRC_OUT       25
#define I2S_DOUT          22
#define I2S_BCLK_IN       14
#define I2S_LRC_IN        15
#define I2S_DIN           32

// I2S端口
#define I2S_PORT_TX       I2S_NUM_0
#define I2S_PORT_RX       I2S_NUM_1

// MQTT发送控制
const int MQTT_SEND_INTERVAL = 10; // 发送间隔(毫秒)
unsigned long last_send_time = 0;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

// 音频缓冲区
int16_t audio_buffer[BUFFER_SIZE];

// 统计信息
unsigned long total_bytes_sent = 0;
unsigned long total_bytes_received = 0;
unsigned long packets_sent = 0;
unsigned long packets_received = 0;
unsigned long failed_sends = 0;
unsigned long record_start_time = 0;  // 记录开始录音的时间

void print_debug_info() {
    if (DEBUG_MQTT && (millis() - last_debug_time >= DEBUG_INTERVAL)) {
        Serial.println("\n=== 系统状态 ===");
        Serial.println("WiFi RSSI: " + String(WiFi.RSSI()) + " dBm");
        Serial.println("MQTT 连接状态: " + String(mqtt_client.connected() ? "已连接" : "未连接"));
        Serial.println("录音状态: " + String(is_recording ? "录音中" : "停止"));
        if (is_recording) {
            Serial.println("录音时长: " + String((millis() - record_start_time) / 1000) + " 秒");
        }
        Serial.println("发送统计:");
        Serial.println("- 总字节数: " + String(total_bytes_sent));
        Serial.println("- 总包数: " + String(packets_sent));
        Serial.println("- 失败次数: " + String(failed_sends));
        Serial.println("- 平均包大小: " + String(packets_sent > 0 ? total_bytes_sent/packets_sent : 0) + " bytes");
        Serial.println("接收统计:");
        Serial.println("- 总字节数: " + String(total_bytes_received));
        Serial.println("- 总包数: " + String(packets_received));
        Serial.println("- 平均包大小: " + String(packets_received > 0 ? total_bytes_received/packets_received : 0) + " bytes");
        Serial.println("MQTT主题:");
        Serial.println("- 发送: " + stream_topic);
        Serial.println("- 接收: " + response_topic);
        Serial.println("===============\n");
        last_debug_time = millis();
    }
}

void setup_wifi() {
    delay(10);
    Serial.println("\n准备连接到WiFi网络: " + String(ssid));
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi已连接");
    Serial.println("IP地址: " + WiFi.localIP().toString());
    Serial.println("信号强度: " + String(WiFi.RSSI()) + " dBm");
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("收到MQTT消息:");
    Serial.println("- 主题: " + String(topic));
    Serial.println("- 数据长度: " + String(length) + " bytes");
    
    if (String(topic) == response_topic) {
        size_t bytes_written = 0;
        i2s_write(I2S_PORT_TX, payload, length, &bytes_written, portMAX_DELAY);
        
        total_bytes_received += length;
        packets_received++;
        
        if (DEBUG_AUDIO) {
            Serial.println("音频输出: " + String(bytes_written) + " bytes written");
        }
    }
}

const char* mqtt_state_str(int state) {
    switch (state) {
        case -4: return "MQTT_CONNECTION_TIMEOUT";
        case -3: return "MQTT_CONNECTION_LOST";
        case -2: return "MQTT_CONNECT_FAILED";
        case -1: return "MQTT_DISCONNECTED";
        case 0: return "MQTT_CONNECTED";
        case 1: return "MQTT_CONNECT_BAD_PROTOCOL";
        case 2: return "MQTT_CONNECT_BAD_CLIENT_ID";
        case 3: return "MQTT_CONNECT_UNAVAILABLE";
        case 4: return "MQTT_CONNECT_BAD_CREDENTIALS";
        case 5: return "MQTT_CONNECT_UNAUTHORIZED";
        default: return "MQTT_UNKNOWN";
    }
}

void reconnect() {
    while (!mqtt_client.connected()) {
        Serial.print("尝试MQTT连接...");
        if (mqtt_client.connect(client_id.c_str())) {
            Serial.println("已连接");
            Serial.println("订阅主题: " + response_topic);
            mqtt_client.subscribe(response_topic.c_str());
        } else {
            Serial.print("连接失败, 状态码: ");
            Serial.print(mqtt_client.state());
            Serial.print(" (");
            Serial.print(mqtt_state_str(mqtt_client.state()));
            Serial.println(")");
            Serial.println("5秒后重试");
            delay(5000);
        }
    }
}

void i2s_init() {
    Serial.println("\n初始化I2S...");
    Serial.println("音频参数:");
    Serial.println("- 采样率: " + String(SAMPLE_RATE) + " Hz");
    Serial.println("- 位深度: " + String(SAMPLE_BITS) + " bits");
    Serial.println("- 声道数: " + String(CHANNELS));
    Serial.println("- 缓冲区大小: " + String(BUFFER_SIZE) + " samples");
    Serial.println("- 实际数据大小: " + String(BUFFER_SIZE * 2) + " bytes");
    
    // 配置I2S输出
    i2s_config_t i2s_config_tx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    i2s_pin_config_t pin_config_tx = {
        .bck_io_num = I2S_BCLK_OUT,
        .ws_io_num = I2S_LRC_OUT,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    esp_err_t err = i2s_driver_install(I2S_PORT_TX, &i2s_config_tx, 0, NULL);
    if (err != ESP_OK) {
        Serial.println("I2S TX驱动安装失败: " + String(esp_err_to_name(err)));
    }
    err = i2s_set_pin(I2S_PORT_TX, &pin_config_tx);
    if (err != ESP_OK) {
        Serial.println("I2S TX引脚配置失败: " + String(esp_err_to_name(err)));
    }

    // 配置I2S输入
    i2s_config_t i2s_config_rx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    i2s_pin_config_t pin_config_rx = {
        .bck_io_num = I2S_BCLK_IN,
        .ws_io_num = I2S_LRC_IN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN
    };

    err = i2s_driver_install(I2S_PORT_RX, &i2s_config_rx, 0, NULL);
    if (err != ESP_OK) {
        Serial.println("I2S RX驱动安装失败: " + String(esp_err_to_name(err)));
    }
    err = i2s_set_pin(I2S_PORT_RX, &pin_config_rx);
    if (err != ESP_OK) {
        Serial.println("I2S RX引脚配置失败: " + String(esp_err_to_name(err)));
    }
    
    Serial.println("I2S初始化完成");
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32音频MQTT客户端启动 ===");
    Serial.println("版本: 1.0.0");
    Serial.println("编译时间: " __DATE__ " " __TIME__);
    
    // 初始化按钮引脚
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.println("按钮已初始化 (GPIO" + String(BUTTON_PIN) + ")");
    
    // 初始化WiFi
    setup_wifi();
    
    // 初始化MQTT客户端
    Serial.println("\n配置MQTT客户端:");
    Serial.println("- 服务器: " + String(mqtt_server));
    Serial.println("- 端口: " + String(mqtt_port));
    Serial.println("- 客户端ID: " + client_id);
    Serial.println("- MQTT最大包大小: " + String(MQTT_MAX_PACKET_SIZE) + " bytes");
    Serial.println("- 音频缓冲区大小: " + String(BUFFER_SIZE * 2) + " bytes");
    
    mqtt_client.setBufferSize(MQTT_MAX_PACKET_SIZE);
    mqtt_client.setServer(mqtt_server, mqtt_port);
    mqtt_client.setCallback(mqtt_callback);
    
    // 初始化I2S
    i2s_init();
    
    Serial.println("\n系统初始化完成");
    Serial.println("MQTT主题配置:");
    Serial.println("- 发送主题: " + stream_topic);
    Serial.println("- 接收主题: " + response_topic);
    Serial.println("\n按下按钮开始/停止录音");
}

void handle_button() {
    bool current_button_state = digitalRead(BUTTON_PIN);
    
    if ((millis() - last_debounce_time) > DEBOUNCE_TIME) {
        if (current_button_state != last_button_state) {
            last_debounce_time = millis();
            
            if (current_button_state == LOW) {
                is_recording = !is_recording;
                
                if (is_recording) {
                    Serial.println("\n开始录音...");
                    record_start_time = millis();
                    total_bytes_sent = 0;
                    packets_sent = 0;
                    failed_sends = 0;
                } else {
                    // 发送结束标记
                    const char* end_marker = "END_OF_STREAM";  // 使用字符串而不是��节数组
                    mqtt_client.publish(stream_topic.c_str(), (const uint8_t*)end_marker, strlen(end_marker));
                    
                    Serial.println("\n停止录音");
                    Serial.println("录音时长: " + String((millis() - record_start_time) / 1000) + " 秒");
                    Serial.println("发送统计:");
                    Serial.println("- 总字节数: " + String(total_bytes_sent));
                    Serial.println("- 总包数: " + String(packets_sent));
                    Serial.println("- 失败次数: " + String(failed_sends));
                }
            }
        }
    }
    
    last_button_state = current_button_state;
}

void handle_audio() {
    if (!is_recording || (millis() - last_send_time < MQTT_SEND_INTERVAL)) {
        return;
    }

    size_t bytes_read = 0;
    esp_err_t result = i2s_read(I2S_PORT_RX, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
    
    if (result != ESP_OK) {
        if (DEBUG_AUDIO) {
            Serial.println("I2S读取错误: " + String(esp_err_to_name(result)));
        }
        return;
    }

    if (bytes_read > 0) {
        // 检查音频数据是否有效（不是静音）
        bool has_audio = false;
        int16_t* samples = (int16_t*)audio_buffer;
        int sample_count = bytes_read / 2;
        int32_t sum = 0;
        
        // 计算音频能量
        for (int i = 0; i < sample_count; i++) {
            sum += abs(samples[i]);
        }
        float average = sum / (float)sample_count;
        
        // 使用平均能量判断是否有效音频
        if (average > 100) {
            has_audio = true;
        }

        if (has_audio || true) {  // 暂时总是发送，用于测试
            uint8_t* audio_data = (uint8_t*)audio_buffer;
            int remaining = bytes_read;
            int offset = 0;
            
            // 分块发送数据
            while (remaining > 0 && is_recording) {
                // 确保发送的数据不超过MQTT限制
                int chunk_size = min(remaining, MQTT_MAX_PACKET_SIZE - 50);  // 留出一些余量
                
                bool success = mqtt_client.publish(stream_topic.c_str(), &audio_data[offset], chunk_size);
                
                if (success) {
                    total_bytes_sent += chunk_size;
                    packets_sent++;
                    if (DEBUG_AUDIO) {
                        Serial.println("音频发送成功: " + String(chunk_size) + " bytes, 偏移: " + String(offset));
                    }
                    remaining -= chunk_size;
                    offset += chunk_size;
                } else {
                    failed_sends++;
                    if (DEBUG_MQTT) {
                        Serial.println("MQTT发送失败 - 数据大小: " + String(chunk_size));
                    }
                    if (!mqtt_client.connected()) {
                        reconnect();
                    }
                    delay(5);  // 短暂延迟后继续
                }
                
                // 在块之间添加短暂延迟
                delay(2);
            }
        }
    }
    
    last_send_time = millis();
}

void loop() {
    if (!mqtt_client.connected()) {
        reconnect();
    }
    mqtt_client.loop();

    // 处理按钮
    handle_button();
    
    // 处理音频数据
    handle_audio();
    
    // 打印调试信息
    print_debug_info();
}