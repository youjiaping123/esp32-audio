#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>

// WiFi设置
const char* ssid = "roefruit";
const char* password = "1234567890";

// MQTT设置
const char* mqtt_server = "198.89.125.170";
const int mqtt_port = 1883;
String client_id = "voice_client_" + String((uint32_t)ESP.getEfuseMac() & 0xFFFFFFFF, HEX);
String stream_topic = "voice/stream/" + client_id;
String response_topic = "voice/response/" + client_id;

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

// MQTT发送控制
const int MQTT_SEND_INTERVAL = 10; // 发送间隔(毫秒)
unsigned long last_send_time = 0;

// MQTT客户端
AsyncMqttClient mqtt_client;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// 音频缓冲区
int16_t audio_buffer[BUFFER_SIZE];

// 统计信息
unsigned long total_bytes_sent = 0;
unsigned long total_bytes_received = 0;
unsigned long packets_sent = 0;
unsigned long packets_received = 0;
unsigned long failed_sends = 0;
unsigned long record_start_time = 0;  // 记录开始录音的时间

// 函数声明
void connectToWifi();
void connectToMqtt();
void onWifiConnect(WiFiEvent_t event, WiFiEventInfo_t info);
void onWifiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void i2s_init();
void handle_button();
void handle_audio();
void print_debug_info();

// 函数实现
void connectToMqtt() {
    Serial.println("正在连接到MQTT服务器...");
    Serial.println("- 服务器: " + String(mqtt_server));
    Serial.println("- 端口: " + String(mqtt_port));
    Serial.println("- 客户端ID: " + client_id);
    mqtt_client.connect();
}

void connectToWifi() {
    Serial.println("\n准备连接到WiFi网络: " + String(ssid));
    WiFi.begin(ssid, password);
}

void onWifiConnect(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("\nWiFi已连接");
    Serial.println("IP地址: " + WiFi.localIP().toString());
    Serial.println("信号强度: " + String(WiFi.RSSI()) + " dBm");
    Serial.println("MAC地址: " + WiFi.macAddress());
    Serial.println("DNS服务器: " + WiFi.dnsIP().toString());
    
    // 等待一下再连接MQTT
    delay(500);
    connectToMqtt();
}

void onWifiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("WiFi连接断开");
    xTimerStop(mqttReconnectTimer, 0);
    xTimerStart(wifiReconnectTimer, 0);
}

void onMqttConnect(bool sessionPresent) {
    Serial.println("已连接到MQTT服务器");
    Serial.println("会话状态: " + String(sessionPresent ? "已恢复" : "新会话"));
    Serial.println("订阅主题: " + response_topic);
    
    // 订阅主题并检查结果
    uint16_t packetId = mqtt_client.subscribe(response_topic.c_str(), 0);
    if (packetId > 0) {
        Serial.println("订阅请求已发送，PacketId: " + String(packetId));
    } else {
        Serial.println("订阅请求发送失败");
    }
    
    // 发送测试消息
    String test_topic = "test/" + client_id;
    packetId = mqtt_client.publish(test_topic.c_str(), 0, true, "设备在线");
    if (packetId > 0) {
        Serial.println("测试消息已发送，PacketId: " + String(packetId));
    } else {
        Serial.println("测试消息发送失败");
    }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("MQTT连接断开");
    Serial.print("断开原因: ");
    switch (reason) {
        case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
            Serial.println("TCP连接断开");
            break;
        case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
            Serial.println("MQTT协议版本不兼容");
            break;
        case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
            Serial.println("客户端ID被拒绝");
            break;
        case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
            Serial.println("服务器不可用");
            break;
        case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
            Serial.println("认证信息格式错误");
            break;
        case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
            Serial.println("未授权");
            break;
        default:
            Serial.println("未知原因");
            break;
    }
    if (WiFi.isConnected()) {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    if (String(topic) == response_topic) {
        size_t bytes_written = 0;
        i2s_write(I2S_PORT_TX, payload, len, &bytes_written, portMAX_DELAY);
        
        total_bytes_received += len;
        packets_received++;
        
        if (DEBUG_AUDIO) {
            Serial.println("音频输出: " + String(bytes_written) + " bytes written");
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
                    const char* end_msg = "END_OF_STREAM";
                    mqtt_client.publish(stream_topic.c_str(), 0, false, end_msg);
                    
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

    if (!mqtt_client.connected()) {
        if (DEBUG_MQTT) {
            Serial.println("MQTT未连接，等待重连...");
        }
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
            // 分成更小的块发送
            uint8_t* audio_data = (uint8_t*)audio_buffer;
            int chunk_size = 128;  // 发送更小的数据块
            int remaining = bytes_read;
            int offset = 0;
            
            while (remaining > 0 && is_recording && mqtt_client.connected()) {
                int current_chunk = min(remaining, chunk_size);
                
                uint16_t packet_id = mqtt_client.publish(
                    stream_topic.c_str(),  // 主题
                    0,                     // QoS
                    false,                 // retain
                    (const char*)&audio_data[offset],  // 数据
                    current_chunk          // 数据长度
                );
                
                if (packet_id > 0) {
                    total_bytes_sent += current_chunk;
                    packets_sent++;
                    if (DEBUG_AUDIO) {
                        Serial.println("音频发送成功: " + String(current_chunk) + " bytes, PacketId: " + String(packet_id));
                    }
                    remaining -= current_chunk;
                    offset += current_chunk;
                } else {
                    failed_sends++;
                    if (DEBUG_MQTT) {
                        Serial.println("MQTT发送失败 - 数据大小: " + String(current_chunk));
                    }
                    delay(5);  // 短暂延迟后重试
                }
                
                delay(2);  // 在块之间添加短暂延迟
            }
        }
    }
    
    last_send_time = millis();
}

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

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32音频MQTT客户端启动 ===");
    Serial.println("版本: 1.0.0");
    Serial.println("编译时间: " __DATE__ " " __TIME__);
    
    // 初始化按钮引脚
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.println("按钮已初始化 (GPIO" + String(BUTTON_PIN) + ")");
    
    // 创建定时器
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

    // 配置WiFi事件处理
    WiFi.onEvent(onWifiConnect, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WiFi.onEvent(onWifiDisconnect, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    // 配置MQTT客户端
    mqtt_client.onConnect(onMqttConnect);
    mqtt_client.onDisconnect(onMqttDisconnect);
    mqtt_client.onMessage(onMqttMessage);
    mqtt_client.setServer(mqtt_server, mqtt_port);
    mqtt_client.setClientId(client_id.c_str());
    mqtt_client.setKeepAlive(60);
    mqtt_client.setMaxTopicLength(128);
    
    // 添加MQTT连接事件处理
    mqtt_client.onPublish([](uint16_t packetId) {
        if (DEBUG_MQTT) {
            Serial.println("消息发布确认，PacketId: " + String(packetId));
        }
    });
    
    mqtt_client.onSubscribe([](uint16_t packetId, uint8_t qos) {
        Serial.println("订阅确认，PacketId: " + String(packetId) + ", QoS: " + String(qos));
    });
    
    Serial.println("\n配置MQTT客户端:");
    Serial.println("- 服务器: " + String(mqtt_server));
    Serial.println("- 端口: " + String(mqtt_port));
    Serial.println("- 客户端ID: " + client_id);
    Serial.println("- MQTT版本: v5");
    Serial.println("- 最大主题长度: 128 bytes");
    Serial.println("- 音频缓冲区大小: " + String(BUFFER_SIZE * 2) + " bytes");
    
    // 初始化I2S
    i2s_init();
    
    // 连接WiFi
    WiFi.disconnect();  // 确保断开之前的连接
    delay(100);
    WiFi.mode(WIFI_STA);  // 设置为Station模式
    connectToWifi();
    
    Serial.println("\n系统初始化完成");
    Serial.println("MQTT主题配置:");
    Serial.println("- 发送主题: " + stream_topic);
    Serial.println("- 接收主题: " + response_topic);
    Serial.println("\n按下按钮开始/停止录音");
}

void loop() {
    static unsigned long last_connection_check = 0;
    const unsigned long CONNECTION_CHECK_INTERVAL = 5000;  // 每5秒检查一次连接状态
    
    // 定期检查连接状态
    if (millis() - last_connection_check >= CONNECTION_CHECK_INTERVAL) {
        if (!WiFi.isConnected()) {
            Serial.println("WiFi连接已断开，尝试重连...");
            connectToWifi();
        } else if (!mqtt_client.connected()) {
            Serial.println("MQTT连接已断开，尝试重连...");
            connectToMqtt();
        }
        last_connection_check = millis();
    }
    
    // 处理按钮
    handle_button();
    
    // 处理音频数据
    handle_audio();
    
    // 打印调试信息
    print_debug_info();
}