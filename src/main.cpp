#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi设置
const char* ssid = "roefruit";
const char* password = "1234567890";

// MQTT设置
const char* mqtt_server = "198.89.125.170";
const int mqtt_port = 1883;
String client_id = "esp32_audio_" + String((uint32_t)ESP.getEfuseMac(), HEX); // 使用MAC地址生成唯一ID
String stream_topic = "voice/stream/" + client_id;
String response_topic = "voice/response/" + client_id;

// 调试标志
#define DEBUG_AUDIO true  // 改为true以查看更多音频相关信息
#define DEBUG_MQTT true   
unsigned long last_debug_time = 0;
const unsigned long DEBUG_INTERVAL = 5000;

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
bool is_recording = false;

// 统计信息
unsigned long total_bytes_sent = 0;
unsigned long total_bytes_received = 0;
unsigned long packets_sent = 0;
unsigned long packets_received = 0;
unsigned long failed_sends = 0;

void print_debug_info() {
    if (DEBUG_MQTT && (millis() - last_debug_time >= DEBUG_INTERVAL)) {
        Serial.println("\n=== 系统状态 ===");
        Serial.println("WiFi RSSI: " + String(WiFi.RSSI()) + " dBm");
        Serial.println("MQTT 连接状态: " + String(mqtt_client.connected() ? "已连接" : "未连接"));
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
    
    // 初始化WiFi
    setup_wifi();
    
    // 初始化MQTT客户端
    Serial.println("\n配置MQTT客户端:");
    Serial.println("- 服务器: " + String(mqtt_server));
    Serial.println("- 端口: " + String(mqtt_port));
    Serial.println("- 客户端ID: " + client_id);
    mqtt_client.setServer(mqtt_server, mqtt_port);
    mqtt_client.setCallback(mqtt_callback);
    
    // 初始化I2S
    i2s_init();
    
    Serial.println("\n系统初始化完成");
    Serial.println("MQTT主题配置:");
    Serial.println("- 发送主题: " + stream_topic);
    Serial.println("- 接收主题: " + response_topic);
}

void handle_audio() {
    if (millis() - last_send_time < MQTT_SEND_INTERVAL) {
        return; // 控制发送频率
    }

    size_t bytes_read = 0;
    esp_err_t result = i2s_read(I2S_PORT_RX, audio_buffer, sizeof(audio_buffer), &bytes_read, 0);
    
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
        int sample_count = bytes_read / 2; // 16位 = 2字节
        
        for (int i = 0; i < sample_count; i++) {
            if (abs(samples[i]) > 500) { // 设置合适的阈值
                has_audio = true;
                break;
            }
        }

        if (has_audio) {
            // 分块发送数据，每次最多发送1024字节
            const int MAX_CHUNK_SIZE = 1024;
            uint8_t* data = (uint8_t*)audio_buffer;
            int remaining = bytes_read;
            int offset = 0;

            while (remaining > 0) {
                int chunk_size = min(remaining, MAX_CHUNK_SIZE);
                bool success = mqtt_client.publish(stream_topic.c_str(), &data[offset], chunk_size);
                
                if (success) {
                    total_bytes_sent += chunk_size;
                    packets_sent++;
                    if (DEBUG_AUDIO) {
                        Serial.println("音频发送成功: " + String(chunk_size) + " bytes");
                    }
                } else {
                    failed_sends++;
                    if (DEBUG_MQTT) {
                        Serial.println("MQTT发送失败 - 状态: " + String(mqtt_client.state()) + 
                                    " (" + String(mqtt_state_str(mqtt_client.state())) + ")");
                    }
                    break; // 如果发送失败，跳出循环
                }
                
                remaining -= chunk_size;
                offset += chunk_size;
                delay(5); // 短暂延迟，避免发送过快
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

    // 处理音频数据
    handle_audio();
    
    // 打印调试信息
    print_debug_info();
}