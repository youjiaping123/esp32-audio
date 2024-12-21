#include <Arduino.h>
#include <driver/i2s.h>

// MAX98357A I2S 输出引脚定义
#define I2S_BCLK_OUT      26  // 输出位时钟
#define I2S_LRC_OUT       25  // 输出字时钟
#define I2S_DOUT          22  // 数据输出

// INMP411 I2S 输入引脚定义
#define I2S_BCLK_IN       14  // 输入位时钟
#define I2S_LRC_IN        15  // 输入字时钟
#define I2S_DIN           32  // 数据输入

// I2S 配置
#define SAMPLE_RATE       44100  // 采样率
#define SAMPLE_BITS       32     // 采样位数
#define CHANNELS          2      // 通道数
#define I2S_PORT_TX       I2S_NUM_0  // I2S输出端口
#define I2S_PORT_RX       I2S_NUM_1  // I2S输入端口

// 缓冲区大小
#define BUFFER_SIZE       512

void i2s_init() {
    // 配置I2S输出
    i2s_config_t i2s_config_tx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
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

    i2s_driver_install(I2S_PORT_TX, &i2s_config_tx, 0, NULL);
    i2s_set_pin(I2S_PORT_TX, &pin_config_tx);

    // 配置I2S输入
    i2s_config_t i2s_config_rx = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
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

    i2s_driver_install(I2S_PORT_RX, &i2s_config_rx, 0, NULL);
    i2s_set_pin(I2S_PORT_RX, &pin_config_rx);
}

void setup() {
    Serial.begin(115200);
    i2s_init();
    Serial.println("I2S系统初始化完成");
}

void loop() {
    // 创建缓冲区
    int32_t buffer[BUFFER_SIZE];
    size_t bytes_read = 0;
    size_t bytes_written = 0;
    
    // 从麦克风读取数据
    i2s_read(I2S_PORT_RX, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
    
    // 直接将数据写入扬声器
    i2s_write(I2S_PORT_TX, buffer, bytes_read, &bytes_written, portMAX_DELAY);
}