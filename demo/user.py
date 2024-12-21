import paho.mqtt.client as mqtt
import os 
import wave
import pyaudio
from dotenv import load_dotenv
import msvcrt
import tempfile
from datetime import datetime
import io
import uuid  # 新增
import requests  # 添加到文件开头的导入部分

load_dotenv()

def log(message):
    """带时间戳的日志打印"""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    print(f"[{timestamp}] {message}")

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        log("已连接到MQTT服务器")
        client.subscribe("voice/response")

def record_and_stream_audio(client):
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000

    p = pyaudio.PyAudio()
    log("\n按空格键开始/停止录音...")

    stream = p.open(format=FORMAT,
                   channels=CHANNELS,
                   rate=RATE,
                   input=True,
                   frames_per_buffer=CHUNK)

    while msvcrt.kbhit():
        msvcrt.getch()
    
    while True:
        if msvcrt.kbhit() and msvcrt.getch() == b' ':
            break
    
    log("开始录音中...")
    recording = True
    while recording:
        try:
            data = stream.read(CHUNK, exception_on_overflow=False)
            client.publish("voice/stream", data)
            
            if msvcrt.kbhit() and msvcrt.getch() == b' ':
                log("停止录音...")
                recording = False
                    
        except Exception as e:
            log(f"录音出错: {str(e)}")
            recording = False

    stream.stop_stream()
    stream.close()
    p.terminate()
    client.publish("voice/stream", b"END_OF_STREAM")
    log("录音结束\n")

class AudioPlayer:
    def __init__(self):
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.is_playing = False
        
        # 初始化默认音频参数
        self.channels = 1
        self.sample_width = 2
        self.sample_rate = 16000
        self.chunk_size = 2048
        
        # 创建输出流
        self.create_stream()
    
    def create_stream(self):
        """创建一个新的音频流"""
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
        
        # 使用较大的缓冲区
        self.stream = self.p.open(
            format=self.p.get_format_from_width(self.sample_width),
            channels=self.channels,
            rate=self.sample_rate,
            output=True,
            frames_per_buffer=self.chunk_size,
            start=False  # 延迟启动流
        )
        # 启动流并写入初始静音
        self.stream.start_stream()
        initial_silence = b'\x00' * (self.chunk_size * self.sample_width * 4)  # 4个缓冲区的静音
        self.stream.write(initial_silence)
    
    def play_wav(self, wav_data):
        try:
            # wav_data 现在是 BytesIO 对象而不是文件路径
            with wave.open(wav_data, 'rb') as wf:
                # 检查音频格式
                if wf.getnchannels() != self.channels or \
                   wf.getsampwidth() != self.sample_width or \
                   wf.getframerate() != self.sample_rate:
                    log(f"警告: 音频格式不匹配 (通道:{wf.getnchannels()}, 位宽:{wf.getsampwidth()}, ��样率:{wf.getframerate()})")
                
                # 增加淡入淡出时长到25ms
                fade_length = int(0.025 * self.sample_rate)
                
                # 读取所有音频数据
                audio_data = wf.readframes(wf.getnframes())
                
                # 转换为数组以便处理
                import array
                import math
                samples = array.array('h', audio_data)
                
                # 使用更平滑的淡入曲线
                for i in range(min(fade_length, len(samples))):
                    # 使用正弦平方函数实现更平滑的淡入
                    factor = math.sin((i / fade_length) * math.pi / 2) ** 2
                    samples[i] = int(samples[i] * factor)
                
                # 使用更平滑的淡出曲线
                for i in range(min(fade_length, len(samples))):
                    factor = math.sin((i / fade_length) * math.pi / 2) ** 2
                    samples[-(i+1)] = int(samples[-(i+1)] * factor)
                
                # 转回字节数据
                processed_data = samples.tobytes()
                
                # 写入较长的前置静音
                pre_silence = b'\x00' * (self.chunk_size * self.sample_width * 2)
                self.stream.write(pre_silence)
                
                # 分块播放处理后的音频
                for i in range(0, len(processed_data), self.chunk_size * self.sample_width):
                    chunk = processed_data[i:i + self.chunk_size * self.sample_width]
                    if len(chunk) < self.chunk_size * self.sample_width:
                        # 填充最后一个块
                        chunk = chunk + b'\x00' * (self.chunk_size * self.sample_width - len(chunk))
                    self.stream.write(chunk)
                
                # 写入较长的后置静音
                post_silence = b'\x00' * (self.chunk_size * self.sample_width * 2)
                self.stream.write(post_silence)
                
        except Exception as e:
            log(f"播放音频出错: {str(e)}")
            # 如果出错，重新创建流
            self.create_stream()
    
    def close(self):
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()
    
    def play_from_url(self, url):
        """从URL下载并播放音频"""
        try:
            log(f"正在从URL下载音频: {url}")
            response = requests.get(url, timeout=10)
            if response.status_code == 200:
                # 将下载的数据转换为BytesIO对象
                wav_data = io.BytesIO(response.content)
                self.play_wav(wav_data)
            else:
                log(f"下载音频失败: HTTP {response.status_code}")
        except Exception as e:
            log(f"播放URL音频出错: {str(e)}")

# 创建全局AudioPlayer实例
audio_player = None

def on_message(client, userdata, msg):
    global audio_player
    if msg.topic == "voice/response":
        try:
            log("收到语音回复，正在播放...")
            # 直接使用 BytesIO 在内存中处理音频数据
            wav_data = io.BytesIO(msg.payload)
            audio_player.play_wav(wav_data)
            log("播放完成")
            
        except Exception as e:
            log(f"播放音频出错: {str(e)}")

class VoiceClient:
    def __init__(self):
        # 生成唯一的客户端ID
        self.client_id = f"voice_client_{str(uuid.uuid4())[:8]}"
        self.mqtt_client = mqtt.Client(client_id=self.client_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.audio_player = AudioPlayer()
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            log(f"客户端 {self.client_id} 已连接到MQTT服务器")
            # 订阅特定于该客户端的响应主题
            client.subscribe(f"voice/response/{self.client_id}")
            
    def on_message(self, client, userdata, msg):
        if msg.topic == f"voice/response/{self.client_id}":
            try:
                # 现在收到的是URL字符串而不是音频数据
                audio_url = msg.payload.decode('utf-8')
                log(f"收到音频URL: {audio_url}")
                self.audio_player.play_from_url(audio_url)
                log("播放完成")
            except Exception as e:
                log(f"播放音频出错: {str(e)}")
                
    def record_and_stream_audio(self):
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000

        p = pyaudio.PyAudio()
        log("\n按空格键开始/停止录音...")

        stream = p.open(format=FORMAT,
                       channels=CHANNELS,
                       rate=RATE,
                       input=True,
                       frames_per_buffer=CHUNK)

        while msvcrt.kbhit():
            msvcrt.getch()
        
        while True:
            if msvcrt.kbhit() and msvcrt.getch() == b' ':
                break
        
        log("开始录音中...")
        recording = True
        while recording:
            try:
                data = stream.read(CHUNK, exception_on_overflow=False)
                # 发布到包含客户端ID的主题
                self.mqtt_client.publish(f"voice/stream/{self.client_id}", data)
                
                if msvcrt.kbhit() and msvcrt.getch() == b' ':
                    log("停止录音...")
                    recording = False
                        
            except Exception as e:
                log(f"录音出错: {str(e)}")
                recording = False

        stream.stop_stream()
        stream.close()
        p.terminate()
        self.mqtt_client.publish(f"voice/stream/{self.client_id}", b"END_OF_STREAM")
        log("录音结束\n")
        
    def start(self):
        mqtt_broker = os.getenv("MQTT_BROKER", "localhost")
        mqtt_port = int(os.getenv("MQTT_PORT", "1883"))
        
        log(f"客户端 {self.client_id} 正在连接到MQTT服务器 {mqtt_broker}:{mqtt_port}")
        self.mqtt_client.connect(mqtt_broker, mqtt_port)
        self.mqtt_client.loop_start()
        
        try:
            while True:
                self.record_and_stream_audio()
        except KeyboardInterrupt:
            pass
        finally:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.audio_player.close()

if __name__ == "__main__":
    client = VoiceClient()
    client.start()               