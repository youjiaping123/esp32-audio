import paho.mqtt.client as mqtt
import azure.cognitiveservices.speech as speechsdk
import io
from openai import OpenAI
from dotenv import load_dotenv
import os
import wave
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
import threading
from queue import Queue
import hashlib
from pathlib import Path

load_dotenv()

def log(message):
    """带时间戳的日志打印"""
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {message}")

class VoiceAIChatbot:
    def __init__(self):
        # 初始化配置
        self.setup_speech_service()
        self.setup_ai_service()
        self.setup_mqtt_client()
        
        # 会话和线程管理
        self.client_sessions = {}
        self.executor = ThreadPoolExecutor(max_workers=10)
        self.message_queue = Queue()
        
        # 在初始化时添加音频文件存储路径配置
        self.audio_dir = Path("static/audio")
        self.audio_dir.mkdir(parents=True, exist_ok=True)
        self.base_url = os.getenv("AUDIO_BASE_URL", "http://localhost:8080/static/audio")
        
        # 启动消息处理线程
        self.processing_thread = threading.Thread(target=self.process_messages)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def setup_speech_service(self):
        """初始化语音服务"""
        self.speech_config = speechsdk.SpeechConfig(
            subscription=os.getenv('SPEECH_KEY'), 
            region=os.getenv('SPEECH_REGION')
        )
        self.speech_config.speech_recognition_language = "zh-CN"

    def setup_ai_service(self):
        """初始化AI服务"""
        self.ai_client = OpenAI(
            api_key=os.getenv("API_KEY"),
            base_url=os.getenv("BASE_URL")
        )

    def setup_mqtt_client(self):
        """初始化MQTT客户端"""
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, reason_code, properties):
        log(f"已连接到MQTT服务器, 返回码: {reason_code}")
        client.subscribe("voice/stream/+")

    def process_messages(self):
        """处理消息队列"""
        while True:
            try:
                client_id, text = self.message_queue.get()
                if client_id and text:
                    self.executor.submit(self.handle_recognition, client_id, text)
            except Exception as e:
                log(f"处理消息出错: {str(e)}")

    def handle_recognition(self, client_id, text):
        """处理语音识别结果"""
        try:
            log(f"客户端 {client_id} 语音识别结果: {text}")
            ai_response = self.get_ai_response(text)
            log(f"AI回复: {ai_response}")
            self.text_to_speech(ai_response, client_id)
        except Exception as e:
            log(f"处理识别结果出错: {str(e)}")

    def text_to_speech(self, text, client_id):
        """语音合成并返回音频URL"""
        try:
            speech_config = speechsdk.SpeechConfig(
                subscription=os.getenv('SPEECH_KEY'), 
                region=os.getenv('SPEECH_REGION')
            )
            speech_config.speech_synthesis_voice_name = "zh-CN-YunzeNeural"
            speech_config.set_speech_synthesis_output_format(
                speechsdk.SpeechSynthesisOutputFormat.Riff16Khz16BitMonoPcm
            )
            
            synthesizer = speechsdk.SpeechSynthesizer(
                speech_config=speech_config,
                audio_config=None
            )
            
            result = synthesizer.speak_text_async(text).get()
            
            if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
                # 生成带时间戳的文件名
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                # 生成短的随机字符串
                random_str = hashlib.md5(text.encode()).hexdigest()[:4]
                # 组合文件名: 时间戳_随机字符.wav
                filename = f"{timestamp}_{random_str}.wav"
                filepath = self.audio_dir / filename
                
                # 保存音频文件
                with wave.open(str(filepath), 'wb') as wav_file:
                    wav_file.setnchannels(1)
                    wav_file.setsampwidth(2)
                    wav_file.setframerate(16000)
                    wav_file.writeframes(result.audio_data)
                
                # 生成URL (使用修改后的base_url)
                audio_url = f"{self.base_url}/{filename}"
                
                # 发送URL给客户端
                self.mqtt_client.publish(f"voice/response/{client_id}", audio_url)
                log(f"已发送音频URL: {audio_url}")
                
        except Exception as e:
            log(f"语音合成错误: {str(e)}")

    def start_stream_recognition(self, client_id):
        """启动语音识别流"""
        try:
            push_stream = speechsdk.audio.PushAudioInputStream()
            audio_config = speechsdk.audio.AudioConfig(stream=push_stream)
            
            speech_recognizer = speechsdk.SpeechRecognizer(
                speech_config=self.speech_config, 
                audio_config=audio_config
            )
            
            def handle_result(evt):
                if evt.result.reason == speechsdk.ResultReason.RecognizedSpeech and evt.result.text.strip():
                    self.message_queue.put((client_id, evt.result.text))
            
            speech_recognizer.recognized.connect(handle_result)
            speech_recognizer.start_continuous_recognition()
            
            self.client_sessions[client_id] = {
                'push_stream': push_stream,
                'speech_recognizer': speech_recognizer,
                'is_recognizing': True
            }
            
        except Exception as e:
            log(f"启动语音识别出错: {str(e)}")
            self.stop_stream_recognition(client_id)

    def stop_stream_recognition(self, client_id):
        """停止语音识别流"""
        if client_id in self.client_sessions:
            session = self.client_sessions[client_id]
            if session['is_recognizing']:
                session['speech_recognizer'].stop_continuous_recognition()
            session['push_stream'].close()
            del self.client_sessions[client_id]

    def on_message(self, client, userdata, msg):
        """处理MQTT消息"""
        client_id = msg.topic.split('/')[-1]
        try:
            if msg.payload == b"END_OF_STREAM":
                self.stop_stream_recognition(client_id)
                return
                
            if client_id not in self.client_sessions:
                self.start_stream_recognition(client_id)
                
            if client_id in self.client_sessions:
                self.client_sessions[client_id]['push_stream'].write(msg.payload)
                
        except Exception as e:
            log(f"处理音频出错: {str(e)}")
            self.stop_stream_recognition(client_id)

    def get_ai_response(self, message):
        """��取AI回复"""
        try:
            response = self.ai_client.chat.completions.create(
                model="anthropic/claude-3.5-haiku-20241022:beta",
                messages=[
                    {"role": "system", "content": "接下来你将扮演五星上将麦克阿瑟，为给定的主题提供幽默而略带荒谬的纪录片风格评论。评论的长度必须在30-50个汉字之间。"},
                    {"role": "user", "content": message}
                ],
                timeout=10
            )
            return response.choices[0].message.content
        except Exception as e:
            log(f"AI调用出错: {str(e)}")
            return "抱歉,我现在无法回答。请稍后再试。"

    def start(self):
        """启动服务"""
        mqtt_broker = os.getenv("MQTT_BROKER", "localhost")
        mqtt_port = int(os.getenv("MQTT_PORT", "1883"))
        
        self.mqtt_client.connect(mqtt_broker, mqtt_port)
        
        try:
            self.mqtt_client.loop_forever()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """停止服务"""
        self.executor.shutdown(wait=True)
        for client_id in list(self.client_sessions.keys()):
            self.stop_stream_recognition(client_id)
        self.mqtt_client.disconnect()

if __name__ == "__main__":
    chatbot = VoiceAIChatbot()
    try:
        chatbot.start()
    except KeyboardInterrupt:
        chatbot.stop() 