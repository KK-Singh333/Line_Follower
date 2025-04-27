import speech_recognition as sr
import paho.mqtt.client as mqtt
class Command():
    def __init__(self):
        self.recognizer=sr.Recognizer()
        # self.communicator=Communicate()
        self.sample_rate=16000
        self.chunk_size= 1024
        self.recognizer.dynamic_energy_threshold=False
        self.recognizer.dynamic_energy_adjustment_damping = 0.15
        self.recognizer.dynamic_energy_adjustment_ratio = 1.5
        self.recognizer.non_speaking_duration=0.5
        self.recognizer.pause_threshold = 0.8
        self.lang='en-GB'
        self.show_all=False
        self.broker_address='localhost'
        self.topic='robot/voice'
        self.client=mqtt.Client()
    def Start_Conversation(self):
        recognizer=self.recognizer
        client=self.client
        client.connect(self.broker_address)
        with sr.Microphone(device_index=1,sample_rate=self.sample_rate,chunk_size=self.chunk_size) as source:
            mic_list = sr.Microphone.list_microphone_names()
            mic_name = mic_list[1]  # 1 is device_index
            print(f"Using microphone: {mic_name}")
            source.SAMPLE_RATE = self.sample_rate
            source.CHUNK = self.chunk_size
            print('Calibarating...')
            recognizer.adjust_for_ambient_noise(source=source,duration=5)
            recognizer.energy_threshold=30
            print(f'Noise level is {recognizer.energy_threshold}')
            print('Start Talking')
            while True:
                audio_data=recognizer.listen(source=source,phrase_time_limit=None,timeout=None)
                print(f'Noise level is {recognizer.energy_threshold}')
                print('Mic Testing')
                print(audio_data)
                try:
                    text=recognizer.recognize_google(audio_data=audio_data,language=self.lang,show_all=self.show_all)
                    print(text)
                    client.publish(self.topic,text)
                except sr.UnknownValueError:
                    print("Sorry, I didn't understand that.")
                except sr.RequestError as e:
                    print(f"Could not request results from Google Speech Recognition service; {e}")
converse=Command()
converse.Start_Conversation()
