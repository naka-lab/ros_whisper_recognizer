#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pyaudio
import time
import sys
import numpy as np
import whisper
import yaml
import rospy
from std_msgs.msg import String

# Whisperの設定
whisper_options = whisper.DecodingOptions(language="japanese")
model = whisper.load_model("small").to("cuda")

# VADの設定
VAD_POWER_THRESHOLD = 2000000
VAD_SILLEN_THRESHOLD = 10  # 15が約1秒
VAD_MIN_LENGTH_SPEECH = 10 # 15が約1秒
VAD_MAX_LENGTH_SPEECH = 150 # 15が約1秒

# 録音設定
RATE = 16000
FORMAT = pyaudio.paInt16 # 16bit
CHANNELS = 1             # モノラル
CHUNK = 1024
DEVICE_INDEX = 0

# デバイスの選択
audio = pyaudio.PyAudio()
for x in range(0, audio.get_device_count()): 
    info = audio.get_device_info_by_index(x)
    print(info["index"], ":" , info["name"])
DEVICE_INDEX = int( input("Select device -> ") )


def record( stream ):
    frames = []

    is_active = False
    activity_list = []
    
    while not rospy.is_shutdown():
        data = stream.read(CHUNK)
        data = np.array(np.frombuffer(data, dtype=np.int16), dtype=np.float32 )
        power = np.sum(data**2)/CHUNK

        if power>VAD_POWER_THRESHOLD:
            is_active = True
            activity_list.append(1)
        elif is_active:
            activity_list.append(0)
        else:
            activity_list = []
        
        if is_active and len(activity_list)>VAD_SILLEN_THRESHOLD and np.sum(activity_list[-VAD_SILLEN_THRESHOLD:])==0:
            if len(activity_list)>VAD_MIN_LENGTH_SPEECH:
                print(" Detected!")
                return np.hstack( frames )
            else:
                print(" Too short....")
            is_active = False
            activity_list = []
            frames = []

        elif is_active and len(activity_list)>VAD_MAX_LENGTH_SPEECH:
            is_active = False
            activity_list = []
            frames = []
            print( " Too long..." )
        
        if is_active:
            print(">", end="")
            frames.append( data )
        sys.stdout.flush()

    stream.stop_stream()
        
    return frames

def recognize(wave):
    start = time.time()
    wave = whisper.pad_or_trim(wave/32767)
    mel = whisper.log_mel_spectrogram(wave).to(model.device)
    #_, probs = model.detect_language(mel)
    result = whisper.decode(model, mel, whisper_options)

    print(f"  -> Recognition result:{result.text} [{time.time()-start} sec]")

    return result.text


def main():
    stream = audio.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    input_device_index = DEVICE_INDEX,
                    frames_per_buffer=1024)

    while not rospy.is_shutdown():
        wave = record(stream)
        result = recognize( wave )

        pub_recres.publish( result )
        pub_recres_nbest.publish( yaml.dump([{"text":result, "conf":0.0}]) )

    stream.close()

    return 


if __name__ == "__main__":
    rospy.init_node('whisper_recognizer' )
    pub_recres = rospy.Publisher('whisper_recognizer/recres', String, queue_size=10)
    pub_recres_nbest = rospy.Publisher('whisper_recognizer/recres_nbest', String, queue_size=10)
    main()

