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
import matplotlib.pyplot as plt
import matplotlib

# Whisperの設定
whisper_options = whisper.DecodingOptions(language="japanese", fp16=False) # 型でエラーが起きる場合はfp16の値を変える
model = whisper.load_model("small").to("cuda")
LOGPROB_THRESHOLD = -1.0
NO_SPEECH_PROB_THRESHOLD = 0.6

# VADの設定
VAD_POWER_THRESHOLD = 2000000
VAD_SILLEN_THRESHOLD = 10  # 15が約1秒
VAD_MIN_LENGTH_SPEECH = 15 # 15が約1秒
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

# 表示設定
matplotlib.use('TkAgg')
GRAPH_MAX_LEN = 150


def record( stream ):
    frames = []

    is_active = False
    activity_list = []
    
    # グラフ表示用
    disp_powers = []
    dsip_acitivities = []
    
    data = stream.read(CHUNK)
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
        else:
            # 発話途中からの入力にならないように，最新の2つは保持する
            frames.append(data)
            if len(frames)>2:
                frames.pop(0)

        sys.stdout.flush()

        # グラフ表示
        disp_powers.append(power)
        if len(disp_powers)>GRAPH_MAX_LEN:
            disp_powers.pop(0)

        dsip_acitivities.append(int(is_active))
        if len(dsip_acitivities)>GRAPH_MAX_LEN:
            dsip_acitivities.pop(0)

        plt.clf()
        plt.plot(range(len(disp_powers)), disp_powers )
        plt.plot( [0,GRAPH_MAX_LEN], [VAD_POWER_THRESHOLD, VAD_POWER_THRESHOLD] )
        plt.fill_between( range(len(dsip_acitivities)), np.zeros(len(dsip_acitivities)), np.array(dsip_acitivities)*VAD_POWER_THRESHOLD*1.5, color="r", alpha=0.3 )
        plt.ylim( 0, VAD_POWER_THRESHOLD*1.5  )
        plt.draw()
        plt.pause(0.001)


    stream.stop_stream()
        
    return frames

def recognize(wave):
    start = time.time()
    wave = whisper.pad_or_trim(wave/32767)
    mel = whisper.log_mel_spectrogram(wave).to(model.device)
    #_, probs = model.detect_language(mel)
    result = whisper.decode(model, mel, whisper_options)

    print(f"text = {result.text}")
    print(f"time = {time.time()-start} sec")
    print(f"log_prob = {result.avg_logprob}")
    print(f"no_speech_prob = {result.no_speech_prob}")

    if result.avg_logprob < LOGPROB_THRESHOLD or result.no_speech_prob>NO_SPEECH_PROB_THRESHOLD:
        print("  -> Rejected...")
        return None
    else:
        print("  -> Accepted")
        return result 


def main():
    stream = audio.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    input_device_index = DEVICE_INDEX,
                    frames_per_buffer=1024)

    while not rospy.is_shutdown():
        print("--------- Recoding ----------")
        wave = record(stream)
        print("-------- Recognizing --------")
        result = recognize( wave )
        print("-----------------------------")
        print("")

        if result is not None:
            pub_recres.publish( result )
            pub_recres_nbest.publish( yaml.dump([{"text":result.text, "conf":result.avg_logprob}]) )

    stream.close()

    return 


if __name__ == "__main__":
    rospy.init_node('whisper_recognizer' )
    pub_recres = rospy.Publisher('whisper_recognizer/recres', String, queue_size=10)
    pub_recres_nbest = rospy.Publisher('whisper_recognizer/recres_nbest', String, queue_size=10)
    main()

