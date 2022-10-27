#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pyaudio
import time
import sys
import os
import numpy as np
import whisper
import yaml
import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib
import torch

# Whisperの設定
DEVICE = "cuda"
whisper_options = whisper.DecodingOptions(language="japanese", fp16=False)
model = whisper.load_model("small").to(DEVICE)
LOGPROB_THRESHOLD = -1.0
NO_SPEECH_PROB_THRESHOLD = 0.6

# VADの設定
torch.set_grad_enabled(False)
vad_model = torch.jit.load( os.path.join(os.path.dirname(__file__), "silero_vad.jit"), map_location=DEVICE )
vad_model.eval()

VAD_PROB_THRESHOLD = 0.5
VAD_SILLEN_THRESHOLD = 5  # 15が約1秒
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
    chunks = []
    is_active = False
    num_silent_frames = 0
    
    # グラフ表示用
    powers = []
    acitivities = []
    vad_probs = []
    
    stream.start_stream()
    data = stream.read(CHUNK)
    while not rospy.is_shutdown():
        data = stream.read(CHUNK)
        data = np.array(np.frombuffer(data, dtype=np.int16), dtype=np.float32 )/32767

        m = np.max(np.abs(data))
        if m!=0:
            data_vad = data/m
        else:
            data_vad = data

        vad_prob = vad_model(torch.from_numpy(data_vad).to(DEVICE),RATE)[0][0].item()

        if vad_prob>VAD_PROB_THRESHOLD:
            is_active = True
            num_silent_frames = 0
        elif is_active:
            num_silent_frames += 1
        else:
            num_silent_frames = 0
        
        if is_active and num_silent_frames>VAD_SILLEN_THRESHOLD:
            if len(chunks)>VAD_MIN_LENGTH_SPEECH:
                print(" Detected!")
                stream.stop_stream()
                return np.hstack( chunks )
            else:
                print(" Too short....")
            is_active = False
            num_silent_frames = 0
            chunks = []

        elif is_active and len(chunks)>VAD_MAX_LENGTH_SPEECH:
            is_active = False
            num_silent_frames = 0
            chunks = []
            print( " Too long..." )
        
        if is_active:
            print(">", end="")
            chunks.append( data )
        else:
            # 発話途中からの入力にならないように，最新の2つは保持する
            chunks.append(data)
            if len(chunks)>2:
                chunks.pop(0)

        sys.stdout.flush()

        # グラフ表示
        powers.append( np.sum(data**2)/CHUNK )
        if len(powers)>GRAPH_MAX_LEN:
            powers.pop(0)

        acitivities.append(int(is_active))
        if len(acitivities)>GRAPH_MAX_LEN:
            acitivities.pop(0)

        vad_probs.append(vad_prob)
        if len(vad_probs)>GRAPH_MAX_LEN:
            vad_probs.pop(0)

        plt.clf()
        plt.plot( range(len(powers)), powers, label="power" )
        plt.plot( range(len(vad_probs)), vad_probs, label="vad_prob" )
        plt.plot( [0,GRAPH_MAX_LEN], [VAD_PROB_THRESHOLD, VAD_PROB_THRESHOLD], label="vad_prob_threshold" )
        plt.fill_between( range(len(acitivities)), np.zeros(len(acitivities)), np.array(acitivities), color="r", alpha=0.3 )
        plt.ylim( 0, 1 )
        plt.legend()
        plt.draw()
        plt.pause(0.001)

    stream.stop_stream()
    return None


def recognize(wave):
    start = time.time()
    wave = whisper.pad_or_trim(wave)
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

