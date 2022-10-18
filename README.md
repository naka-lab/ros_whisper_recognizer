# ros_whisper_recognzer

[Whisper](https://github.com/openai/whisper)を使った音声認識ノード

## インストール
```
pip install git+https://github.com/openai/whisper.git --use-feature=2020-resolver
pip install pyaudio

cd ~/catkin_ws/src
git clone https://github.com/naka-lab/ros_whisper_recognizer.git
```

## 実行
- 音声認識単体で実行：`rosrun ros_whisper_recognizer recognizer.py`
- [言語理解ノード](https://github.com/naka-lab/ros_google_speech#%E6%96%87%E6%B3%95%E3%83%99%E3%83%BC%E3%82%B9%E3%81%AE%E8%A8%80%E8%AA%9E%E7%90%86%E8%A7%A3)と同時起動：`roslaunch ros_whisper_recognizer language_understanding.launch`

## トピック
- Publish
  - `whisper_recognizer/recres`: 認識結果の文字列
  - `whisper_recognizer/recres_nbest`:認識結果の文字列（[Google音声認識](https://github.com/naka-lab/ros_google_speech)と同様のyaml形式）
- Subscribe
  - なし
