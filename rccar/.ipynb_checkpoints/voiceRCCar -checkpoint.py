# 음성 데이터 훈련 .. 
from pop import Dataset as ds

from pop import AI as ai
from pop import Util as util
import os
import numpy as np

# 음성명령 detection .. 
import pyaudio as pa
from array import array
from collections import deque
from queue import Queue, Full
from threading import Thread
import time
from pop import Pilot as rc

car = rc.AutoCar()
speed = 0
angle = 0

cmdSize = 10 # 전체 명령어 갯수 
cnn = ai.CNN(input_size=dataset_size, output_size=cmdSize)

# const valaues for silence detection
SILENCE_THREASHOLD = 2000
SILENCE_SECONDS = 2

    
# const values for mic streaming
CHUNK = 1024
BUFF = CHUNK * 10
FORMAT = pa.paInt16 
RATE = 48000

isDetected = False


def train_data() :
    xdata = []
    ydata = []

    datadir = "/home/soda/Work/study/recordset"
    datalist = os.listdir(datadir)
    
    for data in datalist:
        feat = util.toMFCC(datadir + "/" + data, duration=1)
        label = int(data.split("_")[0])
        label = util.one_hot(label, cmdSize)
        
        xdata.append(feat)
        ydata.append(label)
        
    xdata = np.array(xdata)
    ydata = np.array(ydata)

    dataset_size = xdata.shape[1:3]
    cnn = ai.CNN(input_size=dataset_size, output_size=cmdSize)    

    cnn.X_data = xdata
    cnn.Y_data = ydata

    cnn.train(times=500)
    cnn.save("digit_model")


# define listen function for threading
def listen(q):
    # open stream
    p = pa.PyAudio()
    stream = p.open(
        format=FORMAT,
        channels=1,
        rate=RATE,
        input=True,
        #input_device_index=2,
        frames_per_buffer=CHUNK
    )

    # FIXME: release initial noisy data (1sec)
    for _ in range(0, int(RATE / CHUNK)):
        data = stream.read(CHUNK, exception_on_overflow=False)

    is_started = False
    vol_que = deque(maxlen=SILENCE_SECONDS)

    print('start listening')
    while True:
        try:
            # define temporary variable to store sum of volume for 1 second 
            vol_sum = 0

            # read data for 1 second in chunk
            for _ in range(0, int(RATE / CHUNK)):
                data = stream.read(CHUNK, exception_on_overflow=False)

                # get max volume of chunked data and update sum of volume
                vol = max(array('h', data))
                vol_sum += vol

                # if status is listening, check the volume value
                if not is_started:
                    if vol >= SILENCE_THREASHOLD:
                        print('start of speech detected')
                        is_started = True

                # if status is speech started, write data
                if is_started:
                    #q.put(data)
                    q.append(data)

            # if status is speech started, update volume queue and check silence
            if is_started:
                vol_que.append(vol_sum / (RATE / CHUNK) < SILENCE_THREASHOLD)
                if len(vol_que) == SILENCE_SECONDS and all(vol_que):
                    print('end of speech detected')
                    isDetected = True
                    break
        except Full:
            pass

    # close stream
    stream.stop_stream()
    stream.close()
    p.terminate()

    if isDetected :
        # write to wav file 
        # write_wave(q)
        # check cmd 
        cmd = check_cmd(q)
        if cmd >= 0 :
            run_cmd(cmd)


def check_cmd(q):
    print("sound size : ")
    print(q.len())

    print("trained x size : ")
    print(cnn.X_data[ind].len())

    # cmd check .. 
    resInd = -1
    resVal = 0
    i = 0
    for a in r[0]:
        if resVal < a:
            resVal = a
            resInd = i
        i += 1

    print(resInd, resVal)   

    # 70% 이상일 경우 명령어로 인식함.. 
    if resVal > 0.7:
        return resInd
    else :
        return -1

# action for command 
# 0 : 전진
# 1 : 후진
# 2 : 멈춤
# 3 : 오른쪽 회전 
# 4 : 왼쪽 회전 
# 5 : 빠르게
# 6 : 느리게
# 7 : 따라와
# 8 : 춤춰
# 9 : 경보음 
def run_cmd(cmd):
    if cmd == 0:
        speed = 50
        car.forward(speed)
    elif cmd == 1:
        speed = 50
        car.backward(speed)
    elif cmd == 2:
        speed = 0
        car.stop()
    elif cmd == 3:
        angle += 0.1
        if angle > 1.0 :
            speed = 1.0
    elif cmd == 4:
        angle -= 0.1
        if angle < -1.0 :
            speed = -1.0
    elif cmd == 5:
        speed += 10
        if speed > 99 :
            speed = 99
    elif cmd == 6:
        speed -= 10
        if speed < 0 :
            speed = 0
    elif cmd == 7:
        # TODO : 사람 tracing .. 
        print('todo tracing..')
    elif cmd == 8:
        # TODO : 춤추기 액션 .. 
        print('todo dencing..')
    elif cmd == 9:
        # TODO : 위기 경보 .. 
        print('todo announce..')


def main():
    train_data()
    print("training is terminated~~")
     # loading trained data model.. 
    cnn.load("digit_model")
    print("digit_model was loaded~~")

    Thread(target=listen, args=(q,)).start()

if __name__ == '__main__':
    main()