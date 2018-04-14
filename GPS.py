import math
from math import sin, cos, sqrt, atan2, radians, asin
import time
import os
import smbus
import RPi.GPIO as GPIO
import serial
import sys
import os
import subprocess
from pygame import mixer

#コンパスのx,y軸の誤差
compass_xoffset = 15.0 
compass_yoffset = -164.5

#入出力ピン番号設定
yellow = 13
green = 19
white = 20
blue = 21
button = 26
led = 5
trigger = 24
echo = 23
servo180 = 17  # white
servo_left = 27  # yellow
servo_right = 22  # blue
servo0 = 25  # orange

# GPIO入出力設定
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(yellow, GPIO.OUT)
GPIO.setup(green, GPIO.OUT)
GPIO.setup(white, GPIO.OUT)
GPIO.setup(blue, GPIO.OUT)
GPIO.setup(led, GPIO.OUT)
GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(trigger, GPIO.OUT)
GPIO.output(trigger, 0)
GPIO.setup(echo, GPIO.IN)
GPIO.setup(servo180, GPIO.OUT)
GPIO.setup(servo_left, GPIO.OUT)
GPIO.setup(servo_right, GPIO.OUT)
GPIO.setup(servo0, GPIO.OUT)
GPIO.output(servo180, GPIO.LOW)
GPIO.output(servo_left, GPIO.LOW)
GPIO.output(servo_right, GPIO.LOW)
GPIO.output(servo0, GPIO.LOW)
led_pulse = GPIO.PWM(led, 10)
led_pulse.start(0)

#目的地の設定
destilatlong = (35.897401, 140.064851, 35.897244, 140.064793, 35.897048, 140.064965, 35.897048, 140.065103, 35.897145,
                140.065165,35.897284, 140.065233, 35.897310, 140.065364, 35.897289, 140.065498)

#この関数を呼び出すと前方に動く
def gpio_gofront():
    GPIO.output(yellow, GPIO.LOW)
    GPIO.output(green, GPIO.HIGH)
    GPIO.output(blue, GPIO.HIGH)
    GPIO.output(white, GPIO.LOW)
    print('前方')
#この関数を呼び出すと左方に曲がる
def gpio_spinleft():
    GPIO.output(yellow, GPIO.LOW)
    GPIO.output(green, GPIO.HIGH)
    GPIO.output(blue, GPIO.LOW)
    GPIO.output(white, GPIO.HIGH)
    print('左方')
#この関数を呼び出すと右方に曲がる
def gpio_spinright():
    GPIO.output(yellow, GPIO.HIGH)
    GPIO.output(green, GPIO.LOW)
    GPIO.output(blue, GPIO.HIGH)
    GPIO.output(white, GPIO.LOW)
    print('右方')
#この関数を呼び出すと停止する
def gpio_stop():
    GPIO.output(yellow, GPIO.LOW)
    GPIO.output(green, GPIO.LOW)
    GPIO.output(blue, GPIO.LOW)
    GPIO.output(white, GPIO.LOW)
    print('停止')
#この関数を呼び出すと後方に動く
def gpio_goback():
    GPIO.output(yellow, GPIO.HIGH)
    GPIO.output(green, GPIO.LOW)
    GPIO.output(blue, GPIO.LOW)
    GPIO.output(white, GPIO.HIGH)
    print('後方')

#超音波センサーを使い距離を求める関数
def ultrasonic_distance():
    time.sleep(0.1)
    GPIO.output(trigger, 1)
    time.sleep(0.00001)
    GPIO.output(trigger, 0)
    while GPIO.input(echo) == 0:
        pass
    ultrasonic_start = time.time()
    while GPIO.input(echo) == 1:
        pass
    ultrasonic_stop = time.time()
    timed = (ultrasonic_stop - ultrasonic_start) / 2
    distance = timed * 34300
    return distance

#サーボを制御 angleには角度やサーボの向きを代入する
def servo(angle):
    if angle == '180':
        GPIO.output(servo180, GPIO.HIGH)
        print('180')
        time.sleep(0.5)

    if angle == 'left':
        GPIO.output(servo_left, GPIO.HIGH)
        print('left')
        time.sleep(0.5)

    if angle == 'right':
        GPIO.output(servo_right, GPIO.HIGH)
        print('right')
        time.sleep(0.5)

    if angle == '0':
        GPIO.output(servo0, GPIO.HIGH)
        print('0')
        time.sleep(0.5)

#サーボを90度に戻す関数
def servo_return():
    GPIO.output(servo180, GPIO.LOW)
    GPIO.output(servo_left, GPIO.LOW)
    GPIO.output(servo_right, GPIO.LOW)
    GPIO.output(servo0, GPIO.LOW)

#液晶ディスプレイを制御する関数
def LCD_setup():
    bus = smbus.SMBus(1)
    time.sleep(0.1)
    bus.write_i2c_block_data(0x3e, 0x00, [0x38, 0x39, 0x14, 0x70, 0x56, 0x6c])
    time.sleep(0.3)
    bus.write_i2c_block_data(0x3e, 0x00, [0x38, 0x0c, 0x01])
    bus.write_i2c_block_data(0x3e, 0x00, [0x05, 0x01])
def WRITE_LCD(x, y):
    z = 0x80 + y * 0x40 + x
    bus = smbus.SMBus(1)
    try:
        bus.write_i2c_block_data(0x3e, 0x00, [z])
    except IOError:
        #    print 'error1'
        return -1
def DISPLAY_LCD(a):
    bus = smbus.SMBus(1)
    for c in a:
        try:
            bus.write_i2c_block_data(0x3e, 0x40, [ord(c)])
        except IOError:
            print ('液晶エラー')
            return -1

#液晶・GPIO初期設定
gpio_stop()
LCD_setup()

#目的地の方角を算出する関数
def desti_compass_bearing(PointA, PointB):
    localat1 = math.radians(PointA[0])
    destilat1 = math.radians(PointB[0])
    sublongi = math.radians(PointB[1] - PointA[1])
    x_localat = math.sin(sublongi) * math.cos(destilat1)
    y_localat = math.cos(localat1) * math.sin(destilat1) - (math.sin(localat1) * math.cos(destilat1) *
                                                            math.cos(sublongi))
    initial_bearing = math.atan2(x_localat, y_localat)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

#目的地までの距離を求める関数
def haversine_distance(localongD, localatD, destilongD, destilatD):
    localongD, localatD, destilongD, destilatD = map(radians, [localongD, localatD, destilongD, destilatD])
    dlon = destilongD - localongD
    dlat = destilatD - localatD
    a = sin(dlat / 2) ** 2 + cos(localatD) * cos(destilatD) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    earth_radius = 6371
    distance = earth_radius * c * 1000
    return distance

#GPSモジュールからのデータを正しい形式に変換する関数
def degree_minutes(data_loca):
    deg = int(data_loca) // 100
    min = data_loca - 100 * deg
    return deg, min
def minute_decimal(deg, min):
    return deg + min / 60

#音声出力関数 tに代入された文字を出力する
def voice_output(t):
    open_jtalk = ['open_jtalk']
    mech = ['-x', '/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice = ['-m', '/usr/share/hts-voice/mei/mei_normal.htsvoice']
    speed = ['-r', '0.9']
    outwav = ['-ow', 'open_jtalk.wav']
    cmd = open_jtalk + mech + htsvoice + speed + outwav
    talk = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    talk.stdin.write(t.encode('utf-8'))
    talk.stdin.close()
    talk.wait()
    aplay = ['aplay', '-q', 'open_jtalk.wav']
    wr = subprocess.Popen(aplay)

#コンパスからのx,y,z軸のデータを使い方角を算出する関数
def compass_program():
    bus = smbus.SMBus(1)
    address = 0x1e

    def read_byte(adr):
        return bus.read_byte_data(address, adr)

    def read_word(adr):
        high = bus.read_byte_data(address, adr)
        low = bus.read_byte_data(address, adr + 1)
        val = (high << 8) + low
        return val

    def read_word_2c(adr):
        val = read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def write_byte(adr, value):
        bus.write_byte_data(address, adr, value)

    write_byte(0, 0b01110000)  #受信設定 8サンプル@15Hz
    write_byte(1, 0b00100000)
    write_byte(2, 0b00000000)
    scale = 0.92
    x_offset = compass_xoffset
    y_offset = compass_yoffset
    x_out = (read_word_2c(3) - x_offset) * scale
    y_out = (read_word_2c(7) - y_offset) * scale
    z_out = (read_word_2c(5)) * scale
    bearing = math.atan2(y_out, x_out)
    # bearing = bearing + 0.275253
    if (bearing < 0):
        bearing += 2 * math.pi
    compassdata = math.degrees(bearing)
    return compassdata

#現在向いている方角と目的地の方角の誤差を求める関数
def compasscurrentdestisub(current, destination):
    sub = current - destination
    if (sub < 0):
        sub = sub + 360
    return sub

#現在の方角から目的地の方角の差を求めるプログラム
def rawcompasscurrentdestisub(rawcurrent, rawdestination):
    rawsub = rawcurrent - rawdestination
    return rawsub

#シリアルポートからデータを受信 (9600ボー[baud])
serial_data_recieve = serial.Serial('/dev/serial0', 9600)

global a
#メインプログラム
try:
    while True:
        g = 0
        loopnum =  len(destilatlong)
        previous_x = 0
        input_state = GPIO.input(button)
        if input_state == False:
            WRITE_LCD(0, 0)
            DISPLAY_LCD('READY   ')
            for g in range(3, 0, -1):
                WRITE_LCD(0, 1)
                DISPLAY_LCD('{0}        '.format(g))
                led_pulse.ChangeDutyCycle(1)
                time.sleep(1)
            print('START')
            WRITE_LCD(0, 1)
            DISPLAY_LCD('START   ')
            distance = ultrasonic_distance()
            print(distance)
            servo('180')
            servo_return()
            servo('0')
            servo_return()
            mixer.init()
            mixer.music.load('/home/pi/sound.mp3')
            mixer.music.play()
            time.sleep(3)
            led_pulse.ChangeDutyCycle(50)
            # voice_output('運転開始')
            for g in range(0, loopnum, 2):
                a = 0
                while True:
                    print('\n')
                    print('-------------------------------ループ---------------------------------')
                    line = str(serial_data_recieve.readline())
                    gps_data = line.split(',')
                    if '$PUBX' in line:
                        latitude = float(gps_data[3])
                        longitude = float(gps_data[5])
                        currentSattelite = gps_data[18]

                        localat = minute_decimal(*degree_minutes(latitude))
                        localong = minute_decimal(*degree_minutes(longitude))

                        desticompass = desti_compass_bearing((localat, localong),
                                                             (destilatlong[g], destilatlong[g + 1]))
                        x = haversine_distance(localat, localong, destilatlong[g], destilatlong[g + 1])
                        jtalk_distance_sum = previous_x - x
                        compass_program()
                        time.sleep(0.1)
                        currentcompass = compass_program()
                        subcompass = compasscurrentdestisub(currentcompass, desticompass)
                        rawsubcompass = rawcompasscurrentdestisub(currentcompass, desticompass)

                        print('補足衛星数              ', currentSattelite)
                        print('現在     緯度/経度      ', localat, localong)
                        print('目的地[%d] 緯度/経度    ' % (g), destilatlong[g], destilatlong[g + 1])
                        print('現在方角               ', currentcompass)
                        print('目的地の方角            ', desticompass)
                        print('(現在 - 目的地) 方角    ', subcompass)
                        print('残り(メートル)          ', x)

                        WRITE_LCD(0, 0)
                        DISPLAY_LCD('NOKORI-m')
                        WRITE_LCD(0, 1)
                        DISPLAY_LCD('{0}        '.format(x))

                        if jtalk_distance_sum > 4:
                            talk = '残り%sメートル' % (x)
                            print(talk)
                            voice_output(talk)
                            previous_x = x
                        if subcompass < 20:
                            # Go front
                            gpio_gofront()

                            distance = ultrasonic_distance()
                            if distance < 20:
                                voice_output('障害物')
                                print('障害物')
                                gpio_stop()
                                gpio_goback()
                                time.sleep(1)
                                gpio_stop()
                                servo('right')
                                distance_right = ultrasonic_distance()
                                print(distance_right, "(右)障害物の距離[cm]")
                                servo_return()
                                servo('left')
                                distance_left = ultrasonic_distance()
                                print(distance_left, "(左)障害物の距離[cm]")
                                servo_return()
                                if (distance_right > distance_left):
                                    gpio_spinright()
                                    time.sleep(0.6)
                                    gpio_gofront()
                                    time.sleep(2)
                                else:
                                    gpio_spinleft()
                                    time.sleep(0.6)
                                    gpio_gofront()
                                    time.sleep(2)
                        if subcompass > 20:
                            voice_output('方角修正中')
                            while True:
                                print('\n')
                                print('------------------------方角修正中---------------------------')
                                if rawsubcompass >= -180 and rawsubcompass <= 0:
                                    gpio_spinright()
                                if rawsubcompass < -180:
                                    gpio_spinleft()
                                if rawsubcompass >= 0 and rawsubcompass < 180:
                                    gpio_spinleft()
                                if rawsubcompass >= 180:
                                    gpio_spinright()

                                time.sleep(0.07)
                                gpio_stop()
                                time.sleep(0.08)

                                fix_currentcompasss = compass_program()
                                fix_compass_error = compasscurrentdestisub(fix_currentcompasss, desticompass)
                                rawsubcompass = rawcompasscurrentdestisub(fix_currentcompasss, desticompass)
                                print('(現在-目的地)方角の誤差    :', fix_compass_error)

                                WRITE_LCD(0, 0)
                                DISPLAY_LCD('Syusei  ')
                                WRITE_LCD(0, 1)
                                DISPLAY_LCD('{0}        '.format(fix_compass_error))

                                if fix_compass_error < 10:
                                    print('\n')
                                    print('---------------------方角修正終了-----------------------')
                                    break

                        if x < 0.6:
                            os.system('clear')
                            print('point reached')
                            a = 1
                            gpio_stop()
                            break

                    if a == 1:
                        print(g)
                        gpio_stop()
                        os.system('clear')
                        print('counter read')
                        break

        if g == loopnum:
            voice_output('目的地に到着しました')
            print('目的地に到着しました')
            WRITE_LCD(0, 0)
            DISPLAY_LCD('Touchaku')
            WRITE_LCD(0, 1)
            DISPLAY_LCD('(^_^)   ')
            GPIO.cleanup()
            break
except KeyboardInterrupt:
    GPIO.cleanup()


