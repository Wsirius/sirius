# -*- coding: utf-8 -*-

# 라즈베리파이 GPIO 패키지 
import RPi.GPIO as GPIO
import spidev
from time import sleep
import pygame
import sys
from pygame.locals import *

# 모터 상태
STOP  = 0
FORWARD  = 1
BACKWORD = 2

# 모터 채널
CH1 = 0
CH2 = 1

# PIN 입출력 설정
OUTPUT = 1
INPUT = 0

# PIN 설정
HIGH = 1
LOW = 0

# 실제 핀 정의
#PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin

#GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin
IN5 = 17


# 핀 설정 함수
def setPinConfig(EN, INA, INB):        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    # 100khz 로 PWM 동작 시킴 
    pwm = GPIO.PWM(EN, 91000) 
    # 우선 PWM 멈춤.   
    pwm.start(0) 
    return pwm

# 모터 제어 함수
def setMotorContorl(pwm, INA, INB, speed, stat):

    #모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)  

    #앞으로 A=1, B=0
    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)

    #뒤로 A=0, B=1
    elif stat == BACKWORD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)

    #정지 A=0, B=0
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)


# 모터 제어함수 간단하게 사용하기 위해 한번더 래핑(감쌈)
def setMotor(ch, speed, stat):
    if ch == CH1:
        #pwmA는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        #37, 35 핀의 값
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        #pwmB는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmB, IN3, IN4, speed, stat)


# 라즈베리와 아두이노 연결 모듈 만들기
# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)

spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000

#라즈베리파이에서 아날로그 신호를 읽기 위한 함수
#ADC를 통해 변환된 아날로그 신호를 라즈베리파이에서 SPI 통신 프로토콜을 이용하여 읽기 위해서는 라즈베리파이에서 SPI를 사용할 수 있게 해줘야 한다.
def analog_read(channel):
        #안에 있는 데이터를 쓰고 리턴된 값을 반환한다.
	r = spi.xfer2([1,(8 + channel) <<  4, 0])
        #8비트 중 1, 2번째 비트는 ADC 데이터의 9, 10번째 비트, 세번째는 하위 8비트를 의미한다.
	adc_out = ((r[1]&3) << 8) + r[2]
	return adc_out


#모터 핀 설정
#핀 설정후 PWM 핸들 얻어옴
pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)

GPIO.setup(IN5, GPIO.IN)
#제어 시작

pygame.init()
screen = pygame.display.set_mode((640, 480))

pygame.display.set_caption('RaspiRobot')
pygame.mouse.set_visible(0)

dist = 0
state_on=0


def go(keys):
    if keys[pygame.K_UP]:
        setMotor(CH1, 100, FORWARD)
    if keys[pygame.K_DOWN]:
        setMotor(CH1, 100, BACKWORD)
    if keys[pygame.K_LEFT]:
        setMotor(CH2, 100, FORWARD)
    if keys[pygame.K_RIGHT]:
        setMotor(CH2, 100, BACKWORD)

def stop():
    setMotor(CH1, 80, STOP)
    setMotor(CH2, 80, STOP)


while True:
    for event in pygame.event.get():
        if event.type == QUIT:
        	GPIO.cleanup()
		sys.exit()
	if event.type == KEYUP:
                stop()
		reading = analog_read(0)
                #전압 값 계산, 몇 V인지
		voltage = reading * 3.3 / 1024
		#print("R = %d \t V = %f" %(reading, voltage))

    dist = analog_read(0)
    print(dist)
    print("\n")
    #핀의 값을 받아서 state_on 상태에 입력
    state_on = GPIO.input()

    keys = pygame.key.get_pressed()
    if state_on:
        stop()
    else:
        go()
    
