# -*- coding: utf-8 -*-

# 라즈베리파이 GPIO 패키지
import RPi.GPIO as GPIO
import spidev
from time import sleep
import pygame
import sys, os
from pygame.locals import *

# 모터 상태
STOP  = 0
FORWARD  = 1
BACKWORD = 2

# 모터 채널
CH1 = 0
CH2 = 1
CH3 = 2

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
IN5 = 17  #


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

    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)

    #뒤로
    elif stat == BACKWORD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)

    #정지
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)

    elif stat == FORWARDRIGHT:
        GPIO.output(IN2, LOW)
        GPIO.output(IN1, HIGH)
        GPIO.output(IN3, HIGH)
        GPIO.output(IN4, LOW)

    elif stat == BACKRIGHT:
        GPIO.output(IN2, HIGH)
        GPIO.output(IN1, LOW)
        GPIO.output(IN3, HIGH)
        GPIO.output(IN4, LOW)

    elif stat == BACKLEFT:
        GPIO.output(IN2, HIGH)
        GPIO.output(IN1, LOW)
        GPIO.output(IN3, LOW)
        GPIO.output(IN4, HIGH)


# 모터 제어함수 간단하게 사용하기 위해 한번더 래핑(감쌈)
def setMotor(ch, speed, stat):
    if ch == CH1:
        #pwmA는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        #pwmB는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmB, IN3, IN4, speed, stat)

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)

spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000

def analog_read(channel):
        r = spi.xfer2([1,(8 + channel) <<  4, 0])
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

isAuto = 0
dist = 0

setMotor(CH2, 100, FORWARD)
sleep(0.1)
setMotor(CH2, 100, BACKWORD)
sleep(0.1)
setMotor(CH2, 80, STOP)

def ABS():
        for i in range(0,10):
                setMotor(CH1, 100, BACKWORD)
                sleep(0.05)
                setMotor(CH1, 100, STOP)
                sleep(0.01)

def auto_pilot(dist):
        print(dist)
        dist = analog_read(0)
        setMotor(CH1, 100, FORWARD)
        sleep(0.005)
        setMotor(CH1, 80, STOP)
        sleep(0.005)
        dist = analog_read(0)
        if dist < 1000 and dist > 950:
                ABS()
                setMotor(CH1, 100, BACKWORD)
                setMotor(CH2, 100, FORWARD)
                sleep(0.5)
                setMotor(CH2, 80, STOP)
                sleep(0.1)
                setMotor(CH1, 100, FORWARD)

#평행주차
def par_parking():
	dist0 = analog_read(0)
	dist1 = analog_read(6)
	dist2 = analog_read(4)

	setMotor(CH2, 80, BACKRIGHT)

	while dist1 < 950:
		setMotor(CH1, 80, BACKWARD)

	while dist2 < 950 and dist1 < 950 and dist0 < 950:
		if dist1 < 1000 and dist1 > 950:
			setMotor(CH2, 80, BACKLEFT)
		elif dist0 < 1000 and dist0 > 950:
			setMotor(CH1, 80, FORWARDRIGHT)

#주차공간탐색
def explore_space():
	print(dist)
	dist = analog_read(0)

	#빈공간이 아닐때까지 서행
	space = 0
	while dist < 1000 and dist > 950:
		setMotor(CH1, 50, FORWARD)
		space = space + 1
		dist = analog_read(0)

	setMotor(CH1, 80, STOP)
	print(space)
	
	if space < 100 and space > 80:
		print('직각주차')



while True:
        while isAuto:
                dist = analog_read(0)
                for event in pygame.event.get():
                    if event.type == QUIT:
                        GPIO.cleanup()
                        sys.exit()
                    if event.type == KEYDOWN:
                        if event.key == pygame.K_s:
                            isAuto = 0
                            break

                auto_pilot(dist)

        print(dist)
        for event in pygame.event.get():
                if event.type == QUIT:
                        GPIO.cleanup()
                        sys.exit()
                if event.type == KEYUP:
                        if event.key == pygame.K_a:
                                isAuto = 1
                        if event.key == pygame.K_e:
                                pygame.quit()
                                sys.exit()
                                #os.system('sudo shutdown -h now')

                        setMotor(CH1, 80, STOP)
                        setMotor(CH2, 80, STOP)

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
                setMotor(CH1, 100, FORWARD)
        if keys[pygame.K_DOWN]:
                setMotor(CH1, 100, BACKWORD)
        if keys[pygame.K_LEFT]:
                setMotor(CH2, 100, FORWARD)
        if keys[pygame.K_RIGHT]:
                setMotor(CH2, 100, BACKWORD)
        if keys[pygame.K_SPACE]:
                ABS()
	if keys[pygame.K_x]:
		explore_space()




