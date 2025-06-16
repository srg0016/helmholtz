from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow
from helmholtz_ui import Ui_MainWindow
import sys
import serial
import serial.tools.list_ports 
import time
import minimalmodbus
import ctypes
import numpy as np
from picosdk.ps5000a import ps5000a as ps
import matplotlib.pyplot as plt
from picosdk.functions import adc2mV, assert_pico_ok, mV2adc
import csv
import statistics
import scipy.integrate as it
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.backends.backend_qtagg import \
    NavigationToolbar2QT as NavigationToolbar
from matplotlib.backends.qt_compat import QtWidgets
from matplotlib.figure import Figure
from scipy import signal
import configparser
from datetime import date
import os

# ---------- Создать файл для измерений если не существует  ----------
today = date.today()
today_string = today.strftime("%b-%d-%Y")
if not os.path.exists(f'Data-{today_string}.csv'):
    with open(f'Data-{today_string}.csv', 'w', newline='') as csvfile:
        csvfile.write(f"Характеристики постоянных магнитов\n")
        csvfile.write(f"Дата измерений: {today_string}\n\n")

# ---------- Modbus-адрес драйвера шагового двигателя  ----------
STEPPER_MB_ADDRESS = 1

# ---------- Общие переменные ----------
global rotatingTime, cycle, bias, FILTER, mode, calCoef, magMoment, calIntegral, ID
rotatingTime = 0
maxSamples = 0
cycle = 0
bias = 0
FILTER = 0
mode = "Normal"
ID = 0

# ---------- Переменные шагового двигателя ----------
global motorSpeed, motorAcc, motorDec, motorState, motorTurns, servo, stepper
motorSpeed = 60 # rpm
motorAcc = 20
motorDec = 20
motorState = 0
motorTurns = 10

# ---------- Переменные Picoscope 5442D ----------
global sampleRates, timeBase, interval, channels, intervals, resolution
resolutions = ["14", "15", "16"]
pico_resolutions = ["PS5000A_DR_14BIT", "PS5000A_DR_15BIT", "PS5000A_DR_16BIT"]
pico_channels = ["PS5000A_CHANNEL_A", "PS5000A_CHANNEL_B", "PS5000A_CHANNEL_C", "PS5000A_CHANNEL_D"]
pico_ranges = ["PS5000A_10MV", "PS5000A_20MV", "PS5000A_50MV", "PS5000A_100MV", "PS5000A_200MV", "PS5000A_500MV", "PS5000A_1V", "PS5000A_2V", "PS5000A_5V", "PS5000A_10V", "PS5000A_20V", "PS5000A_50V"]
pico_status_setchannel = ["setChA", "setChB", "setChC", "setChD"]
resolution = 14
ranges = ["10 mV", "20 mV", "50 mV", "100 mV", "200 mV", "500 mV", "1 V", "2 V", "5 V", "10 V", "20 V", "50 V"]
channels = [0, 0, 0, 0]
intervals_14bit_15bit = ["104", "200", "504", "1000", "2000"]
intervals_16bit = ["112", "208", "512", "1008", "2000"]
sampleRates_14bit_15bit = ["9.62 МС/c", "5 МС/c", "1.98 МС/c", "1 МС/c", "500 кС/c"]
sampleRates_16bit = ["8.93 МС/c", "4.81 МС/c", "1.95 МС/c", "992 кС/c", "500 кС/c"]
motorTypes = ["Stepper", "BLDC"]

# ---------- Чтение конфигурационного файла ----------
config = configparser.ConfigParser()
config.optionxform = str
config.read("helmholtz.conf")
calIntegral = float(config['Calibration']['CalibrationIntegral'])
magMoment = float(config['Calibration']['CalibrationMoment'])
calCoef = float(config['Calibration']['CalibrationMomentCoef'])

# ---------- Описание функций ----------

# ---------- Запуск бесконечного вращения шагового двигателя ----------
def rotate():
	global motorSpeed, motorAcc, motorDec, motorState, motorTurns, servo, stepper
	motorSpeed = int(application.ui.Speed.text())
	motorAcc = int(application.ui.Acceleration.text())
	motorDec = int(application.ui.Deceleration.text())
	motorTurns = int(application.ui.Turns.text())
	motorCurrent = int(application.ui.Current.value()*10)
	stepper.write_register(0x1801, 0x1111, functioncode=6)
	stepper.write_register(0x6201, 10000, functioncode=6) # Set PR0 position high
	stepper.write_register(0x6202, 10000, functioncode=6) # Set PR0 position low
	stepper.write_register(0x6203, motorSpeed, functioncode=6) # Set PR0 velocity
	stepper.write_register(0x6204, motorAcc, functioncode=6) # Set PR0 acc
	stepper.write_register(0x6205, motorDec, functioncode=6) # Set PR0 dec
	stepper.write_register(0x0191, motorCurrent, functioncode=6)
	stepper.write_register(0x0193, 50, functioncode=6)
	stepper.write_register(0x0195, 50, functioncode=6)
	stepper.write_register(0x6002, 0x10, functioncode=6) # START
	print("stepper rotate")

# ---------- Запуск вращения шагового двигателя на заданное количество оборотов ----------
def start():
	global motorSpeed, motorAcc, motorDec, motorState, motorTurns, servo, stepper
	motorSpeed = int(application.ui.Speed.text())
	motorAcc = int(application.ui.Acceleration.text())
	motorDec = int(application.ui.Deceleration.text())
	motorTurns = int(application.ui.Turns.text())
	motorCurrent = int(application.ui.Current.value()*10)
	stepper.write_register(0x6203, motorSpeed, functioncode=6) # Set PR0 velocity
	stepper.write_register(0x6204, motorAcc, functioncode=6) # Set PR0 acc
	stepper.write_register(0x6205, motorDec, functioncode=6) # Set PR0 dec
	stepper.write_register(0x6201, ((10000*motorTurns)>>16)&0xFFFF, functioncode=6) # Set PR0 position high
	stepper.write_register(0x6202, (10000*motorTurns)&0xFFFF, functioncode=6) # Set PR0 position low
	stepper.write_register(0x0191, motorCurrent, functioncode=6)
	stepper.write_register(0x0193, 50, functioncode=6)
	stepper.write_register(0x0195, 50, functioncode=6)
	stepper.write_register(0x6002, 0x10, functioncode=6)
	print("start")

# ---------- Остановка шагового двигателя ----------
def stop():
	global servo, stepper
	stepper.write_register(0x6002, 0x40, functioncode=6)
	print("stop")

# ---------- Снятие тока удержания двигателя ----------
def releaseShaft():
	global servo, stepper
	stepper.write_register(0x6002, 0x40, functioncode=6)
	time.sleep(0.5)
	stepper.write_register(0x0193, 0x00, functioncode=6)
	print("shaft released")


# ---------- Вывод индекса типа мотора ----------
def motorType():
	print(application.ui.MotorType.currentIndex())

# ---------- Инициализация драйвера шагового двигателя ----------
def stepper_init():
	global stepper, motorSpeed, motorAcc, motorDec, motorTurns
	try:
		print(motorSpeed, motorAcc, motorDec, motorTurns)
		stepper = minimalmodbus.Instrument(application.ui.SerialPort.currentText(), STEPPER_MB_ADDRESS)
		stepper.serial.baudrate = 115200
		stepper.serial.parity = serial.PARITY_NONE
		stepper.serial.stopbits = 1
		motorCurrent = int(application.ui.Current.value()*10)
		stepper.write_register(0x0001, 10000, functioncode=6) # 10000 Pulses per revolution
		stepper.write_register(0x0003, 2, functioncode=6) # Closed loop
		stepper.write_register(0x0007, 1, functioncode=6) # CCW
		stepper.write_register(0x6200, 65, functioncode=6) # Set PR mode to relative position mode
		stepper.write_register(0x6201, ((10000*motorTurns)>>16)&0xFFFF, functioncode=6) # Set PR0 position high
		stepper.write_register(0x6202, (10000*motorTurns)&0xFFFF, functioncode=6) # Set PR0 position low
		stepper.write_register(0x6203, 60, functioncode=6) # Set PR0 velocity
		stepper.write_register(0x6204, 50, functioncode=6) # Set PR0 acc
		stepper.write_register(0x6205, 50, functioncode=6) # Set PR0 dec
		stepper.write_register(0x0191, motorCurrent, functioncode=6) # peak current, X*0.1A (step)
		application.ui.ServoStatus.setText("Подключен")
		print("stepper enabled")
	except:
		application.ui.ServoStatus.setText("Не подключен")
		print("stepper not found")

def stepper_run():
	global stepper, motorSpeed, motorAcc, motorDec, motorTurns
	try:
		stepper.write_register(0x0001, 10000, functioncode=6) # 10000 Pulses per revolution
		stepper.write_register(0x0003, 2, functioncode=6) # Closed loop
		stepper.write_register(0x0007, 1, functioncode=6) # CCW
		stepper.write_register(0x6200, 65, functioncode=6) # Set PR mode to relative position mode
		stepper.write_register(0x6201, 0, functioncode=6) # Set PR0 position high
		stepper.write_register(0x6202, 10000*motorTurns, functioncode=6) # Set PR0 position low
		stepper.write_register(0x6203, 10, functioncode=6) # Set PR0 velocity
		stepper.write_register(0x6204, 50, functioncode=6) # Set PR0 acc
		stepper.write_register(0x6205, 50, functioncode=6) # Set PR0 dec
		#application.ui.Current.value()
		#stepper.write_register(0x6002, 0x10, functioncode=6) # START
		#stepper.write_register(0x6002, 0x40, functioncode=6) # STOP
		application.ui.ServoStatus.setText("Подключен")
		print("stepper enabled")
	except:
		application.ui.ServoStatus.setText("Не подключен")
		print("stepper not found")

def portChanged():
	global port, servo
	if application.ui.SerialPort.currentIndex() != -1:
		servo.close()

def calcTime():
	global rotatingTime, motorSpeed, motorTurns
	motorSpeed = int(application.ui.Speed.text())
	motorTurns = int(application.ui.Turns.text())
	rotatingTime = motorTurns/motorSpeed*60
	application.ui.Time.setText(str(rotatingTime))
	print("Rotating time: ", rotatingTime)

def updateInterval():
	global timeBase, interval, resolution
	resolution = 0
	if application.ui.Resolution.count() != 0:
		resolution = int(application.ui.Resolution.currentText())
		if resolution in [14, 15]:
			application.ui.Interval.clear()
			application.ui.Interval.addItems(intervals_14bit_15bit)
		elif resolution == 16:
			application.ui.Interval.clear()
			application.ui.Interval.addItems(intervals_16bit)
			application.ui.Interval.setCurrentIndex(2) 
		interval = int(application.ui.Interval.currentText())
		print(resolution, interval)

def resolutionUpdate():
	global timeBase, interval, resolution, channels
	channels[0] = application.ui.Channel1Enable.checkState()
	channels[1] = application.ui.Channel2Enable.checkState()
	channels[2] = application.ui.Channel3Enable.checkState()
	channels[3] = application.ui.Channel4Enable.checkState()
	if channels.count(2) >= 3:
		application.ui.Resolution.clear()
		application.ui.Resolution.addItems(['14'])
	if channels.count(2) == 2:
		application.ui.Resolution.clear()
		application.ui.Resolution.addItems(['14', '15'])
	if channels.count(2) == 1:
		application.ui.Resolution.clear()
		application.ui.Resolution.addItems(['14', '15', '16 '])
	print(channels)

def calcTimeBase():
	global timeBase, interval, resolution, rotatingTime, maxSamples
	if resolution in [14, 15]:
		application.ui.SampleRate.setText(sampleRates_14bit_15bit[application.ui.Interval.currentIndex()])
		maxSamples = int(rotatingTime/int(intervals_14bit_15bit[application.ui.Interval.currentIndex()])*pow(10,9)) + 100
		print(maxSamples)
		application.ui.MaxSamples.setText(f"{maxSamples}")
	elif resolution == 16:
		application.ui.SampleRate.setText(sampleRates_16bit[application.ui.Interval.currentIndex()])
		maxSamples = int(rotatingTime/int(intervals_16bit[application.ui.Interval.currentIndex()])*pow(10,9)) + 100
		print(maxSamples)
		application.ui.MaxSamples.setText(f"{maxSamples}")

def startADC():
	global cycle, resolution, channels, stepper, startTime
	application.ui.statusBar.setText("Сбор данных")
	application.ui.statusBar.repaint()
	#resolutionUpdate()
	chRangeA = 0
	chRangeB = 0
	chandle = ctypes.c_int16()
	status = {}
	startTime = time.time()
	
	# Set resolution
	# Установка разрешения
	res =ps.PS5000A_DEVICE_RESOLUTION[pico_resolutions[application.ui.Resolution.currentIndex()]]
	
	# Open 5000 series PicoScope
	# Запуск осциллографа
	status["openunit"] = ps.ps5000aOpenUnit(ctypes.byref(chandle), None, res)
	try:
		assert_pico_ok(status["openunit"])
	except: # PicoNotOkError:
		powerStatus = status["openunit"]
		if powerStatus == 286:
			status["changePowerSource"] = ps.ps5000aChangePowerSource(chandle, powerStatus)
		elif powerStatus == 282:
			status["changePowerSource"] = ps.ps5000aChangePowerSource(chandle, powerStatus)
		else:
			raise
		assert_pico_ok(status["changePowerSource"])

	#  Setup analog channels:
	#  Настройка аналоговых каналов
	chRange = [0,0,0,0]
	for i in channels:
		if channels[i] != 0:
			channel = ps.PS5000A_CHANNEL[pico_channels[i]]
			#coupling_type = ps.PS5000A_COUPLING["PS5000A_DC"]
			if i == 0:
				#chRange[i] = ps.PS5000A_RANGE[pico_ranges[application.ui.Channel1Range.currentIndex()]]
				chRange[i] = ps.PS5000A_RANGE["PS5000A_10V"]
			if i == 1:
				#chRange = ps.PS5000A_RANGE[pico_ranges[application.ui.Channel2Range.currentIndex()]]
				chRange[i] = ps.PS5000A_RANGE["PS5000A_10V"]
			if i == 2:
				#chRange = ps.PS5000A_RANGE[pico_ranges[application.ui.Channel3Range.currentIndex()]]
				chRange[i] = ps.PS5000A_RANGE["PS5000A_10V"]
			if i == 3:
				#chRange = ps.PS5000A_RANGE[pico_ranges[application.ui.Channel4Range.currentIndex()]]
				chRange[i] = ps.PS5000A_RANGE["PS5000A_10V"]
			coupling_type = ps.PS5000A_COUPLING["PS5000A_AC"]
			status[pico_status_setchannel[i]] = ps.ps5000aSetChannel(chandle, channel, 1, coupling_type, chRange[i], 0)
			chRangeA = chRange[0]
			chRangeB = chRange[1]
			
			assert_pico_ok(status[pico_status_setchannel[i]])

	# Set up digital channel
	# Настройка цифровых каналов
	channel = ps.PS5000A_CHANNEL["PS5000A_DIGITAL_PORT0"]
	status["setChDP0"] = ps.ps5000aSetDigitalPort(chandle, channel, 1, 20000)
	assert_pico_ok(status["setChDP0"])
	
	# find maximum ADC count value
	# Расчет максимального количества сэмплов АЦП
	maxADC = ctypes.c_int16()
	status["maximumValue"] = ps.ps5000aMaximumValue(chandle, ctypes.byref(maxADC))
	assert_pico_ok(status["maximumValue"])

	# Set number of pre and post trigger samples to be collected
	# Настройка количества сэмплов для считывания до и после триггера
	preTriggerSamples = 0
	postTriggerSamples = int(application.ui.MaxSamples.text())
	maxSamples = preTriggerSamples + postTriggerSamples
	print(maxSamples)

	# Set timebase:
	if resolution in [14, 15]:
		sample_interval = int(application.ui.Interval.currentText())
		timebase = 2 + int(0.125*sample_interval)
		print(sample_interval, timebase)
	elif resolution == 16:
		sample_interval = int(application.ui.Interval.currentText())
		timebase = 3 + int(0.0625*sample_interval)
		print(sample_interval, timebase)
	timeIntervalns = ctypes.c_float()
	returnedMaxSamples = ctypes.c_int32()
	status["getTimebase2"] = ps.ps5000aGetTimebase2(chandle, timebase, maxSamples, ctypes.byref(timeIntervalns), ctypes.byref(returnedMaxSamples), 0)
	assert_pico_ok(status["getTimebase2"])

	# Set trigger
	# Установка триггера
	source = ps.PS5000A_CHANNEL["PS5000A_CHANNEL_A"]
	direction = ps.PS5000A_THRESHOLD_DIRECTION["PS5000A_ABOVE"]
	status["setTrigger"] = ps.ps5000aSetSimpleTrigger(chandle, 1, source, 1, direction, 0, 1)
	assert_pico_ok(status["setTrigger"])
	# Run block capture
	t0 = time.time()
	status["runBlock"] = ps.ps5000aRunBlock(chandle, preTriggerSamples, postTriggerSamples, timebase, None, 0, None, None)
	assert_pico_ok(status["runBlock"])

	# Запуск вращения шагового двигателя
	start()

	# Check for data collection to finish using ps5000aIsReady
	# Проверка готовности данных
	ready = ctypes.c_int16(0)
	check = ctypes.c_int16(0)
	while ready.value == check.value:
		status["isReady"] = ps.ps5000aIsReady(chandle, ctypes.byref(ready))
	
	# Create buffers ready for assigning pointers for data collection
	bufferAMax = (ctypes.c_int16 * maxSamples)()
	bufferAMin = (ctypes.c_int16 * maxSamples)() # used for downsampling which isn't in the scope of this example
	bufferDigital = (ctypes.c_int16 * maxSamples)()
	# Set data buffer location for data collection from channel A
	source = ps.PS5000A_CHANNEL["PS5000A_CHANNEL_A"]
	status["setDataBuffersA"] = ps.ps5000aSetDataBuffers(chandle, source, ctypes.byref(bufferAMax), ctypes.byref(bufferAMin), maxSamples, 0, 0)
	assert_pico_ok(status["setDataBuffersA"])
	# Set data buffer location for data collection from digital channels
	source = ps.PS5000A_CHANNEL["PS5000A_DIGITAL_PORT0"]
	status["setDataBuffersDigital"] = ps.ps5000aSetDataBuffers(chandle, source, ctypes.byref(bufferDigital), ctypes.byref(bufferDigital), maxSamples, 0, 0)
	assert_pico_ok(status["setDataBuffersDigital"])
	# create overflow loaction
	overflow = ctypes.c_int16()
	# create converted type maxSamples
	cmaxSamples = ctypes.c_int32(maxSamples)
	# Retried data from scope to buffers assigned above
	status["getValues"] = ps.ps5000aGetValues(chandle, 0, ctypes.byref(cmaxSamples), 0, 0, 0, ctypes.byref(overflow))
	assert_pico_ok(status["getValues"])
	# convert ADC counts data to mV
	#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	adc2mVChAMax =  adc2mV(bufferAMax, chRangeA, maxADC) #!!!!!!!chRange!!!!!!!!!!!!!!!!!
	l = [adc2mVChAMax, list(bufferDigital)]
	#print(l)
	lt = zip(*l)
	#print("cycle = ", cycle)
	with open(f'raw_data_{cycle}.csv', 'w',newline='') as f:
		write = csv.writer(f)
		write.writerows(lt)
	CH1 = [float(a) for a in adc2mVChAMax]
	D = [int(d) for d in list(bufferDigital)]
	A = [((d >> 0) & 1) for d in D]
	B = [((d >> 1) & 1) for d in D]
	Z = [((d >> 2) & 1) for d in D]
	t1 = time.time() - t0
	print("Data receive time = ", t1)
	
	"""
	# Speed measurement
	a_prev = 0
	period = 0
	Period = []
	for i in range(len(A)):
		period += 1
		if A[i] != a_prev:
			a_prev = A[i]
			if A[i] == 1:
				Period.append(period)
				period = 0
	print("Period = ", Period)
	"""
	"""
	tick = [x for x in range(len(CH1))]
	application.ax1.clear()
	application.ax2.clear()
	application.ax1.plot(tick, CH1)
	application.ax2.plot(tick, Z)
	application.ax1.grid()
	application.ax2.grid()
	application.canvas.draw()"""
	try:
		status["closeunit"] = ps.ps5000aCloseUnit(chandle)
		assert_pico_ok(status["closeunit"])
	except:
		print("error close pico")
	application.ui.statusBar.setText("Готово")

	return CH1, D

def selectFile():
	print("select file")

def measure():
	global cycle, a, b, c, Mx, My, Mz, M, phi, theta, bias, calCoef, magMoment, calIntegral, startTime, ID
	# a - Mxy, проекция полного момента на плоскость XY
	# b - Mxz, проекция полного момента на плоскость XZ
	# c - Myz, проекция полного момента на плоскость YZ
	# Mx - проекция полного момента М на ось Х
	# My - проекция полного момента М на ось Y
	# Mz - проекция полного момента М на ось Z
	# M -  полный магнитный момент
	# phi - азимутальный угол
	# theta - полярный угол

	CH1, D = startADC()

	# Фильтрация входного сигнала, фильтр Баттерворта 2 порядка, частота 50 Гц
	if application.ui.calculationMethod.currentText() == 'Интегрирование':
		sos = signal.butter(1, 10, 'lp', fs=1000000, output='sos')
		if FILTER == 0:
			CH1 = signal.sosfilt(sos, CH1)
		#sos = signal.butter(2, 5, 'hp', fs=1000000, output='sos')
		#CH1 = signal.sosfilt(sos, CH1)

	# Разделяем A, B и Z каналы энкодера
	A = [((d >> 0) & 1) for d in D]
	B = [((d >> 1) & 1) for d in D]
	Z = [((d >> 2) & 1) for d in D]

	# Строим графики
	tick = [x for x in range(len(CH1))]
	if cycle != -1:
		application.ax1.clear()
		application.ax2.clear()
		application.ax1.plot(tick, CH1)
		application.ax2.plot(tick, Z)
		application.ax1.grid()
		application.ax2.grid()
		application.canvasMeasure.draw()
	else:	
		application.ax3.clear()
		application.ax4.clear()
		application.ax3.plot(tick, CH1)
		application.ax4.plot(tick, Z)
		application.ax3.grid()
		application.ax4.grid()
		application.canvasCalibration.draw()
	calcTime = time.time()
	# -----------------------------
	
	if application.ui.calculationMethod.currentText() == 'Интегрирование':
		z_prevLevel = 0
		z_index_list = []
		for i in range(len(D)):
			if (((D[i] >> 2) & 0x01) == 1) and (z_prevLevel == 0):
				z_index_list.append(i)
			z_prevLevel = (D[i] >> 2) & 0x01
		print(z_index_list)
		# -----------------------------
		A_indexes = []
		for turn in range(len(z_index_list)-1):
			A_turn  = A[z_index_list[turn]:z_index_list[turn+1]]
			a_prevLevel = 0
			a_index_list = []
			for i in range(len(A_turn)):
				if (A_turn[i] == 1) and (a_prevLevel == 0):
					a_index_list.append(i)
				a_prevLevel = A_turn[i]
			A_indexes.append(a_index_list)
		print(len(A_indexes))
		# -----------------------------
		V_phi = []
		mean = []
		for turn in range(len(A_indexes)):
			V = [CH1[z_index_list[turn]+i] for i in A_indexes[turn]]
			V_phi.append(V)
			mean.append(statistics.mean(V_phi[turn]))
		print(mean)
		# -----------------------------
		for turn in range(len(V_phi)):
			V_phi[turn] = [(item - mean[turn]) for item in V_phi[turn]]
			V_phi[turn] = V_phi[turn][:57600]
			print(len(V_phi[turn]))
		# -----------------------------
		if FILTER == 0:
			V_phi_abs = np.absolute(V_phi)
		elif FILTER == 1:
			V_phi_abs = V_phi
		# -----------------------------
		F = []
		if FILTER == 0:
			F = np.trapz(V_phi_abs, x=None, dx=1.0, axis=-1)
		elif FILTER == 1:
			for i in range(len(V_phi_abs)):
				F.append(np.absolute(np.trapz(V_phi_abs[i][:28800])) + np.absolute(np.trapz(V_phi_abs[i][28801:57600])))
	elif application.ui.calculationMethod.currentText() == 'Lock-in':
		counter = 0
		a_prevLevel = 0
		V = []
		SIN = []
		COS = []
		for i in range(len(A)):
			if (A[i] == 1) and (a_prevLevel == 0):
				counter += 1
				V.append(CH1[i])
				SIN.append(np.sin(2*np.pi*counter/57600))
				COS.append(np.cos(2*np.pi*counter/57600))
			a_prevLevel = A[i]
		f_sin = np.multiply(SIN, V)
		f_cos = np.multiply(COS, V)
		voltage_amp = np.power(statistics.mean(f_sin)**2 + statistics.mean(f_sin)**2, 0.5)

	if cycle == -1:
		if application.ui.calculationMethod.currentText() == 'Интегрирование':
			calIntegral = statistics.mean(F)
		elif application.ui.calculationMethod.currentText() == 'Lock-in':
			calIntegral = 0
		application.ui.CalibIntegral.setText(f"{calIntegral:.2f}")
		magMoment = float(application.ui.MagMoment.text())
		calCoef = float(magMoment/calIntegral)
		application.ui.CalibCoef.setText(f"{calCoef:.12f}")
		cycle = 0
		return
	elif cycle == 0:
		if application.ui.calculationMethod.currentText() == 'Интегрирование':
			print(F)
			a = (statistics.mean(F)-bias)*calCoef
		elif application.ui.calculationMethod.currentText() == 'Lock-in':
			a = voltage_amp*calCoef*57600*3.88/3.10
		application.ui.XY.setText(f"{a:.5f}")
		print("Cycle time = ", time.time() - startTime)
		print("Calculation time = ", time.time() - calcTime)
	elif cycle == 1:
		if application.ui.calculationMethod.currentText() == 'Интегрирование':
			b = (statistics.mean(F)-bias)*calCoef
		elif application.ui.calculationMethod.currentText() == 'Lock-in':
			b = voltage_amp*calCoef*57600*3.88/3.10
		application.ui.XZ.setText(f"{b:.5f}")
	elif cycle == 2:
		if application.ui.calculationMethod.currentText() == 'Интегрирование':
			c = (statistics.mean(F)-bias)*calCoef
		elif application.ui.calculationMethod.currentText() == 'Lock-in':
			c = voltage_amp*calCoef*57600*3.88/3.10
		application.ui.YZ.setText(f"{c:.5f}")
		M = (0.5*(a*a + b*b + c*c))**0.5
		application.ui.M.setText(f"{M:.5f}")
		print(a, b, c)
		Mx = np.power(np.absolute(0.5*(a*a + b*b - c*c)), 0.5)
		My = np.power(np.absolute(0.5*(a*a - b*b + c*c)), 0.5)
		Mz = np.power(np.absolute(0.5*(-a*a + b*b + c*c)), 0.5)
		print(Mx, My, Mz)
		phi = np.rad2deg(np.arctan(My/Mx))
		application.ui.Phi.setText(f"{phi:.5f}")
		theta = np.rad2deg(np.arctan(np.power(Mx*Mx + My*My, 0.5)/Mz))
		application.ui.Theta.setText(f"{theta:.5f}")

		# Сохранить в файл
		with open(f'Data-{today_string}.csv','a', newline='') as csvfile:
			writer = csv.writer(csvfile)
			fields = [application.ui.ID.text(), \
					application.ui.Series.text(), \
					application.ui.Mass.text(), \
					application.ui.Volume.text(), \
					application.ui.Temperature.text(), \
					application.ui.XY.text(), \
					application.ui.XZ.text(), \
					application.ui.YZ.text(), \
					application.ui.M.text(), \
					application.ui.Phi.text(), \
					application.ui.Theta.text()]
			writer.writerow(fields)

def plotData(data):
	Achannel, Dchannel = np.loadtxt("read-1.csv", delimiter=",", skiprows=0, dtype=str, unpack=True)
	CH1 = [float(a) for a in Achannel]
	D = [int(d) for d in Dchannel]
	A = [((d >> 0) & 1) for d in D]
	B = [((d >> 1) & 1) for d in D]
	Z = [((d >> 2) & 1) for d in D]
	tick = [x for x in range(len(CH1))]
	application.ax1.plot(tick, CH1)
	application.ax2.plot(tick, Z)
	application.ax1.grid()
	application.ax2.grid()
	application.canvas.draw()

def measureCycles():
	global cycle
	measureFlag = 1
	while (measureFlag):
		application.ui.statusBar.setText(f"Вращение цикла {cycle+1}")
		application.ui.statusBar.repaint()
		#startADC()
		measure()
		application.ui.statusBar.setText(f"Вращение {cycle+1} завершено")
		dlg = QMessageBox()
		dlg.setText(f"Установите магнит в положение 2, затем нажмите ОК, чтобы перемерить - Retry, для выхода из измерения - Abort")
		dlg.setStandardButtons(QMessageBox.Ok | QMessageBox.Retry | QMessageBox.Abort)
		button = dlg.exec()
		if button == QMessageBox.Ok:
			break
		elif button ==  QMessageBox.Retry:
			pass
		elif button == QMessageBox.Abort:
			measureFlag = 0
	while (measureFlag):
		cycle = 1
		application.ui.statusBar.setText(f"Вращение цикла {cycle+1}")
		application.ui.statusBar.repaint()
		#startADC()
		measure()
		application.ui.statusBar.setText(f"Вращение {cycle+1} завершено")
		dlg = QMessageBox()
		dlg.setText(f"Установите магнит в положение 3, затем нажмите ОК, чтобы перемерить Retry, для выхода из измерения - Abort")
		dlg.setStandardButtons(QMessageBox.Ok | QMessageBox.Retry | QMessageBox.Abort)
		button = dlg.exec()
		if button == QMessageBox.Ok:
			break
		elif button ==  QMessageBox.Retry:
			pass
		elif button == QMessageBox.Abort:
			measureFlag = 0
	while (measureFlag):
		cycle = 2
		application.ui.statusBar.setText(f"Вращение цикла {cycle+1}")
		application.ui.statusBar.repaint()
		#startADC()
		measure()
		application.ui.statusBar.setText(f"Вращение {cycle+1} завершено")
		application.ui.statusBar.repaint()
		cycle = 0
		dlg = QMessageBox()
		dlg.setText("Для завершения нажмите ОК, чтобы перемерить Retry, для выхода из измерения - Abort")
		dlg.setStandardButtons(QMessageBox.Ok | QMessageBox.Retry | QMessageBox.Abort)
		button = dlg.exec()
		if button == QMessageBox.Ok:
			break
		elif button ==  QMessageBox.Retry:
			pass
		elif button == QMessageBox.Abort:
			measureFlag = 0
	application.ui.statusBar.setText("Готово")
	application.ui.statusBar.repaint()

def measureCalibration():
	global cycle, mode
	mode = 'Calibration'
	cycle = -1
	print("measureCalibration")
	measure()

def saveCalibration():
	global calCoef, magMoment, calIntegral
	magMoment = float(application.ui.MagMoment.text())
	calIntegral = float(application.ui.CalibIntegral.text())
	calCoef = magMoment/calIntegral
	application.ui.CalibCoef.setText(f"{calCoef:.10f}")
	config['Calibration']['CalibrationIntegral'] = str(calIntegral)
	config['Calibration']['CalibrationMoment'] = str(magMoment)
	config['Calibration']['CalibrationMomentCoef'] = f"{calCoef:.10f}"
	with open('helmholtz.conf', 'w') as configfile:
		config.write(configfile)
	print("saveCalibration")

def filterChanged():
	global FILTER
	#FILTER = application.ui.Filter.currentIndex()
	#print(application.ui.Filter.currentIndex())

def save_and_exit():
	dlg = QMessageBox()
	#dlg.setWindowTitle("I have a question!")
	dlg.setText("Save and exit")
	dlg.setStandardButtons(QMessageBox.Ok | QMessageBox.Retry | QMessageBox.Abort)
	button = dlg.exec()
	if button == QMessageBox.Ok:
		print("Yes!")
	elif button ==  QMessageBox.Retry:
		print("No")
	elif button == QMessageBox.Abort:
		print("Nsd")

def incrementID():
	global ID
	ID += 1
	application.ui.ID.setText(str(ID))
	print("Magnet ID = ", ID)

# ---------- Serial ----------
global port
portList = serial.tools.list_ports.comports(include_links=False)
comPorts = []
for item in portList:
	comPorts.append(item.device)
print("Available COM ports: " + str(comPorts))
#port = serial.Serial(str(comPorts[0]))

class window(QtWidgets.QMainWindow):
	def __init__(self):
		super(window, self).__init__()
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		self.setFocus(True)

		for i in range(len(comPorts)):
			self.ui.SerialPort.addItem(str(comPorts[i]))
		
		# Init Pico parameters
		#for resolution in resolutions:
		self.ui.Resolution.addItems(resolutions)
		#for rng in ranges:
		self.ui.Channel1Range.addItems(ranges)
		#for rng in ranges:
		self.ui.Channel2Range.addItems(ranges)
		#for rng in ranges:
		self.ui.Channel3Range.addItems(ranges)
		#for rng in ranges:
		self.ui.Channel4Range.addItems(ranges)
		self.ui.Interval.addItems(intervals_14bit_15bit)
		self.ui.SampleRate.setText(sampleRates_14bit_15bit[0])
		self.ui.MotorType.addItems(motorTypes)

		#self.ui.SerialPort.currentIndexChanged.connect(portChanged)
		self.ui.ContRotation.clicked.connect(rotate)
		self.ui.StartRotation.clicked.connect(start)
		self.ui.StopRotation.clicked.connect(stop)
		self.ui.ReleaseShaft.clicked.connect(releaseShaft)
		self.ui.Connect.clicked.connect(stepper_init)
		#self.ui.StartRecord.clicked.connect(startRecord)
		self.ui.startButton.clicked.connect(measureCycles)
		self.ui.nextButton.clicked.connect(incrementID)
		self.ui.restartButton.clicked.connect(plotData)
		self.ui.saveButton.clicked.connect(save_and_exit)
		
		# Calculate rotation time
		self.ui.Speed.editingFinished.connect(calcTime)
		self.ui.Turns.editingFinished.connect(calcTime)

		# Calculate Picoscope
		self.ui.Resolution.setCurrentIndex(2) # Set 16 bit default
		self.ui.Resolution.currentIndexChanged.connect(updateInterval)
		self.ui.Interval.currentIndexChanged.connect(calcTimeBase)

		# Checkbox changed
		self.ui.Channel1Enable.setChecked(True)
		self.ui.Channel1Enable.stateChanged.connect(resolutionUpdate)
		self.ui.Channel2Enable.stateChanged.connect(resolutionUpdate)
		self.ui.Channel3Enable.stateChanged.connect(resolutionUpdate)
		self.ui.Channel4Enable.stateChanged.connect(resolutionUpdate)
		global channels
		channels = [1, 0, 0, 0]
		# Motor type changed
		self.ui.MotorType.currentIndexChanged.connect(motorType)

		# Select file
		self.ui.SelectFile.clicked.connect(selectFile)
		self.ui.StopRecord.clicked.connect(self.button_clicked)
		self.ui.MeasureCalibration.clicked.connect(measureCalibration)
		self.ui.SaveCalibration.clicked.connect(saveCalibration)

		# Plot
		#plt.tight_layout()
		#plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=None)
		self.figMeausre = Figure(figsize=(5, 4))
		self.canvasMeasure = FigureCanvas(self.figMeausre)
		self.ui.verticalLayout.addWidget(self.canvasMeasure)
		(self.ax1, self.ax2) = self.canvasMeasure.figure.subplots(2,1)
		self.canvasMeasure.figure.set_tight_layout(True)
		self.ax1.set(xlabel='samples')
		self.ax1.set_title('Coil voltage, mV',loc='left')
		self.ax2.set(xlabel='samples')
		self.ax2.set_title('Z-pulses',loc='left')

		# Calibration plot
		self.figCalibration = Figure(figsize=(5, 4))
		self.canvasCalibration = FigureCanvas(self.figCalibration)
		self.ui.verticalLayout.addWidget(self.canvasCalibration)
		self.ui.verticalLayoutCalibration.addWidget(self.canvasCalibration)
		(self.ax3, self.ax4) = self.canvasCalibration.figure.subplots(2,1)
		self.ax3.set(xlabel='samples')
		self.ax3.set_title('Coil voltage, mV',loc='left')
		self.ax4.set(xlabel='samples')
		self.ax4.set_title('Z-pulses',loc='left')
		self.canvasCalibration.figure.set_tight_layout(True)
		#self.ui.Filter.currentIndexChanged.connect(filterChanged)

	def button_clicked(self, s):
		print("click", s)
		dlg = CustomDialog(1)
		if dlg.exec():
			print("Success!")
		else:
			print("Cancel!")

	def show_dialog(self):
		# Create a QDialog instance
		dialog = QDialog(self)
		dialog.setWindowTitle("Dialog Box")
		# Create a label with a message
		label = QLabel("This is a message in the dialog box.")
		# Create a layout for the dialog
		dialog_layout = QVBoxLayout()
		dialog_layout.addWidget(label)
		# Set the layout for the dialog
		dialog.setLayout(dialog_layout)
		# Show the dialog as a modal dialog (blocks the main window)
		dialog.exec_()

class CustomDialog(QDialog):
	def __init__(self, cycle):
		super().__init__()

		self.setWindowTitle("Измерение завершено")

		#QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel
		
		#retryBtn = QPushButton(text="Remeasure",parent=self)
		QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel
		
		self.buttonBox = QDialogButtonBox(QBtn)
		#self.buttonBox.addButton(retryBtn)
		#self.buttonBox.addButton(self, "Again", ButtonRole.AcceptRole)
		self.buttonBox.accepted.connect(self.accept)
		self.buttonBox.rejected.connect(self.reject)
		self.buttonBox.setCenterButtons(True)

		self.layout = QVBoxLayout()
		if cycle != 4:
			message = QLabel(f"Установите магнит в положение {cycle}, затем нажмите ОК, чтобы перемерить Cancel")
		else:
			message = QLabel(f"Для завершения нажмите ОК, чтобы перемерить Cancel")
		self.layout.addWidget(message)
		self.layout.addWidget(self.buttonBox)
		self.setLayout(self.layout)

class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

app = QtWidgets.QApplication([])
application = window()
application.show()

# ---------- Инициализация ----------
stepper_init()
calcTime()
updateInterval()
# ---------- Отображение калибровочных данных ----------
application.ui.CalibIntegral.setText(f"{calIntegral:.2f}")
application.ui.MagMoment.setText(f"{magMoment:.10f}")
application.ui.CalibCoef.setText(f"{calCoef:.10f}")
sys.exit(app.exec_())