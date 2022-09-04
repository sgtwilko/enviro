import time
from breakout_bme280 import BreakoutBME280
from breakout_ltr559 import BreakoutLTR559
from machine import Pin, PWM
from enviro import i2c
from phew import logging

bme280 = BreakoutBME280(i2c, 0x77)
ltr559 = BreakoutLTR559(i2c)

piezo_pwm = PWM(Pin(28))

moisture_sensor_pins = [
  Pin(15, Pin.IN, Pin.PULL_DOWN),
  Pin(14, Pin.IN, Pin.PULL_DOWN),
  Pin(13, Pin.IN, Pin.PULL_DOWN)
]

pump_pins = [
  Pin(12, Pin.OUT),
  Pin(11, Pin.OUT),
  Pin(10, Pin.OUT)
]

pump_runtime = [0, 0, 0]

running_pump = -1

pump_start_tick = 0

def moisture_readings():
  results = []

  for i in range(0, 3):
    # count time for sensor to "tick" 25 times
    sensor = moisture_sensor_pins[i]

    last_value = sensor.value()
    start = time.ticks_ms()
    first = None
    last = None
    ticks = 0
    while ticks < 10 and time.ticks_ms() - start <= 1000:
      value = sensor.value()
      if last_value != value:
        if first == None:
          first = time.ticks_ms()
        last = time.ticks_ms()
        ticks += 1
        last_value = value

    if not first or not last:
      results.append(0.0)
      continue

    # calculate the average tick between transitions in ms
    average = (last - first) / ticks
    # scale the result to a 0...100 range where 0 is very dry
    # and 100 is standing in water
    #
    # dry = 10ms per transition, wet = 90ms per transition
    min_ms = 20
    max_ms = 80
    average = max(min_ms, min(max_ms, average)) # clamp range
    scaled = ((average - min_ms) / (max_ms - min_ms)) * 100
    results.append(round(scaled, 2))

  return results

def get_sensor_readings():
  # bme280 returns the register contents immediately and then starts a new reading
  # we want the current reading so do a dummy read to discard register contents first
  bme280.read()
  time.sleep(0.1)
  bme280_data = bme280.read()

  ltr_data = ltr559.get_reading()

  moisture_data = moisture_readings()

  from ucollections import OrderedDict

  return OrderedDict({
    "temperature": round(bme280_data[0], 2),
    "humidity": round(bme280_data[2], 2),
    "pressure": round(bme280_data[1] / 100.0, 2),
    "light": round(ltr_data[BreakoutLTR559.LUX], 2),
    "moisture_1": round(moisture_data[0], 2),
    "moisture_2": round(moisture_data[1], 2),
    "moisture_3": round(moisture_data[2], 2)
  })
  
def play_tone(frequency = None):
  if frequency:
    piezo_pwm.freq(frequency)
    piezo_pwm.duty_u16(32768)

def stop_tone():
  piezo_pwm.duty_u16(0)

def start_pump(pump_number):
  global running_pump, pump_start_tick
  stop_pump()
  if pump_number >=0 and pump_number<3:
    logging.debug(f"      - Starting pump {pump_number}")
    running_pump = pump_number
    pump_start_tick = time.ticks_ms()
    pump_pins[running_pump].value(1)
        
def stop_pump():
  global running_pump, pump_runtime, pump_start_tick
  if running_pump>=0:
    logging.debug(f"      - stopping pump {running_pump}")
    pump_runtime[running_pump] += time.ticks_diff(time.ticks_ms(), pump_start_tick)
    running_pump=-1;
  for i in range(0, 3):
    pump_pins[i].value(0)
        
def actionValues(thresholds):
  logging.debug("  - anything to do")
  check_for_action = True
  actioned = False
  start = time.ticks_ms()
  while check_for_action and time.ticks_diff(time.ticks_ms(), start)<60000:
    check_for_action = False
    logging.debug("    - Get readings")
    moisture_data = moisture_readings()
    logging.debug(moisture_data)
    for index, val in enumerate(moisture_data):
      if val>0 and val<thresholds[index]:
        logging.debug(f"      - start pump {index}, {val}<{thresholds[index]}")
        check_for_action = True
        start_pump(index)
        actioned = True
      time.sleep(5)
      stop_pump()            
  logging.debug(pump_runtime)
  return actioned   

def stop_actions():
  stop_pump()
