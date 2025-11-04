#!/usr/bin/python
# -*- coding: latin-1 -*-
import RPi.GPIO as GPIO
from pad4pi import rpi_gpio
import RPi.GPIO as IO        # calling for header file for GPIOÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ¢ÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂs of PI
import time                           # calling for time to provide delays in program
import smbus
import sys
import serial
import datetime
from gpiozero import Servo
from time import sleep
 
myGPIO=25
 
myCorrection=0.45
maxPW=(2.0+myCorrection)/1000
minPW=(1.0-myCorrection)/1000
 
servo = Servo(myGPIO,min_pulse_width=minPW,max_pulse_width=maxPW)
servo.min()
servo.detach()

KEYPAD = [[1,2,3,'A'],
     [4,5,6,'B'],
     [7,8,9,'C'],
     ['*',0,'#','D']]

ROW_PINS = [6,13,19,26]
COL_PINS = [12,16,20,21]


now = datetime.datetime.now()
entered_password = ""
correct_password = "99999"


# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address, if any error, change this address to 0x3f
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1

def sent_message():
    # Enable Serial Communication
    port = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=1)
     
    # Transmitting AT Commands to the Modem
    # '\r\n' indicates the Enter key
     
    port.write('AT'+'\r\n')
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
     
    port.write('ATE0'+'\r\n')      # Disable the Echo
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
     
    port.write('AT+CMGF=1'+'\r\n')  # Select Message format as Text mode 
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
     
    port.write('AT+CNMI=2,1,0,0,0'+'\r\n')   # New SMS Message Indications
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
     
    # Sending a message to a particular Number
     
    port.write('AT+CMGS="+60174914051"'+'\r\n')
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
     
    port.write('Reminder!! \n Server Room 2 was opened on :' + (now.strftime(" %H:%M:%S \n %d-%m-%Y")) +'\r\n')  # Message
    rcv = port.read(10)
    print (rcv)
     
    port.write("\x1A") # Enable to send SMS
    for i in range(10):
        rcv = port.read(10)
        print (rcv)


def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for data
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def countdown():
    for i in reversed(range(0, 10)):
        time.sleep(1)
        lcd_string("Please Close Before",LCD_LINE_1)
        lcd_string("ore : %s\r" %i,LCD_LINE_2)
        print ("%s\r" %i,)

def cleanup():
    global keypad
    keypad.cleanup()

    lcd_init()
    #sys.exit()
    

def correct_password_entered():
    lcd_string("> Access Granted!!  <",LCD_LINE_1)
    lcd_string("> Door Opened!  <",LCD_LINE_2)
    servo.max()
    time.sleep(1)
    sent_message()
    countdown()
    #lcd_string("> Access Denied!!  <",LCD_LINE_1)
    lcd_string("> Door Closed!  <",LCD_LINE_2)
    servo.min()
    time.sleep(2)
    cleanup()
    
def incorrect_password_entered():
    lcd_string("> Access Denied!!  <",LCD_LINE_1)
    lcd_string("> Door Closed!  <",LCD_LINE_2)
    time.sleep(5)
    cleanup()
    
def digit_entered(key):
    global entered_password, correct_password
    
    entered_password += str(key)
    
    lcd_string(entered_password,LCD_LINE_2)
    
    if len(entered_password) == len(correct_password):
        if entered_password == correct_password:
            correct_password_entered()
        else:
            incorrect_password_entered()
            
def non_digit_entered(key):
    global entered_password
    
    if key== "*" and len(entered_password) > 0:
        entered_password = entered_password[:-1]
        lcd_string(entered_password,LCD_LINE_2)
        
def key_pressed(key):
    try:
        int_key = int(key)
        if int_key >= 0 and int_key <= 9:
            digit_entered(key)
    except ValueError:
        non_digit_entered(key)
        
while True:            
    try:
        while True:
                lcd_init()
                factory = rpi_gpio.KeypadFactory()

        # Try factory.create_4_by_3_keypad
        # and factory.create_4_by_4_keypad for reasonable defaults
                keypad = factory.create_keypad(keypad=KEYPAD, row_pins=ROW_PINS, col_pins=COL_PINS)
        #def printKey(key):
        #print(key)
        # printKey will be called each time a keypad button is pressed
                keypad.registerKeyPressHandler(key_pressed)


                while True:
                    time.sleep(1)
            
    except KeyboardInterrupt:
        cleanup()
        
    finally:
        cleanup()

