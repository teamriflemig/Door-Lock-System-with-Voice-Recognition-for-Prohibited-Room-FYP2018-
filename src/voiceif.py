#!/usr/bin/python
# -*- coding: latin-1 -*-
import speech_recognition as sr
import RPi.GPIO as IO        # calling for header file for GPIOÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ¢ÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂs of PI
import time                           # calling for time to provide delays in program
import smbus
import os, time
import datetime
import serial

import sys
from gpiozero import Servo
from time import sleep
 
myGPIO=25
 
myCorrection=0.45
maxPW=(2.0+myCorrection)/1000
minPW=(1.0-myCorrection)/1000
 
servo = Servo(myGPIO,min_pulse_width=minPW,max_pulse_width=maxPW)
servo.min()
servo.detach()

now = datetime.datetime.now()

#IO.setwarnings(False)          # do not show any warnings
#IO.setmode (IO.BCM)            # programming the GPIO by BCM pin numbers. (like PIN29 asÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ¢ÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂGPIO5ÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ¢ÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ)
#IO.setup(25,IO.OUT)             # initialize GPIO19 as an output
#p = IO.PWM(25,50)              # GPIO19 as PWM output, with 50Hz frequency
#p.start(7.5)                             # generate PWM signal with 7.5% duty cycle


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
    
def cleanup():
    global keypad
    keypad.cleanup()

def countdown():
    for i in reversed(range(0, 15)):
        time.sleep(1)
        lcd_string("Please Close Before",LCD_LINE_1)
        lcd_string("ore : %s\r" %i,LCD_LINE_2)
        print ("%s\r" %i,)

def main():
  # Main program block

  # Initialise display
    lcd_init()
  
  
#k = sr.Recognizer()
r = sr.Recognizer()
r.energy_threshold = 550

f = open("voicepassword.txt", "r") 

lists = []
i = 0

for line in f: 
    lists.append(line)
    

        
while 1:
    with sr.Microphone() as source:
        lcd_init()
        #Asking open the door
        lcd_string("> Door is lock!!  <",LCD_LINE_1)
        time.sleep(2)
        lcd_string("> Wanna open  ",LCD_LINE_1)
        lcd_string("> door? Say it!! ",LCD_LINE_2)
        print("Listening... :")
        audio = r.listen(source)
        r.adjust_for_ambient_noise(source, duration=4)
        try:
            text = r.recognize_google(audio)
            lcd_string("> Alright!!  <",LCD_LINE_1)
            print("You said : {}".format(text))
            
            if (text == ("Open Door") or text == ("open the door")):
                lcd_string("> Alright!!  <",LCD_LINE_1)
                print("Password :")
                #Asking the password for authetication
                lcd_string(">  Tell Your  <",LCD_LINE_1)
                lcd_string(">  Password!  <",LCD_LINE_2)
                audio2 = r.listen(source)
                access = r.recognize_google(audio2)
                acc= access+"\n"
                print(acc)
                valid = False
                
                for list in lists:
                    if  acc == list:
                        valid = True 
                        break
                 
                if valid:
                    print ("SUCCESS")
                    lcd_string("> Access Granted!!  <",LCD_LINE_1)
                    lcd_string("> Door Opened!  <",LCD_LINE_2)
                    #p.ChangeDutyCycle(12.5)                  # change duty cycle for getting the servo position to 1
                    servo.max()
                    time.sleep(1)
                    countdown()
                    # Send some test
                    #lcd_string("> Access Denied!!  <",LCD_LINE_1)
                    lcd_string("> Door Closed!  <",LCD_LINE_2)
                    servo.min()
                    # Sent message GSM
                    sent_message()
                    time.sleep(1)
                    #servo.detach()
                    #cleanup()
                    #p.ChangeDutyCycle(2.5)                  # change duty cycle for getting the servo position to 0ÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂº
                    #p.disable()
                    time.sleep(3)
                    
                else:
                    print ("WRONG")
                    lcd_string("> Wrong Password!!  <",LCD_LINE_1)
                    lcd_string("> Start Again!  <",LCD_LINE_2)
                    time.sleep(4)
                        
            
            else:
                print ("Please try again!")
                lcd_string("> Sorry. I can ",LCD_LINE_1)
                lcd_string("> open door only.",LCD_LINE_2)
                time.sleep(3)
            
            
        except:
            print("Sorry could not recognize what you said")
            time.sleep(3)
        
        
if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)