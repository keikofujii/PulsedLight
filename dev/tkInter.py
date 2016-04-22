import Tkinter
import random
import time
import sys

import serial

global serial

# This function initializes the Serial connection with the arduino
def initializeSerial():

    # Configure the Serial for arduino on a mac
    global serial
    serial = serial.Serial(
        port = '/dev/cu.usbmodem1411',
        baudrate = 115200,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS
    )

    # don't know what the following line is
    # serial.isOpen()
    time.sleep(2)


# This function gets the distance values of the sensors in cm and returns them in a sequential list
def getDistanceArray():

    # flush the buffer
    serial.flush()

    # This constant will request the values for all of the sensors
    sensorValsArray = []

    # Write that value to the Arduinos serial monitor
    amountOfSensors = 5

    # set inital value of i to the first sensor
    i = 1

    # set inital value of sensor val to 1 so the while loop is entered
    sensorVal = 1

    # while loop that asks for sensor values sequentially, converts them to an integer and adds them to an array
    while(sensorVal > -1):

        # ask for the sensor value from the arduino
        serial.write(str.encode(str(i)))

        # read the sensor value, conert it to an integer and store it in the temp variable
        sensorVal = int(serial.readline())

        # if the sensor was actually there store the value in the array
        if(sensorVal > -1):
            sensorValsArray.append(sensorVal)

        # delay
        time.sleep(0.01)

        # increment i to check for next sensor
        i += 1



    return sensorValsArray

#A class that bundles some of the GUI functionality. It's not necessary for it to be a class for the GUI to work, it was just kind of fun. 
class App:

    #Create the app, passing in the tkinter root that was created in the "main" function.
    def __init__(self, master):
        #build a frame (basically a window)
        frame = tkinter.Frame(master)
        #Have the frame size itself to fit into the overall GUI root. 
        frame.pack()

        #create a canvas object that can be drawn on. I just kinda ballparked the width and height.
        self.barCanvas = tkinter.Canvas(frame, width=900, height=750)
        #fit it into the GUI
        self.barCanvas.pack()

    #this function draws bars on the canvas. The variable "bars" should be a list of integers. 
    def drawBars(self, bars):
        #clear the canvas
        self.barCanvas.delete("all")
        #now draw on it again.
        #this variables kinda weird. I'll explain it in a minute
        position = 0
        #draw ALL the bars. 
        for bar in bars:
            #The create rectangle function is weird. Instead of taking the height and width of the rectangle,
            #it takes the x coordinate of the upper left corner of the rectangle, the y coordinate of the upper left corner of the rectangle
            #and then the x and y coordinates of bottom right corner. And then a fill color.
            #
            #So this draws a rectangle that has an upper left position of x=position, y=bar (so the rectangle is as high as the height of the bar we want to draw)
            # an lower right corner at position+50 (so the bar itself is 50 wide) and has its base at y=1000.
            #For some reason, the height of the bar is backwards. Right now it's being subtracted from the height of the window to normalize the values
            self.barCanvas.create_rectangle(position, 750 - (bar * .1875), position + 50, 1000, fill="cyan3")
            #self.barCanvas.create_text(position, 1, text = bar)
            #so, position. Position is the x position of the upper left corner of the rectangle. So we start out with an x position of 0,
            #and then draw a rectangle 50 wide, so we want the next rectangle to have an x position of 50 so they don't overlap. And so on.

            #This will create labels for each of the bars that will display the actual value of the bar
            self.barCanvas.create_text(position + 25, 740, text=bar)
            position += 50

#Just test code. 
def returnList():
    #replace this return with a call to the LIDAR module, or just replace all returnList references with that call. Either or. 
    return getDistanceArray()

#This is a task that essentially calls itself, so it's kind of like an infinite recursive loop, but without the stack addition since it's not waiting for the return.
#I think. 
def task():
    #call draw bars
    app.drawBars(returnList())
    #remind root to call this function again after a 100 milisecond wait. 
    root.after(100, task)

#Initialize serial connection with the Arduino
initializeSerial()

#Set up the graphical root. 
root = Tkinter.Tk()

#Build an app with it. 
app = App(root)

#Now make that graphical root remember to call task after entering its main loop. It'll wait 100 miliseconds after entering its main loop, and then call task.
root.after(100, task())
#run the mainloop of the graphical root (this is basically just drawing the graphics, responding to clicks, stuff like that). 
root.mainloop()
