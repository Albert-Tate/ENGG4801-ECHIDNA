import numpy as np
import sys
from bokeh.plotting import *

SCREEN_WIDTH = 800
PLOT_HEIGHT = 350
#           2021T,      PRES,   X   Y   Z   L
#bytes = b'{\00007E5\x0001ADB2\xFF\xAA\xCC\x05\x'

"""
struct MEASUREMENT {
    //MS5637
    int32_t TEMPERATURE;
    int32_t PRESSURE;
    //Accelerometery
    int16_t X_ACC;
    int16_t Y_ACC;
    int16_t Z_ACC;
    //Light sensor
    uint16_t LALOG;
    //Time
    uint8_t day; //0-255 (I wish)
    uint8_t hour; //lots of bad values could happen here
    uint8_t minute;
    uint8_t second;
    //TOTAL 160 bit (1Mbit = 2^20, 2^20/144 = 7281 across 2 thingys = 14652 = 4 hours of sampling at 1hz. pretty neato
    //Debug: Battery remaining (0-10 000)
    //int16_t BATT;
    //now 172 bit
"""

print sys.argv[1]

#prep data
N = 100
#Pull out all time in seconds sequentially -> might look like x =[0, 1, 2, 3, 4]
x = np.linspace(0, 100, N)

TEMP = np.random.uniform(20,29, size=N)
PRESSURE = np.random.uniform(100, 105, size=N)
XACC = np.random.uniform(-2,2, size = N)
YACC = np.random.uniform(-2,2, size = N)
ZACC = np.random.uniform(-2,2, size = N)
LALOG = np.random.uniform(0,65535, size = N)
"""
x = np.linspace(0, 4*np.pi, N)
y0 = np.sin(x)
y1 = np.cos(x)
y2 = np.sin(x) + np.cos(x)
"""
output_file("lines.html", title="uTracker Data")

s1 = figure(width=SCREEN_WIDTH, plot_height=PLOT_HEIGHT,
            title="Temperature", x_axis_label='t(s)', y_axis_label='Degrees (C)')
s1.line(x, TEMP, line_width=2)

s2 = figure(width=SCREEN_WIDTH, height=PLOT_HEIGHT, x_range=s1.x_range,
            title='Pressure', x_axis_label='t(s)', y_axis_label='Air Pressure kPa')
s2.line(x, PRESSURE, line_width=2)

s3 = figure(width=SCREEN_WIDTH, height=PLOT_HEIGHT, x_range=s1.x_range,
            title='Accelerometry', x_axis_label='t(s)', y_axis_label='g force')
s3.line(x, XACC, legend="X Axis", line_width=2, color='red')
s3.line(x, YACC, legend="Y Axis", line_width=2, color='blue')
s3.line(x, ZACC, legend="Z Axis", line_width=2, color='green')

s4 = figure(width=SCREEN_WIDTH, height=PLOT_HEIGHT, x_range=s1.x_range,
            title='Light Intensity', x_axis_label='t(s)', y_axis_label='Light Intensity')
s4.line(x, LALOG, line_width=2)

p = vplot(s1, s2, s3, s4)

show(p)


"""


#output to file
output_file("lines.html", title="line plot example")

#Create plot
p = figure(title="simple line example", x_axis_label='time', y_axis_label='temp')

#Add line renderer
p.line(x,y, legend="Temp.", line_width=2)

#Create plot
q = figure(title="simple line example", x_axis_label='x', y_axis_label='y')

#Add line renderer
q.line(x,y, legend="Temp2.", line_width=2)

#show results
show(p)
"""
