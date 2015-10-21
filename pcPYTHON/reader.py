import serial
import csv
import struct

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

ser = serial.Serial('COM3', 142000)

FORMAT_STR = "<iihhhHBBBB"

print struct.calcsize(FORMAT_STR)
EXPECTED = 60
RECEIVED = 0

with open('data.csv', 'wb') as csvfile:
    datawriter = csv.writer(csvfile, delimiter=',')
    datawriter.writerow(['Temp', 'Pres', 'Xa', 'Ya', 'Za', 'La', 'D', 'h', 'm', 's'])
    while(1):
        x = ser.read(20)
        y = ser.read(1)
        if y != '\n':
            if y == 'T':
                break
            ser.readline()
            continue
        RECEIVED = RECEIVED + 1
        #print ":".join("{:02x}".format(ord(c)) for c in x)
        if len(x) == 20:
            f = struct.unpack(FORMAT_STR, x)
            print f
            datawriter.writerow([f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9] ])
            if RECEIVED == EXPECTED:
                break
