#!/usr/bin/python
#
#       This program includes a number of calculations to improve the
#       values returned from a BerryIMU. If this is new to you, it
#       may be worthwhile first to look at berryIMU-simple.py, which
#       has a much more simplified version of code which is easier
#       to read.
#
#
#       The BerryIMUv1, BerryIMUv2 and BerryIMUv3 are supported
#
#       This script is python 2.7 and 3 compatible
#
#       Feel free to do whatever you like with this code.
#       Distributed as-is; no warranty is given.
#
#       https://ozzmaker.com/berryimu/
import time
import math
import IMU
import datetime
import os
import sys
import bearing


class Heading:
    def __init__(self):
        self.initialize_system()
        self.RAD_TO_DEG = 57.29578
        self.M_PI = 3.14159265358979323846
        self.G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
        self.AA =  0.40      # Complementary filter constant

        self.MAGNETIC_DECLINATION = -12.5 # Declination angle of your location in radians (if 0, then magnetic North is straight up)

        ################# Compass Calibration values ############
        self.magXmin =  -1272
        self.magYmin =  -2837
        self.magZmin =  -2546
        self.magXmax =  2749
        self.magYmax =  867
        self.magZmax =  1411

        ############### END Calibration offsets #################

        #Kalman filter variables
        self.Q_angle = 0.02
        self.Q_gyro = 0.0015
        self.R_angle = 0.005
        self.y_bias = 0.0
        self.x_bias = 0.0
        self.XP_00 = 0.0
        self.XP_01 = 0.0
        self.XP_10 = 0.0
        self.XP_11 = 0.0
        self.YP_00 = 0.0
        self.YP_01 = 0.0
        self.YP_10 = 0.0
        self.YP_11 = 0.0
        self.KFangleX = 0.0
        self.KFangleY = 0.0

    def initialize_system(self):
        IMU.detectIMU()     #Detect if BerryIMU is connected.
        if(IMU.BerryIMUversion == 99):
            print(" No BerryIMU found... exiting ")
            sys.exit()
        IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

    def kalmanFilterY (self, accAngle, gyroRate, DT):
        y=0.0
        S=0.0

        self.KFangleY = self.KFangleY + DT * (gyroRate - self.y_bias)

        self.YP_00 = self.YP_00 + ( - DT * (self.YP_10 + self.YP_01) + self.Q_angle * DT )
        self.YP_01 = self.YP_01 + ( - DT * self.YP_11 )
        self.YP_10 = self.YP_10 + ( - DT * self.YP_11 )
        self.YP_11 = self.YP_11 + ( + self.Q_gyro * DT )

        y = accAngle - self.KFangleY
        S = self.YP_00 + self.R_angle
        K_0 = self.YP_00 / S
        K_1 = self.YP_10 / S

        self.KFangleY = self.KFangleY + ( K_0 * y )
        y_bias = y_bias + ( K_1 * y )

        self.YP_00 = self.YP_00 - ( K_0 * self.YP_00 )
        self.YP_01 = self.YP_01 - ( K_0 * self.YP_01 )
        self.YP_10 = self.YP_10 - ( K_1 * self.YP_00 )
        self.YP_11 = self.YP_11 - ( K_1 * self.YP_01 )

        return self.KFangleY

    def kalmanFilterX (self, accAngle, gyroRate, DT):
        x=0.0
        S=0.0

        self.KFangleX = self.KFangleX + DT * (gyroRate - self.x_bias)

        self.XP_00 = self.XP_00 + ( - DT * (self.XP_10 + self.XP_01) + self.Q_angle * DT )
        self.XP_01 = self.XP_01 + ( - DT * self.XP_11 )
        self.XP_10 = self.XP_10 + ( - DT * self.XP_11 )
        self.XP_11 = self.XP_11 + ( + self.Q_gyro * DT )

        x = accAngle - self.KFangleX
        S = self.XP_00 + self.R_angle
        K_0 = self.XP_00 / S
        K_1 = self.XP_10 / S

        KFangleX = KFangleX + ( K_0 * x )
        x_bias = x_bias + ( K_1 * x )

        self.XP_00 = self.XP_00 - ( K_0 * self.XP_00 )
        self.XP_01 = self.XP_01 - ( K_0 * self.XP_01 )
        self.XP_10 = self.XP_10 - ( K_1 * self.XP_00 )
        self.XP_11 = self.XP_11 - ( K_1 * self.XP_01 )

        return self.KFangleX


    def get_heading(self):
        gyroXangle = 0.0
        gyroYangle = 0.0
        gyroZangle = 0.0
        CFangleX = 0.0
        CFangleY = 0.0
        kalmanX = 0.0
        kalmanY = 0.0

        a = datetime.datetime.now()

        while True:
            #Read the accelerometer,gyroscope and magnetometer values
            ACCx = IMU.readACCx()
            ACCy = IMU.readACCy()
            ACCz = IMU.readACCz()
            GYRx = IMU.readGYRx()
            GYRy = IMU.readGYRy()
            GYRz = IMU.readGYRz()
            MAGx = IMU.readMAGx()
            MAGy = IMU.readMAGy()
            MAGz = IMU.readMAGz()


            #Apply compass calibration
            MAGx -= (self.magXmin + self.magXmax) /2
            MAGy -= (self.magYmin + self.magYmax) /2
            MAGz -= (self.magZmin + self.magZmax) /2


            ##Calculate loop Period(LP). How long between Gyro Reads
            b = datetime.datetime.now() - a
            a = datetime.datetime.now()
            LP = b.microseconds/(1000000*1.0)
            outputString = "Loop Time %5.2f " % ( LP )



            #Convert Gyro raw to degrees per second
            rate_gyr_x =  GYRx * self.G_GAIN
            rate_gyr_y =  GYRy * self.G_GAIN
            rate_gyr_z =  GYRz * self.G_GAIN


            #Calculate the angles from the gyro.
            gyroXangle+=rate_gyr_x*LP
            gyroYangle+=rate_gyr_y*LP
            gyroZangle+=rate_gyr_z*LP



        #Convert Accelerometer values to degrees
            AccXangle =  (math.atan2(ACCy,ACCz)*self.RAD_TO_DEG)
            AccYangle =  (math.atan2(ACCz,ACCx)+self.M_PI)*self.RAD_TO_DEG

            #convert the values to -180 and +180
            if AccYangle > 90:
                AccYangle -= 270.0
            else:
                AccYangle += 90.0


            #Complementary filter used to combine the accelerometer and gyro values.
            CFangleX=self.AA*(CFangleX+rate_gyr_x*LP) +(1 - self.AA) * AccXangle
            CFangleY=self.AA*(CFangleY+rate_gyr_y*LP) +(1 - self.AA) * AccYangle

            #Kalman filter used to combine the accelerometer and gyro values.
            kalmanY = self.kalmanFilterY(AccYangle, rate_gyr_y,LP)
            kalmanX = self.kalmanFilterX(AccXangle, rate_gyr_x,LP)


            #Calculate heading
            heading = (180 * math.atan2(MAGy,MAGx)/self.M_PI) - self.MAGNETIC_DECLINATION

            #Only have our heading between 0 and 360
            if heading < 0:
                heading += 360

            ####################################################################
            ###################Tilt compensated heading#########################
            ####################################################################
            #Normalize accelerometer raw values.
            accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)


            #Calculate pitch and roll
            pitch = math.asin(accXnorm)
            roll = -math.asin(accYnorm/math.cos(pitch))


            #Calculate the new tilt compensated values
            #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
            #This needs to be taken into consideration when performing the calculations

            #X compensation
            if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
                magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
            else:                                                                #LSM9DS1
                magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)

            #Y compensation
            if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
            else:                                                                #LSM9DS1
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)




            #Calculate tilt compensated heading
            tiltCompensatedHeading = (180 * math.atan2(magYcomp,magXcomp)/self.M_PI) - self.MAGNETIC_DECLINATION

            if tiltCompensatedHeading < 0:
                tiltCompensatedHeading += 360


            ##################### END Tilt Compensation ########################



            if 1:                       #Change to '0' to stop showing the angles from the accelerometer
                outputString += "#  ACCX Angle %5.2f ACCY Angle %5.2f  #  " % (AccXangle, AccYangle)

            if 1:                       #Change to '0' to stop  showing the angles from the gyro
                outputString +="\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)

            if 1:                       #Change to '0' to stop  showing the angles from the complementary filter
                outputString +="\t#  CFangleX Angle %5.2f   CFangleY Angle %5.2f  #" % (CFangleX,CFangleY)

            if 1:                       #Change to '0' to stop  showing the heading
                outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)

            if 1:                       #Change to '0' to stop  showing the angles from the Kalman filter
                outputString +="# kalmanX %5.2f   kalmanY %5.2f #" % (kalmanX,kalmanY)

            return outputString

            #slow program down a bit, makes the output more readable
            #time.sleep(0.03)
