import qwiic_icm20948
import time
import sys
import csv
#import pandas

def runExample(filename):
	print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
	IMU = qwiic_icm20948.QwiicIcm20948()

	if IMU.connected == False:
		print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return

	IMU.begin()
	IMU.setFullScaleRangeGyro(0x02)
	mxList = []
	myList = []
	mzList = []
	i = 0
	print("starting magCal readings")
	while i < 200:
		if IMU.dataReady():
			IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
			mVec = [IMU.mxRaw*0.15, IMU.myRaw*0.15, IMU.mzRaw*0.15]
			mxList.append(mVec[0])
			myList.append(mVec[1])
			mzList.append(mVec[2])
			i += 1
			print("M:", i)
			time.sleep(0.2)
		else:
			print("Waiting for data")
			time.sleep(0.5)
	print("done with magCal readings")
	
	j = 0
	with open(filename, "w") as csvFile:
		a = csv.writer(csvFile)
		a.writerow(["mag_x", "mag_y", "mag_z"])
		while j < i:
			a.writerow([mxList[j], myList[j], mzList[j]])
			j += 1
	
	return mxList, myList, mzList
	

def main(filename):
    bxhat, byhat, bzhat = runExample(filename)
	
                               
