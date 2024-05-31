import magCalMeasure  #runs file to measure magnetic field vals as vehicle is rotated
import magCalParameter   #runs parameter calculations, outputs to file

# See accompanying documentation for how to use this code.
inputFileName = input("Name for raw magnetometer data storage file: ")
inputMagNorm = float(input("Reference normal magnetic field at test site (preferably check outdoors with IGRF/WMM): "))

# Comment out if not using SparkFun:
magCalMeasure.main(inputFileName)

magCalParameter.main(inputFileName, inputMagNorm)