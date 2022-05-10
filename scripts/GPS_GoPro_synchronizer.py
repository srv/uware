#!/usr/bin/env python3

import os
import argparse
import pyexiv2
import pandas as pd
from PIL import Image


class Synchronizer:

    def __init__(self, inI, inGPS, outCSV):

        # Declare paths.
        self.inputImage = inI
        self.inputGPS = inGPS
        self.outputCSV = outCSV

        # Determine if the output files exist. 
        if os.path.isdir( self.outputCSV):
            if not "/" in self.outputCSV[-1]:
                self.outputCSV += "/"
            self.exist = False
            for file in sorted(os.listdir(self.outputCSV)):
                if (".csv" in file) and not ((self.outputCSV + file) is inGPS):
                    self.outputDataFrame = pd.read_csv(self.outputCSV + file, sep = ",")
                    self.exist = True
                    self.outputCSV += file
                    break
            if not self.exist:
                self.outputDataFrame = pd.DataFrame(columns = ['#id', 'Latitude', 'Longitude', 'Timestamp'])
        elif os.path.isfile(self.outputCSV) and (".csv" in self.outputCSV):
            self.outputDataFrame = pd.read_csv(self.outputCSV, sep = ",")
            self.exist = True


    # def fpsDeterminer(self):
    #     print()

    def synchronize(self):

        # Obtain image name.
        idImage = self.inputImage.split("/")
        idImage = idImage[-1]
        print(idImage)

        # Obtain image time stamp from image metadata.
        metadata = pyexiv2.ImageMetadata(self.inputImage)
        metadata.read()
        for exif_key in metadata.exif_keys:
            if "Image.DateTime" in exif_key:
                timeImage = metadata[exif_key].raw_value.split(" ")
                timeImage = timeImage[-1]

        # Read CSV with GPS locations, then clean the dataset.
        if ".LLH" in self.inputGPS:
            df = pd.read_csv(self.inputGPS, sep = " ")
            df = df.dropna(axis = 1)
            df.columns = ['Date', 'Timestamp', 'Latitude', 'Longitude', 'Height', 'Random 1', 'Random 2']
        elif ".csv" in self.inputGPS:
            df = pd.read_csv(self.inputGPS, sep = ",")

        # Determine the miliseconds of the image, depends on the fps
        timestamps = list(df[df['Timestamp'].str.contains(timeImage)]['Timestamp'])
        if len(self.outputDataFrame) > 0:
            if not self.outputDataFrame['Timestamp'].str.contains(timestamps[0]).any():
                timeImage = timestamps[0]
            else:
                timeImage = timestamps[3]
        else:
            timeImage = timestamps[0]
        print(timeImage)

        # Obtain latitude and longitude of the image.
        latitudeImage = float(df[df['Timestamp'] == timeImage]['Latitude'])
        longitudeImage = float(df[df['Timestamp'] == timeImage]['Longitude'])

        # Add new row to the dataset and write it to a file.
        add_values = {'#id': idImage, 
                      'Latitude': latitudeImage,
                      'Longitude': longitudeImage,
                      'Timestamp': timeImage}
        add_row = pd.Series(add_values)
        self.outputDataFrame = self.outputDataFrame.append(add_row, ignore_index = True)
        if self.exist:
            self.outputDataFrame.to_csv(path_or_buf = self.outputCSV, sep = ",", index = False)
        else:
            self.outputDataFrame.to_csv(path_or_buf = self.outputCSV + "image_latlon.csv", sep = ",", index = False)

        print("---------------------------------------------------------------------------")


# Auxiliar function that transform strings to booleans.
def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 'True', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'False', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


# Auxiliar function to donwsample images.
def decimateImages(imagePath, folderPath, decimation):

    decimatePath = folderPath + "/" + "decimateX2"

    if not os.path.exists(decimatePath):
        os.makedirs(decimatePath)

    auxImageName = imagePath.split("/")
    auxImageName = auxImageName[-1]

    image = Image.open(imagePath)
    width, height = image.size
    new_image = image.resize((int(width / decimation), int(height / decimation)))
    new_image.save(decimatePath + "/" + auxImageName)


if __name__ == '__main__':

    # Read from the command line.
    parser = argparse.ArgumentParser(description = "")
    parser.add_argument("--inImage", help = "Path to .jpg or path to the directory with .jpg")
    parser.add_argument("--inGPS", help = "Path to GPS.csv")
    parser.add_argument("--outCSV", default = os.getcwd(), help = "Path to save .csv file with images and lat-lng")
    parser.add_argument("--enableDecimateX2", type = str2bool, nargs = '?', const = True, default = False, help = "Enable Decimate X2? (True/False)")
    args = parser.parse_args()

    # Sanity check.
    if os.path.exists(args.inImage) and os.path.exists(args.inGPS):

        # Synchronize one image.
        if os.path.isfile(args.inImage) and os.path.isfile(args.inGPS):
            sync = Synchronizer(args.inImage, args.inGPS, args.outCSV)
            sync.synchronize()
            if args.enableDecimateX2:
                decimateImages(args.inImage, os.path.dirname(args.inImage), 2)

        # Synchronize some images.
        elif os.path.isdir(args.inImage) and os.path.isfile(args.inGPS):

            for file in sorted(os.listdir(args.inImage)):
                image = args.inImage + "/" + file
                if os.path.isfile(image) and (".JPG" in image):
                    sync = Synchronizer(image, args.inGPS, args.outCSV)
                    sync.synchronize()
                    if args.enableDecimateX2:
                        decimateImages(image, args.inImage, 2)

    else:
        print("Input paths don't exists")
