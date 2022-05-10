#!/usr/bin/env python3

import os
import argparse
import pandas as pd
from pathlib import Path
from PIL import Image
from osgeo import gdal, osr

class Jpg2Geotiff:


    def __init__(self, inI, inCSV, outI):

        # Declare paths.
        self.inputImage = inI
        self.inputGeocsv = inCSV
        self.outputImage = outI


    def jpg2tiff(self):

        im = Image.open(self.inputImage)
        im.save(self.outputImage , 'TIFF')


    def tiff2geotiff(self):

        # Get the image name frome the input path.
        auxImagename = self.inputImage.split("/")
        auxImagename = auxImagename[-1].split(".")

        # Dataset with all the latitude and longitude coordinates.
        df = pd.read_csv(self.inputGeocsv, sep = ",", index_col=False)
        columns = list(df.columns)
        corners = {}

        # Remove all leading zeros on the image name.
        if "000000" in auxImagename[0]:
            imageName = 0
        else:
            imageName = int(auxImagename[0].lstrip('0'))
        
        # Get the latitude and longitude, from the dataset with all the latitude
        # and longitude coordinates, of all the corners of the image.
        for column in columns:
            if ("lat" in column) or ("lon" in column):
                corners[column] = float(df[df[columns[0]] == imageName][column])

        ds = gdal.Open(self.outputImage, gdal.GA_Update)

        # Get raster dimensions
        width = ds.RasterXSize
        height = ds.RasterYSize

        # Set src
        src = osr.SpatialReference()
        src.ImportFromEPSG(4326) 

        # Set latitude and longitude to all image corners
        gcps = [
        gdal.GCP(corners['uprightlon'], corners['uprightlat'], 0, width, 0),
        gdal.GCP(corners['upleftlon'], corners['upleftlat'], 0, 0, 0),
        gdal.GCP(corners['downrightlon'], corners['downrightlat'], 0, width, height),
        gdal.GCP(corners['downleftlon'], corners['downleftlat'], 0, 0, height)]
        ds.SetGCPs(gcps, src.ExportToWkt())

        # ds = None


if __name__ == '__main__':

    # Read from the command line
    parser = argparse.ArgumentParser(description = "")
    parser.add_argument("--inImage", help = "Path to .jpg or path to the directory with .jpg")
    parser.add_argument("--inCSV", help = "Path to .csv with the lat-lng of the image corners")
    parser.add_argument("--outImage", help = "Path to save .tiff")
    args = parser.parse_args()

    inI = Path(args.inImage)
    inLATLON = Path(args.inCSV)
    outI = Path(args.outImage)

    # Sanity check
    if (inI.exists() and inLATLON.exists()):

        # Sanity check
        if not outI.exists():
            os.makedirs(args.outImage)

        # Sanity check
        if not (args.outImage[-1] is "/"):
            args.outImage = args.outImage + "/"

        # Transform one .JPG
        if inI.is_file():
            j2gt = Jpg2Geotiff(args.inImage, args.inCSV, args.outImage + inI.stem + ".tiff")
            j2gt.jpg2tiff()
            j2gt.tiff2geotiff()

        # Transform some .JPG
        elif inI.is_dir():
            if not (args.inImage[-1] is "/"):
                args.inImage = args.inImage + "/"
            files = sorted(os.listdir(args.inImage))
            for file in files:
                print(file)
                if ".jpg" in file:
                    auxFile = file.split(".")
                    j2gt = Jpg2Geotiff(args.inImage + file, args.inCSV, args.outImage + auxFile[0] + ".tiff")
                    j2gt.jpg2tiff()
                    j2gt.tiff2geotiff()

    else:
        print("Paths don't exist")

