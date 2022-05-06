#!/usr/bin/env python3

import os
import argparse
import pandas as pd
from pathlib import Path
from PIL import Image
from osgeo import gdal, osr


class jpg2geotiff:


    def __init__(self, inI, inCSV, outI):

        self.inputImage = inI
        self.inputGeocsv = inCSV
        self.outputImage = outI


    def jpg2tiff(self):

        im = Image.open(self.inputImage)
        im.save(self.outputImage , 'TIFF')


    def tiff2geotiff(self):

        auxImagename = self.inputImage.split("/")
        auxImagename = auxImagename[-1].split(".")
        df = pd.read_csv(self.inputGeocsv, sep = ",", index_col=False)
        columns = list(df.columns)
        corners = {}

        print(auxImagename[0] == "000000")

        if "000000" in auxImagename[0]:
            imageName = 0
        
        else:
            imageName = int(auxImagename[0].lstrip('0'))
        
        for column in columns:
            if ("lat" in column) or ("lon" in column):
                corners[column] = float(df[df[columns[0]] == imageName][column])

        ds = gdal.Open(self.outputImage, gdal.GA_Update)

        width = ds.RasterXSize
        height = ds.RasterYSize

        src = osr.SpatialReference()
        src.ImportFromEPSG(4326) 

        gcps = [
        gdal.GCP(corners['uprightlon'], corners['uprightlat'], 0, width, 0),
        gdal.GCP(corners['upleftlon'], corners['upleftlat'], 0, 0, 0),
        gdal.GCP(corners['downrightlon'], corners['downrightlat'], 0, width, height),
        gdal.GCP(corners['downleftlon'], corners['downleftlat'], 0, 0, height)]

        ds.SetGCPs(gcps, src.ExportToWkt())

        # ds = None


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = "")
    parser.add_argument("--inImage", help = "Path to .jpg or path to the directory with .jpg")
    parser.add_argument("--inCSV", help = "Path to .csv with the lat-lng of the image corners")
    parser.add_argument("--outImage", help = "Path to save .tiff")
    args = parser.parse_args()

    inI = Path(args.inImage)
    inLATLON = Path(args.inCSV)
    outI = Path(args.outImage)

    # Sanity checks
    if (inI.exists() and inLATLON.exists()):

        if not outI.exists():
            os.makedirs(args.outImage)

        if not (args.outImage[-1] is "/"):
            args.outImage = args.outImage + "/"

        if inI.is_file():
            j2gt = jpg2geotiff(args.inImage, args.inCSV, args.outImage + inI.stem + ".tiff")
            j2gt.jpg2tiff()
            j2gt.tiff2geotiff()

        elif inI.is_dir():

            if not (args.inImage[-1] is "/"):
                args.inImage = args.inImage + "/"

            files = sorted(os.listdir(args.inImage))

            for file in files:
                print(file)

                if ".jpg" in file:
                    auxFile = file.split(".")
                    j2gt = jpg2geotiff(args.inImage + file, args.inCSV, args.outImage + auxFile[0] + ".tiff")
                    j2gt.jpg2tiff()
                    j2gt.tiff2geotiff()

    else:
        print("Paths don't exist")

