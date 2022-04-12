#!/usr/bin/env python

from pyproj import Proj, transform
import argparse
import pandas as pd

def changeEPSG(EPSG_in, EPSG_out, x_in, y_in):
    
    inProj = Proj(init = EPSG_in)
    outProj = Proj(init = EPSG_out)
    
    x_out, y_out = transform(inProj, outProj, x_in, y_in)
    
    return x_out, y_out


if __name__ == '__main__':

    df_epsg3857 = pd.DataFrame(columns = ["lon, lat"])
    EPSG_in = 'epsg:4326' # Lat-Lon
    EPSG_out = 'epsg:3857' # GoogleMapsSatellite

    parser = argparse.ArgumentParser()
    parser.add_argument("path", help = "Path to .CSV")
    args = parser.parse_args()

    df = pd.read_csv(args.path, delimiter = ";")

    print(df)

    df_aux = df.iloc[:, 1:3]

    print(df_aux.iloc[0, 0])
  

    for i in range(len(df_aux)):
        #lon   #lat                                  #lon               #lat
        x_out, y_out = changeEPSG(EPSG_in, EPSG_out, df_aux.iloc[i, 1], df_aux.iloc[i, 0],)

        add_values = {'lon':x_out, 'lat':y_out}
        add_row = pd.Series(add_values)
        df_epsg3857 = df_epsg3857.append(add_row, ignore_index = True)

        if i % 100 == 0:
            print(i)

    df_epsg3857.to_csv("/home/uib/georeferenced/DecimationX4_StoreDistance0_55/test.csv", index = False)


    # corners_list = []

    # corners_list.append()
    
    # x_in = 2.579083681106567 # lon
    # y_in = 39.531471252441406 # lat

    # x_out, y_out = changeEPSG(EPSG_in, EPSG_out, x_in, y_in)
    
    # print(x_out, y_out)