#!/usr/bin/env python

from pyproj import Proj, transform


def changeEPSG(EPSG_in, EPSG_out, x_in, y_in):
    
    inProj = Proj(init = EPSG_in)
    outProj = Proj(init = EPSG_out)
    
    x_out, y_out = transform(inProj, outProj, x_in, y_in)
    
    return x_out, y_out


if __name__ == '__main__':

    EPSG_in = 'epsg:4326' # Lat-Lon
    EPSG_out = 'epsg:3857' # GoogleMapsSatellite

    upright = [39.531265258789062, 2.578706264495850]
    upleft = [39.531276702880859, 2.578685522079468]
    downright = [39.531276702880859, 2.578717470169067]
    downleft = [39.531288146972656,	2.578696727752686]

    x_upright, y_uprigt = changeEPSG(EPSG_in, EPSG_out, upright[1], upright[0])
    print(x_upright, y_uprigt)
    x_upleft, y_upleft = changeEPSG(EPSG_in, EPSG_out, upleft[1], upleft[0])
    print(x_upleft, y_upleft)
    x_downright, y_downright = changeEPSG(EPSG_in, EPSG_out, downright[1], downright[0])
    print(x_downright, y_downright)
    x_downleft, y_downleft = changeEPSG(EPSG_in, EPSG_out, downleft[1], downleft[0])
    print(x_downleft, y_downleft)
        
