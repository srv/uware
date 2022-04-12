#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 14:09:21 2022

@author: uib
"""

import pandas as pd

df = pd.read_csv("/home/uib/georeferenced/DecimationX4_StoreDistance0_55/latlonimages.csv", delimiter = ";")

df_aux = df.iloc[:, 1:3]