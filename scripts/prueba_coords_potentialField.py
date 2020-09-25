#! /usr/bin/env python
import numpy as np

arr = [0.049, 3.1564,2.165485,4.5464,-1.15615]
output=[]
for i in arr:
    output.append(round((np.ceil(round(i,2) / 0.05) * 0.05) ,2))
print(output)
# expected output (0.05, 3.20, 2.20, 4.55, -1.20)