import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
import csv
from scipy.interpolate import splrep, BSpline


if __name__ == "__main__":
    from svgpathtools import svg2paths

    lines, _  = svg2paths('./trajectory/2023AutoX.svg')

    TRUE_WIDTH = 1205
    TRUE_HEIGHT = 105

    should_flip_axes = False

    #last = None
    for line in lines:
        NUM_SAMPLES = 750

        myPath = []
        with open("./trajectory/23_michigan_autox_ft.csv", "w", newline='', encoding='utf-8') as outfile:
            spamwriter = csv.writer(outfile, delimiter=',')
            spamwriter.writerow(["X", "Y"])
            Xs = []
            Ys = []
            for i in range(NUM_SAMPLES):
                pt = line.point(i/(NUM_SAMPLES-1))
                # Adjust these scaling factors as necessary cause Solidworks can't make up its mind when it comes
                # to units in drawings for some reason.
                # Don't forget that there is also scaling present on the Matlab Curvature script, may also need
                # to be adjusted.
                # TODO: Fix drawing scaling
                Xs.append(pt.real)
                Ys.append(pt.imag)
            
            Xs = np.array(Xs)
            Ys = np.array(Ys)

            tol = 10

            if min(Xs) < 0 and max(Xs) > 0:
                Xs += abs(min(Xs)) + tol
            
            if min(Ys) < 0 and max(Ys) > 0:
                Ys += abs(min(Ys)) + tol

            print(min(Xs), max(Xs))
            print(min(Ys), max(Ys))

            dX = max(Xs) - min(Xs)
            dY = max(Ys) - min(Ys)
            scaleX = TRUE_WIDTH / dX
            scaleY = TRUE_HEIGHT / dY

            Xs *= scaleX
            Ys *= scaleY

            for i in range(NUM_SAMPLES):
                spamwriter.writerow([Xs[i], Ys[i]])


    

