import numpy as np
from numpy.polynomial import Chebyshev, Polynomial
from numpy.polynomial.polynomial import polyval
import math
from csaps import csaps as csaps_fun
import matplotlib.pyplot as plt


def csaps(x, y):
    fit_x = np.linspace(
            x[0], x[-1], 150
        )
    fit_y = csaps_fun(x, y, fit_x, smooth=0.9)

    return FitFunction("CSAPS", x, y, fit_x, fit_y)

def polyfit(x, y, degree):
    p = Polynomial.fit(x, y, deg=degree)
    xx, yy = p.linspace()
    return FitFunction("POLYNOMIAL", x, y, xx, yy) 


class FitFunction():
    def __init__(self, fit_type, x, y, fit_x, fit_y):
        self.__type = fit_type
        self.__x = x
        self.__y = y
        self.__fit_x = fit_x
        self.__fit_y = fit_y
    
    def evaluate(self, x_val):
        return np.interp(x_val, self.__fit_x, self.__fit_y)
    
    def plot(self, show=True, save=""):
        fig, ax = plt.subplots()
        ax.plot(self.__x, self.__y, "o")
        ax.plot(self.__fit_x, self.__fit_y)
        plt.grid()
        if show:
            plt.show()
        if save != "":
            plt.savefig(save)
    
    def __str__(self) -> str:
        s = self.__type + "\n"

if __name__ == "__main__":
    x = np.array(list(range(0, 30)))
    y = np.array([3,4.6
                    ,8.8
                    ,15.6
                    ,25
                    ,37
                    ,51.6
                    ,68.8
                    ,88.6
                    ,111
                    ,136
                    ,163.6
                    ,193.8
                    ,226.6
                    ,262
                    ,300
                    ,340.6
                    ,383.8
                    ,429.6
                    ,478
                    ,529
                    ,582.6
                    ,638.8
                    ,697.6
                    ,759
                    ,823
                    ,889.6
                    ,958.8
                    ,1030.6
                    ,1105])
    
    a = polyfit(x, y, 2)
    a.plot()
    print(a.evaluate(4))
        
