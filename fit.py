import pickle
from numpy.polynomial import Chebyshev
import numpy as np
import matplotlib.pyplot as plt

dbfile = open('lateral_x_y', 'rb')   
db = pickle.load(dbfile)

fit = Chebyshev.fit(db['x'], db['y'], deg=7)
fit = fit.linspace()

fig, ax = plt.subplots()
ax.plot(db['x'], db['y'], "o")
ax.plot(fit[0], fit[1])
plt.show()

dbfile.close()