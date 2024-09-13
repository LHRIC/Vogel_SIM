import svg_tracks
import matplotlib.pyplot as plt
from matplotlib import cm
t=svg_tracks.Trajectory(track_id='tracktest.svg',steps=1000,r_min=0,r_max=100,scalar=0.1)

fig3, ax3 = plt.subplots()
sc = ax3.scatter(t.xpos, t.ypos, c=t.radii)
ax3.axis("equal")
cbar = plt.colorbar(sc)
cbar.set_label("Radii")

plt.show()