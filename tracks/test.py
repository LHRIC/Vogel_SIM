import svg_tracks
import matplotlib.pyplot as plt
from matplotlib import cm
# t1=svg_tracks.Trajectory(track_id='2023_IC_michigan_endurance.svg',steps=1000,r_min=0,r_max=100,scalar=0.1,track_length=2000)
t2=svg_tracks.Trajectory(track_id='2019_IC_michigan_endurance.svg',\
                         steps=1000,track_length=2000,ggv_detail=100)

fig3, ax3 = plt.subplots()
sc = ax3.scatter(t2.xpos, t2.ypos, c=t2.radii)
ax3.axis("equal")
cbar = plt.colorbar(sc)
cbar.set_label("Radii")

fig5, ax5 = plt.subplots()
sc = ax5.scatter(list(range(len(t2.r_set))),t2.r_set)
plt.xlabel("Num")
plt.ylabel("Radii")
cbar = plt.colorbar(sc)
cbar.set_label("Radii")
plt.show()

# fig4, ax4 = plt.subplots()
# sc = ax4.scatter(t1.xpos, t1.ypos, c=t1.radii)
# ax4.axis("equal")
# cbar = plt.colorbar(sc)
# cbar.set_label("Radii")
# plt.show()