import numpy as np
import math
import svgpathtools as svg

class Trajectory:
    def __init__(self,track_id,steps,r_min,r_max,scalar):
        self.steps = steps
        self.radii = []
        self.xpos = []
        self.ypos = []
        self.dL = []
        self.parse(track_id,scalar)
        self.calc(steps=steps)

    def parse(self,track_id,scalar):
        svg_read = svg.svg2paths(track_id)
        svg_path = svg_read[0]
        self.track = svg.Path(svg_path[0])
        if self.track.iscontinuous()==False:
            print('Discontinuous Track SVG')
        if self.track.isclosed()==False:
            print('Open Tack SVG')
        if len(svg.kinks(self.track))>1:
            print(svg.kinks(self.track))
            self.track=svg.smoothed_path(self.track)
            print('Warning: Non-differentiable Track SVG, Smoothing')
            # Untested
        while len(self.track)==1:
            self.track=self.track[0]
        self.track=self.track.scaled(scalar)

    def calc(self, steps):
        for i in range(steps):
            t=i/(steps)
            seg_index, sub_t = self.track.T2t(t)[0],self.track.T2t(t)[1],
            kappa=svg.path.segment_curvature(self.track[seg_index],t)
            if kappa==0:
                self.radii.append(0)
            else:
                r=(1/kappa)
                if r>100:
                    self.radii.append(100)
                else:
                    self.radii.append(1/kappa)
            self.xpos.append(svg.real(self.track.point(t)))
            self.ypos.append(svg.imag(self.track.point(t)))
            self.dL.append(self.track.length(t,t+(1/(steps))))
