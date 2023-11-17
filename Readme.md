# LHRC: QSS Laptime Simulation

# Requirements
 - Python 3.x
 - MATLAB 2022 R2 (Optional)

# Installation and Environment Setup
In the root directory, create a virtual environment to store python libraries
```
$ python -m venv env
```

Activate the virtual environment. On windows
```
$ . ./env/Scripts/Activate
```
On UNIX/MacOS
```
$ . ./env/bin/activate
```

Install the required libraries into the virtual environment
```
$ pip install -r ./requirements.txt
```

You're now ready to run the lapsim!

# Configuration and Execution

`main.py` handles the setup of the Lapsim Engine, loads the default AERO, DYN, and PTN models, and loads the default 2017 Lincoln Endurance Track. This is all you need to start running parameter sweeps.
```
def main():
    aero_model = models.AERO()
    dyn_model = models.DYN()
    ptn_model = models.PTN()

    simulator = Engine(trajectory="./trajectory/17_lincoln_endurance_track_highres.xls")
```

The Lapsim Engine class implements a `.sweep()` method. This method takes in two arguments, `num_steps` and `**kwargs`. `num_steps` is used to determine the resolution of the sweep domain and **kwargs is used to determine the upper and lower bounds of the sweep_interval.

For instance, if you wanted to request a sweep of the vehicle parameter DYN.total_weight, from 1200 to 2700 N, the argument would be shaped as follows.

``` 
simulator.sweep(num_steps=30, total_weight=(1200, 2700))
```