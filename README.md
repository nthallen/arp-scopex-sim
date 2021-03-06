# arp-scopex-sim
This is a simple kinematic simulation of a propeller-driven research balloon
platform written in C++ using
[Open Dynamics Engine](http://ode.org/wiki/index.php?title=Manual).
We will use this model to anticipate the stability of the platform and to tune
our control algorithms.

The model is capable of producing 3D graphics under X-Windows, but we will
probably not focus much effort on that, as the really interesting data can
be obtained from the log files and analyzed in Matlab.

## Basic Operation

The model has a very simple organization:

  - Initialization
  - Main Loop
  
During the initialization, all parameters are assigned default values
currently based on our best estimates when the model was originally
created. After the default values are assigned, any commands listed
at time zero in the specified script are executed. These can change
the values of selected parameters.

In the main loop, the kinematic simulation is stepped, and commands are
executed from the specified script at the appropriate time. The simulation
is stopped when the Quit command is executed in the script.

## Script Syntax

The script syntax is very simple. Each can be either a comment
(beginning with an octothorpe '#') or an instruction. Instruction lines
consist of an unsigned time delta value in seconds followed by a command.

Currently support commands include:

  - Noop
    - does nothing
  - Set \<parameter> \<value>
    - sets the specified parameter to the given value
  - Adjust \<parameter> \<value>
    - adds the value to the specified paramter
  - Quit
    - terminates the simulation

At present (3/22/20), the model is far from being in a completed state.
Many more (or all) parameters could easily be added to the scripting
language. Parameters are added to the scripting language in
SCoPEx::Init().

## Matlab Tools

The files in the Matlab directory provide various examples of how the
data generated by scopex-sim can be explored.

  - scopex.m:
    - a semi-interactive script that works with the 'doit' script
      to exercise the trajectory control algorithm.

  - scopex_load.m:
    - Loads scopex-sim output data into a useful data structure.
  
  - scopex_anal.m
    - Loads data and produces plots of trajectorys, velocity error
      and speed

  - GetBodyData.m
  - GetJointData.m
  - GetSimVar.m:
    - Functions used by scopex_load and other scripts to organize
      scopex-sim output.

  - PlotBodyData.m
  - PlotJointData.m
  - PlotSimVar.m
    - Functions for displaying system data.

  - write_scp.m
    - Function for programmatically creating scopex-sim scripts.

  - shape.m:
    - A model of balloon shape that might be useful for estimating
      horizontal area for drag calculations
      
## Building
  - Make sure you have cmake installed
  - Download, build and install the [ode library](http://ode.org/wiki/index.php?title=Manual)
    - I recommend building with cmake
    - Let me know if you encounter problems
  - mkdir scopex-sim
  - cd scopex-sim
  - git clone https://github.com/nthallen/arp-scopex-sim.git git
  - mkdir build
  - cd build
  - cmake ../git
  - make
  - make install
