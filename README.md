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
language.
