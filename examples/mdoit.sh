#! /bin/bash
export PATH=/usr/local/bin:/usr/bin:$PATH
# ./scopex-sim -f SingleStep.scp -lscopex-sim.log
./scopex-sim -f Tune.scp -lscopex-sim.log
# 
#./scopex-sim -f Straight.scp -lscopex-sim.log
# Check buoyancy:
#./scopex-sim -f Climb.scp -lscopex-sim.log
