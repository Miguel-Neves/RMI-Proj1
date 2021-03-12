# RMI-Proj1
Pathfinder solver using the CiberRato simulation environment.

---

#### How to run 

Build the project: 

`make all`

Run the viewer and simulator: 

`./startChallenge1_2` 

In the folder pClient, run the agent script (in the target argument, multiple target cells are separated by "\_").
For example, to run the default labyrinth, starting in cell (2,3), with targets in cells (8,6) and (12,3):

`./s1.sh -m ../Labs/PathFinder/pathFinder2Beacons_lab.xml -s 2,3 -t 8,6_12,3`