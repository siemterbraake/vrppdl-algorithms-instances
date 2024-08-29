# Vehicle Routing Problem with Pickups, Deliveries, and Linehauls (VRPPDL)

*Casper Bazelmans, Siem ter Braake, Albert Schrotenboer, Rolf van Lieshout, Tom van Woensel*

This repository contains the complimentary instances and algorithms for the *Vehicle Routing Problem with Pickups, Deliveries, and Linehauls* paper. 

## Instances
The instances are split by size. The small folder includes the synthetic instances used to test the column generation algorithms. The medium and large folders include the real-world instances. All solutions are stored in `.json` format. The small solutions include a relaxed solution as well as an integer solution. These are either exact or heuristic. The medium and large solutions include only heuristic integer solutions obtained using the ALNS.

The first line of each instance includes the name, the number of vehicle types, the number of vehicle type depot pairs, the number of linehauls, and the number of nodes. Then, the parameters for each vehicle type, the number of vehicle types for each depot, the paramers for each linehaul and the parameters for each node are listed in a tab-seperated format. The header for each table describes the order of the parameters.

All distances between nodes are determined by the haversine distance between node coordinates. The unit for the vehicle speed is km/h. The time windows, service time and vehicle time constraints are all in seconds. All quantities are of the same quantity unit. The cost per request for using a linehaul is in the unit cost/unit quanity. The node type -1 stands for depot, 0 stands for pickup order, and 1 stands for delivery order.  

## Algorithms
The ALNS column generation implementation is shared in the `alns-column-generation` folder and the ALNS algorithm for large-scale instances is shared in the `large-scale-alns` folder. Both folders include a CMake file to build the application. The column generation application uses the Gurobi MIP solver. Therefore, a Gurobi installation and license is required to build this application. Gurobi is free for academics and a license can be obtained [here](https://www.gurobi.com/academia/academic-program-and-licenses/). From the root directory of the program use the following commands to build the application:
```
cd build
cmake ..
cmake --build .
```
Both programs have `settings.txt` file that includes the parameters of the program. 



