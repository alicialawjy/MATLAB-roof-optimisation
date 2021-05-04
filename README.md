![Header](https://github.com/alicialawjy/MATLAB-roof-optimisation/blob/main/Screenshots/Stadium_Design.png)
# Roof Optimisation
My team was tasked to design a stadium in collaboration with engineering firm, ARUP, as part of our 3rd Year Group Design Project.
The main and most critical feature of the stadium is its Arched Steel Roof Truss, spanning an incredible 105 metres and supported 20 metres above the ground. For the solution to work and be economically viable, the roof had to be highly optimised. 

<img src="https://github.com/alicialawjy/MATLAB-roof-optimisation/blob/main/Screenshots/Functional_Framing.png" width="500">
The roof is comprised of 4 main components:

1. Main Arch
2. Cantilever Truss
3. Longitudinal Bracing
4. Cross Bracing

As my team's programmer, I was responsible for scripting a solution that considered all potential configurations of the 4 components in order to assess and determine the most optimised outcome. This was done via Finite Element Analysis (FEM). For more information about FEM, please see my README.md for this project [here](https://github.com/alicialawjy/MATLAB-sway-structure)

To read about the roof design and optimisation in more detail, read **pg41-51 Roof Optimisation Report Extract.pdf**, in particular Section 6.3

## Scripts
#### BEAM.m
This script contains the object class used in the main code **Pin-Roll-Tie_Optimisation**. 

#### Pin-Roll-Tie_Optimisation.m
In this script, different configurations are iterated and analysed through FEM following the steps illustrated in the below flowchart:
<img src="https://github.com/alicialawjy/MATLAB-roof-optimisation/blob/main/Screenshots/Optimisation_Flowchart.png" width="600">
<img src="https://github.com/alicialawjy/MATLAB-roof-optimisation/blob/main/Screenshots/Optimisation_Process.png" width="600">

#### Scheme Analysis
We also had to present an evaluation of our final design in comparison to other plausible solutions, the following scripts were used to assess final costs and tonnage (ie. how much steel used) when designing the roof to different end conditions:
* **Pin-Pin_Scheme.m**: for pin-pin end connections
* **Pin-Roller_Scheme.m**: for pin-roller end connections
* **Pin-Roller-Tie_Scheme.m**: for pin-roller end connections using a tie (the final selected scheme)
<img src="https://github.com/alicialawjy/MATLAB-roof-optimisation/blob/main/Screenshots/end-connections.png" width="600">
