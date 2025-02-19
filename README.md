Electric Grid Design Program
The Electric Grid Design Program is a comprehensive application that allows users to design, manage, and optimize electric power grids. The program enables the creation of a graph with substations and power lines, and supports operations such as adding/removing components, calculating Minimum Spanning Trees (MSTs), applying budget constraints, simulating substation failures, and analyzing grid performance.

This program helps visualize the structure of a power grid, ensure that it meets budget constraints, and calculate the most cost-effective ways to connect substations using MST algorithms (Prim's and Kruskal's).

Features
Graph Creation & Management:
Create and Load Graphs: Users can create a new graph for the power grid or load an existing grid from a file.
Add/Remove Substations and Power Lines: You can add new substations to the grid or remove existing ones. Power lines can be added or removed between any two substations.
Display the Graph: View the current graph showing the substations and their connections through power lines.
Optimization:
Prim's MST Algorithm: Calculate the Minimum Spanning Tree (MST) using Prim's algorithm to find the minimum cost for connecting all substations.
Kruskal's MST Algorithm: Compute the MST using Kruskal's algorithm, providing another method for cost optimization.
Apply Budget Constraints: Ensure that the total cost of selected power lines does not exceed the available budget. The program will optimize the grid if necessary.
Analysis:
Connectivity Check: Check if a power line exists between any two substations.
Shortest Path Calculation: Calculate the shortest path between two substations using Dijkstra's algorithm.
Power Line Count: Determine the number of power lines connected to a specific substation.
Distance Calculation: Calculate the minimum distance (in terms of power line cost) between two substations.
Simulation:
Substation Failure Simulation: Simulate the failure of a substation and observe its impact on the grid, removing any power lines connected to the failed substation.
File Operations:
Save the Grid to File: Save the current state of the grid to a file for later use.
Load the Grid from File: Load a previously saved grid from a file.
