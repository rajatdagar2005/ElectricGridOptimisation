#  Electric Grid Design Program  

The **Electric Grid Design Program** is a comprehensive application for designing, managing, and optimizing electric power grids. It enables users to create a graph with substations and power lines while supporting various operations such as adding/removing components, calculating Minimum Spanning Trees (MSTs), applying budget constraints, simulating substation failures, and analyzing grid performance.  

This program provides visualization tools to understand the grid structure, ensures adherence to budget constraints, and determines the most cost-effective ways to connect substations using MST algorithms like **Prim’s** and **Kruskal’s**.  
  

###  Graph Creation & Management  
- **Create and Load Graphs** – Create a new power grid or load an existing one from a file.  
- **Add/Remove Substations and Power Lines** – Dynamically add or remove substations and power lines.  
- **Display the Graph** – Visualize the current grid structure with substations and their connections.  

###  Optimization  
- **Prim’s MST Algorithm** – Compute the **Minimum Spanning Tree (MST)** using **Prim’s algorithm** for cost-efficient grid connectivity.  
- **Kruskal’s MST Algorithm** – Calculate MST using **Kruskal’s algorithm** as an alternative optimization method.  
- **Apply Budget Constraints** – Ensure the grid remains within a specified budget while maintaining connectivity.  

###  Analysis  
- **Connectivity Check** – Verify if a power line exists between two substations.  
- **Shortest Path Calculation** – Use **Dijkstra’s algorithm** to determine the shortest path between substations.  
- **Power Line Count** – Count the number of power lines connected to a specific substation.  
- **Distance Calculation** – Compute the minimum cost distance between two substations.  

###  Simulation  
- **Substation Failure Simulation** – Simulate the failure of a substation and analyze its impact on the grid by removing its connections.  

###  File Operations  
- **Save the Grid to File** – Save the current state of the grid for future use.  
- **Load the Grid from File** – Load a previously saved grid configuration.  

##  Usage  
Simply run the program to start designing and optimizing electric grids. Choose between different algorithms and features for an interactive experience.  

##  Technologies Used  
- **C++** for core logic and graph algorithms  
- **Graph Theory** for optimization and analysis  
- **File Handling** for saving and loading grid states  
