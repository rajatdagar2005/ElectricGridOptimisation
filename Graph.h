#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <limits>
#include <unordered_map>

using namespace std;

// Structure to represent a Substation
struct Substation {
    int id;              // Unique identifier for the substation
    double demand;       // Power demand of the substation
    double production;   // Power production capacity of the substation

    // Constructor to initialize Substation with id, demand, and production values
    Substation(int id, double demand = 0.0, double production = 0.0) : id(id), demand(demand), production(production) {}
};

// Structure to represent a PowerLine between two substations
struct PowerLine {
    int u, v;            // The two substations connected by the power line
    double cost;         // The cost of constructing the power line

    // Constructor to initialize a power line with two substations and a cost
    PowerLine(int u, int v, double cost) : u(u), v(v), cost(cost) {}

    // Comparator to sort power lines based on cost (used in Kruskal's algorithm)
    bool operator<(const PowerLine &e) const { return cost < e.cost; }
};

// Class to represent a graph of substations and power lines
class Graph {
public:
    int n, m;                  // Number of substations (n) and power lines (m)
    double budget;              // Budget for constructing power lines
    double TotalCost;          // Total cost of selected power lines (MST)
    vector<Substation> Substations;  // List of all substations in the graph
    vector<PowerLine> PowerLines;    // List of all power lines in the graph
    vector<PowerLine> mstPowerLines; // List of power lines in the MST
    unordered_map<int, vector<PowerLine>> adjacencyList;  // Adjacency list for the graph representation
    bool mstCalculated;         // Flag to check if MST is calculated

    // Constructor to initialize the graph with n substations and m power lines
    Graph(int n, int m, double budget);

    // Function to add a power line to the graph
    void addPowerLine(int u, int v, double cost);

    // Function to validate the graph's input dimensions (number of substations and power lines)
    bool validateInputs(int n, int m);

    // Function to compute the Minimum Spanning Tree (MST) using Prim's algorithm
    void primMST();

    // Function to compute the Minimum Spanning Tree (MST) using Kruskal's algorithm
    void kruskalMST();

    // Function to calculate multiple MSTs based on varying conditions
    void calculateMultipleMSTs();

    // Function to calculate the minimum cost of a given MST
    double calculateMinimumCost(const vector<PowerLine>& mst);

    // Function to update the graph after a change in power line configuration
    void updateGraphAfterPowerLineChange();

    // Function to check the connectivity between two substations (u and v)
    void checkPowerLineConnectivity(int u, int v);

    // Function to simulate the failure of a substation
    void simulateSubstationFailure(int Substation);

    // Function to apply budget constraints when selecting power lines
    void applyBudgetConstraints();

    // Function to add a new substation to the graph
    void addSubstation(int id, double demand, double production);

    // Function to remove a power line from the graph
    void removePowerLine(int u, int v);

    // Function to display the graph with its current configuration
    void displayGraph();

    // Function to save the current grid configuration to a file
    void saveGridToFile(const string& filename);

    // Function to load the grid configuration from a file
    void loadGridFromFile(const string& filename);

    // Function to update the budget available for constructing power lines
    void updateBudget(double budget);

    // Function to change the cost of a power line (for simulation purposes)
    void changePowerLinecost();

    // Function to count the total number of power lines in the graph
    void countPowerLines();

    // Depth-First Search (DFS) function to explore connected components in the graph
    void dfs(int Substation, vector<bool>& visited);

    // Function to check if the graph is fully connected (all substations are reachable)
    bool isConnected();

    // Function to recalculate the MST when the graph changes (e.g., after a power line removal)
    void recalculateMST();

    // Function to adjust the total power flow in the graph after changes
    void adjustTotalPowerFlow();

    // Function to calculate the minimum distance between substations (for optimization)
    void minimumdistancebetweensubstations();
};

#endif
