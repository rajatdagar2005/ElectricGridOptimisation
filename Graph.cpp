#include "Graph.h"
#include <limits>
#include <queue>
#include <iomanip>
#include <unordered_set>
#include <fstream>
#include<limits.h>
#include<algorithm>
#include<stack>

// Constructor to initialize the Graph with substations, power lines, and budget
Graph::Graph(int n, int m, double budget) : n(n), m(m), budget(budget), mstCalculated(false) 
{
    if (!validateInputs(n, m)) 
    {
        // If inputs are invalid, display an error and terminate the program
        cout << "Invalid inputs provided. Power Grid cannot be created.\n";
        exit(EXIT_FAILURE);
    }
}

// Function to validate the inputs for the Graph
bool Graph::validateInputs(int n, int m) 
{
    // Valid if substations and power lines are greater than 0 and sufficient to form a connected graph
    return n > 0 && m > 0 && m >= n - 1;
}

// Function to add a new power line between two substations with a specified cost
void Graph::addPowerLine(int u, int v, double cost) 
{
    // Add the power line to the adjacency list for both substations
    adjacencyList[u].push_back(PowerLine(u, v, cost));
    adjacencyList[v].push_back(PowerLine(v, u, cost));

    // Store the power line in the list of all power lines
    PowerLines.push_back(PowerLine(u, v, cost));

    // Print confirmation message
    cout << "PowerLine (" << u << ", " << v << ") added with cost: " << cost << "\n";

    // Update the graph after adding a new power line
    updateGraphAfterPowerLineChange();
}

// Function to handle updates after a power line change
void Graph::updateGraphAfterPowerLineChange() 
{
    // Notify that the power grid has been updated
    cout << "Power Grid updated after PowerLine change.\n";

    // Check if the power grid is connected
    if (isConnected()) 
    {
        cout << "The Power Grid is connected.\n";
    } 
    else 
    {
        cout << "Warning: The Power Grid is disconnected!\n";
    }

    // Recalculate the minimum spanning tree (MST) for the updated graph
    recalculateMST();

    // Adjust the total power flow based on the updated power lines
    adjustTotalPowerFlow();
}

// Function to perform a depth-first search (DFS) to traverse the graph
void Graph::dfs(int Substation, vector<bool>& visited) 
{
    // Mark the current substation as visited
    visited[Substation] = true;

    // Recursively visit all connected substations that have not been visited
    for (const PowerLine& PowerLine : adjacencyList[Substation]) 
    {
        if (!visited[PowerLine.v]) 
        {
            dfs(PowerLine.v, visited);
        }
    }
}

// Check if the Graph is connected
bool Graph::isConnected() 
{
    vector<bool> visited(n, false);  
    // Create a visited array to track visited substations
    dfs(0, visited);  
    // Start DFS from Substation 0 (assuming Substation 0 exists in the Graph)
    
    // If any Substation is unvisited, the Graph is disconnected
    for (bool SubstationVisited : visited) 
    {
        if (!SubstationVisited) return false;  
        // Return false if a Substation is not visited
    }
    return true;  
    // Return true if all Substations are visited
}

// Recalculate Minimum Spanning Tree (MST) using Prim's Algorithm
void Graph::recalculateMST() 
{
    cout << "Recalculating Minimum cost for the power grid...\n";

    // Priority queue to pick the PowerLine with the minimum cost
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    vector<double> key(n, INT_MAX);   
    // Store minimum PowerLine cost for each Substation
    vector<bool> inMST(n, false);    
    // Track Substations included in MST
    vector<int> parent(n, -1);       
    // Store parent of each Substation in MST

    key[0] = 0;  
    // Start from Substation 0
    pq.push({0, 0});  
    // Push Substation 0 with cost 0 to the priority queue

    while (!pq.empty()) 
    {
        int u = pq.top().second;  
        // Extract the Substation with the minimum cost
        pq.pop();

        inMST[u] = true;  
        // Mark the Substation as included in MST

        // Explore all adjacent Substations
        for (const PowerLine& e : adjacencyList[u]) 
        {
            int v = e.v;       
            // Adjacent Substation
            double cost = e.cost;  
            // Cost of the PowerLine

            // If v is not in MST and PowerLine cost is smaller, update the key and parent
            if (!inMST[v] && cost < key[v]) 
            {
                key[v] = cost;         
                // Update the minimum cost for Substation v
                pq.push({key[v], v});  
                // Push the updated cost and Substation to the priority queue
                parent[v] = u;         
                // Update the parent of Substation v
            }
        }
    }

    // Clear the previous MST PowerLines
    mstPowerLines.clear();

    // Print the MST PowerLines and add them to mstPowerLines
    cout << "Power lines in Power Grid:\n";
    double MinimumCost = 0;  
    // Variable to store the total MST cost
    for (int i = 1; i < n; ++i) 
    {
        if (parent[i] != -1) 
        {  
            // If Substation has a valid parent
            cout << parent[i] << " - " << i << " with cost: " << key[i] << "\n";
            mstPowerLines.push_back(PowerLine(parent[i], i, key[i]));  
            // Save PowerLines to mstPowerLines vector
            MinimumCost += key[i];  
            // Add the cost to the total MST cost
        }
    }

    // Display the final MST cost
    cout << "\nMinimum cost: " << MinimumCost << "\n";
}


// Adjust power flow based on the PowerLine costs (as capacities)
void Graph::adjustTotalPowerFlow() 
{
    cout << "Adjusting Total power flow based on PowerLine capacities...\n";

    double newFlow = 0.0;
    // Sum the costs of all PowerLines to represent total capacity or power flow
    for (const PowerLine& e : PowerLines) 
    {
        newFlow += e.cost;  
        // Accumulate the cost of each PowerLine
    }

    TotalCost = newFlow;  
    // Update the total power flow
    cout << "Total power flow adjusted to: " << TotalCost << "\n";
}

// Display the current state of the graph
void Graph::displayGraph() 
{
    if (adjacencyList.empty()) {
        std::cout << "No substations or power lines to display.\n";
        return;
    }

    std::cout << "--------------------- Power Grid Connections ---------------------\n";

    for (const auto& Substation : adjacencyList) 
    {
        // Display the substation node
        std::cout << "Substation " << Substation.first << " -> ";

        // Display the linked list of connected PowerLines
        if (Substation.second.empty()) {
            std::cout << "No connections.\n";
        } else {
            bool first = true;
            for (const auto& PowerLine : Substation.second) 
            {
                if (!first) {
                    std::cout << " -> "; // Representing the linked list arrow
                }
                std::cout << "(Substation " << PowerLine.v << ", cost: " << PowerLine.cost << ")";
                first = false;
            }
            std::cout << "\n";
        }
    }
    
    std::cout << "-------------------------------------------------------------------\n";
}


// Generate Minimum Spanning Tree (MST) using Prim's Algorithm
void Graph::primMST() 
{
    if (n <= 1 || m < n - 1) 
    {
        cout << "Not enough Substations or PowerLines to form an MST.\n";
        return;   
        // Exit if conditions for MST are not met
    }

    mstPowerLines.clear();  
    // Clear the previous MST
    double MinimumCost = 0; 
    // Initialize the total cost of MST
    vector<bool> inMST(n, false); 
    // Track Substations included in MST
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> minHeap; 
    // Min-heap for edge selection

    inMST[0] = true;  
    // Start MST from Substation 0
    for (const auto& PowerLine : adjacencyList[0]) 
    {
        minHeap.push(std::make_pair(PowerLine.cost, PowerLine.v));  
        // Add initial edges to the heap
    }

    while (!minHeap.empty() && mstPowerLines.size() < n - 1) 
    {
        double cost = minHeap.top().first;
        int v = minHeap.top().second;

        minHeap.pop();

        if (inMST[v]) 
        {
            continue;  
            // Skip if the Substation is already in MST
        }

        inMST[v] = true;  
        // Mark the Substation as included in MST
        for (const auto& PowerLine : adjacencyList[v]) 
        {
            if (!inMST[PowerLine.v]) 
            {
                minHeap.push(std::make_pair(PowerLine.cost, PowerLine.v));  
                // Add edges of the current Substation
            }
        }

        mstPowerLines.push_back({v, minHeap.top().second, cost});  
        // Add the selected PowerLine to MST
        MinimumCost += cost;  
        // Accumulate the cost of the PowerLine
    }

    if (mstPowerLines.size() == n - 1) 
    {
        cout << "Prim's MST completed successfully.\n";
        cout << "Total MST cost: " << MinimumCost << "\n";
        for (const auto& PowerLine : mstPowerLines) 
        {
            cout << "PowerLine: " << PowerLine.u << " - " << PowerLine.v << " | cost: " << PowerLine.cost << "\n";
        }
    } 
    else 
    {
        cout << "Unable to create MST with the given PowerLines.\n";
    }
}

// Generate Minimum Spanning Tree (MST) using Kruskal's Algorithm
void Graph::kruskalMST() 
{
    struct DisjointSet 
    {
        vector<int> parent, rank;
        DisjointSet(int n) : parent(n), rank(n, 0) 
        {
            for (int i = 0; i < n; i++) parent[i] = i;
            // Initialize each Substation as its own parent
        }

        int find(int u) 
        {
            if (u != parent[u]) parent[u] = find(parent[u]); 
             // Path compression
            return parent[u];
        }

        void unite(int u, int v) 
        {
            u = find(u), v = find(v);  
            // Find the roots of both nodes
            if (u != v) {
                if (rank[u] > rank[v]) parent[v] = u; 
                // Union by rank
                else if (rank[u] < rank[v]) parent[u] = v;
                else 
                {
                    parent[v] = u;
                    rank[u]++;
                }
            }
        }
    };

    sort(PowerLines.begin(), PowerLines.end());  
    // Sort PowerLines by cost
    DisjointSet ds(n);  
    // Initialize the disjoint set
    mstPowerLines.clear();  
    // Clear the previous MST
    double MinimumCost = 0; 
     // Initialize the total cost of MST

    for (const auto& PowerLine : PowerLines)  
    {
        if (ds.find(PowerLine.u) != ds.find(PowerLine.v)) 
        {
            mstPowerLines.push_back(PowerLine);  
            // Add the PowerLine to MST
            MinimumCost += PowerLine.cost;  
            // Accumulate the cost
            ds.unite(PowerLine.u, PowerLine.v);  
            // Merge the sets
            if (mstPowerLines.size() == n - 1) break;  
            // Stop when MST is complete
        }
    }

    if (mstPowerLines.size() == n - 1) 
    {
        cout << "Kruskal's MST completed successfully.\n";
        cout << "Total MST cost: " << MinimumCost << "\n";
        for (const auto& PowerLine : mstPowerLines) 
        {
            cout << "PowerLine: " << PowerLine.u << " - " << PowerLine.v << " | cost: " << PowerLine.cost << "\n";
        }
    } else {
        cout << "Unable to create MST with the given PowerLines.\n";
    }
}

// Calculate and compare MSTs using different algorithms
void Graph::calculateMultipleMSTs() 
{
    // Display the MST calculated using Prim's algorithm
    cout << "Prim's algorithm:" << endl;
    primMST();

    // Display the MST calculated using Kruskal's algorithm
    cout << "Kruskal's algorithm:" << endl;
    kruskalMST();

    cout << "Multiple MSTs calculation is completed and compared.\n";
}

// Simulate the failure of a specific Substation in the network
void Graph::simulateSubstationFailure(int Substation) 
{
    cout << "Simulating failure at Substation: " << Substation << "\n";

    // Remove all PowerLines connected to the failed Substation
    for (auto& PowerLineList : adjacencyList) 
    {
        // Remove all PowerLines that are connected to the failed substation
        PowerLineList.second.erase(remove_if(PowerLineList.second.begin(), PowerLineList.second.end(), [Substation](PowerLine& e) 
        {
            return e.u == Substation || e.v == Substation; 
        }), PowerLineList.second.end());
    }

    // Now also remove the PowerLines from the main PowerLines vector
    PowerLines.erase(remove_if(PowerLines.begin(), PowerLines.end(), [Substation](PowerLine& e) 
    {
        return e.u == Substation || e.v == Substation; 
    }), PowerLines.end());

    // Optionally, you could also remove the substation itself from the Substations list or mark it as failed
    // For this example, we're keeping the substation, but you can choose to remove it or flag it as inactive
    cout << "Substation " << Substation << " removed from the network for failure simulation.\n";
}


// Calculate the total cost of an MST
double Graph::calculateMinimumCost(const vector<PowerLine>& mst) 
{
    double MinimumCost = 0;

    // Sum the costs of all PowerLines in the MST
    for (const auto& PowerLine : mst) 
    {
        MinimumCost += PowerLine.cost;
    }

    return MinimumCost;
}

// Apply budget constraints to the MST and optimize if necessary
void Graph::applyBudgetConstraints() 
{
    cout << "Applying budget constraints. Budget: " << budget << "\n";

    // Calculate the cost of the current MST
    double MinimumCost = calculateMinimumCost(mstPowerLines);

    if (MinimumCost > budget) 
    {
        cout << "MST exceeds budget; optimizing...\n";

        // Sort PowerLines by descending cost to optimize MST by removing the most expensive PowerLines
        sort(mstPowerLines.begin(), mstPowerLines.end(), [](const PowerLine& a, const PowerLine& b) 
        {
            return a.cost > b.cost;
        });

        // Remove PowerLines until the MST cost is within the budget
        while (MinimumCost > budget && !mstPowerLines.empty()) 
        {
            MinimumCost -= mstPowerLines.back().cost; // Subtract the cost of the removed PowerLine
            mstPowerLines.pop_back(); // Remove the most expensive PowerLine
        }

        cout << "Optimized MST cost: " << MinimumCost << "\n";
    } 
    else 
    {
        cout << "MST is within budget.\n";
    }
}

// Save the current state of the Power Grid to a file
void Graph::saveGridToFile(const string& filename) 
{
    ofstream file(filename);

    if (file.is_open()) 
    {
        // Write all PowerLines to the file
        for (const auto& PowerLine : PowerLines) 
        {
            file << PowerLine.u << " " << PowerLine.v << " " << PowerLine.cost << "\n";
        }
        file.close(); // Close the file after writing
        cout << "Graph saved to " << filename << "\n";
    } 
    
    else 
    {
        cout << "Error: Unable to open file for saving.\n";
    }
}

// Load a Power Grid from a file
void Graph::loadGridFromFile(const string& filename) {
    ifstream infile(filename); // Try to open the file
    if (!infile.is_open()) {   // Check if the file is successfully opened
        cerr << "Error: File \"" << filename << "\" does not exist or cannot be opened.\n";
        return;  // Exit the function if file opening fails
    }

    int u, v;
    double weight;

    // Clear existing graph data to load new data
    adjacencyList.clear();
    PowerLines.clear();
    Substations.clear();

    cout << "Loading graph from file \"" << filename << "\"...\n";

    while (infile >> u >> v >> weight) {  // Read edges from the file
        if (u < 0 || v < 0 || weight < 0) {
            cerr << "Invalid data in file: (" << u << ", " << v << ", " << weight << "). Skipping.\n";
            continue;
        }
        addPowerLine(u, v, weight);
    }

    infile.close();  // Close the file after reading

    cout << "Graph successfully loaded with " << PowerLines.size() << " power lines.\n";
}



// AddSubstation: Adds a new Substation (substation) to the Graph
void Graph::addSubstation(int id, double demand, double production) 
{
    Substations.push_back(Substation(id, demand, production));  // Add new Substation to Substations list
    n++;  // Increment the number of Substations
    adjacencyList[id] = {};  // Initialize the adjacency list for the new Substation
    cout << "Substation " << id << " added successfully.\n";  // Confirmation message
}

// RemovePowerLine: Removes a specific PowerLine from the Graph
void Graph::removePowerLine(int u, int v) 
{
    // Remove PowerLine from Substation u's adjacency list
    adjacencyList[u].erase(remove_if(adjacencyList[u].begin(), adjacencyList[u].end(),
                                     [v](PowerLine& e) { return e.v == v; }),
                           adjacencyList[u].end());
    // Remove PowerLine from Substation v's adjacency list
    adjacencyList[v].erase(remove_if(adjacencyList[v].begin(), adjacencyList[v].end(),
                                     [u](PowerLine& e) { return e.u == u; }),
                           adjacencyList[v].end());

    // Remove PowerLine from the PowerLines vector
    PowerLines.erase(remove_if(PowerLines.begin(), PowerLines.end(),
                           [u, v](PowerLine& e) { return (e.u == u && e.v == v) || (e.u == v && e.v == u); }),
                PowerLines.end());
    cout << "PowerLine (" << u << ", " << v << ") removed.\n";  // Confirmation message
    updateGraphAfterPowerLineChange();  // Update the graph after PowerLine removal
}

// CheckPowerLineConnectivity: Checks if there is a PowerLine between two Substations
void Graph::checkPowerLineConnectivity(int u, int v) 
{
    bool connected = false;  // Flag to track connectivity

    // Loop through the adjacency list of Substation u
    for (const auto& PowerLine : adjacencyList[u]) 
    {
        if (PowerLine.v == v) 
        {  // Check if PowerLine exists between u and v
            connected = true;
            break;
        }
    }

    // Output whether the PowerLine exists between the substations
    if (connected) 
    {
        cout << "PowerLine (" << u << ", " << v << ") exists in the Graph.\n";
    } 
    
    else 
    {
        cout << "No PowerLine between Substation " << u << " and Substation " << v << ".\n";
    }
}


void Graph::changePowerLinecost() 
{
    int u, v;
    double newcost;

    // Ask the user for the PowerLine (u, v) and the new cost
    cout << "Enter the first Substation of the PowerLine (u): ";
    cin >> u;
    cout << "Enter the second Substation of the PowerLine (v): ";
    cin >> v;
    cout << "Enter the new cost for the PowerLine: ";
    cin >> newcost;

    // Iterate through the PowerLines vector to find the PowerLine
    bool PowerLineFound = false;
    for (auto& PowerLine : PowerLines) 
    {
        if ((PowerLine.u == u && PowerLine.v == v) || (PowerLine.u == v && PowerLine.v == u)) 
        {
            PowerLine.cost = newcost;  // Update the cost of the found PowerLine
            PowerLineFound = true;
            cout << "PowerLine cost between " << u << " and " << v << " updated to " << newcost << endl;

            // Optionally, if the PowerLine exists in the adjacency list, update the corresponding adjacency list as well
            for (auto& adjPowerLine : adjacencyList[u]) 
            {
                if (adjPowerLine.u == u && adjPowerLine.v == v || adjPowerLine.u == v && adjPowerLine.v == u) 
                {
                    adjPowerLine.cost = newcost;
                }
            }
            for (auto& adjPowerLine : adjacencyList[v]) 
            {
                if (adjPowerLine.u == u && adjPowerLine.v == v || adjPowerLine.u == v && adjPowerLine.v == u) 
                {
                    adjPowerLine.cost = newcost;
                }
            }
            break;  // Exit after updating the PowerLine
        }
    }

    if (!PowerLineFound) 
    {
        cout << "PowerLine not found between " << u << " and " << v << endl;  // PowerLine not found
    }
}



void Graph::countPowerLines() 
{
    int Substation;
    cout << "Enter the Substation whose PowerLine count you want to know: ";
    cin >> Substation;

    // Check if the Substation exists in the adjacency list
    if (adjacencyList.find(Substation) == adjacencyList.end()) 
    {
        cout << "Substation " << Substation << " not found in the Graph.\n";
    } 
    
    else 
    {
        // Print the number of PowerLines connected to the Substation
        cout << "Substation " << Substation << " is connected to " << adjacencyList[Substation].size() << " PowerLines.\n";
    }
}

// SetBudget: Sets the budget for the MST
void Graph::updateBudget(double newBudget) 
{
    budget = newBudget;
    cout << "Budget set to: " << budget << "\n";
}

void Graph:: minimumdistancebetweensubstations() 
{
        int start, end;
        // Taking input for start and end substations within the function
        cout << "Enter the start substation ID: ";
        cin >> start;
        cout << "Enter the end substation ID: ";
        cin >> end;

        // Dijkstra's algorithm initialization
        vector<double> dist(n + 1, numeric_limits<double>::infinity());
        vector<int> prev(n + 1, -1); // To track the path
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

        dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            if (u == end) break; // Found the shortest path to the destination

            for (const auto& edge : adjacencyList[u]) {
                int v = (edge.u == u) ? edge.v : edge.u; // To ensure we get the correct neighboring node
                double weight = edge.cost;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        // Now reconstruct the path and print the substations involved
        if (dist[end] == numeric_limits<double>::infinity()) {
            cout << "No path exists between the substations." << endl;
        } else {
            stack<int> path;
            for (int v = end; v != -1; v = prev[v]) {
                path.push(v);
            }

            cout << "Shortest path from substation " << start << " to substation " << end << ":\n";
            double totalCost = 0;
            while (!path.empty()) {
                int u = path.top();
                path.pop();
                cout << "Substation " << u;
                if (!path.empty()) {
                    cout << " -> ";
                }
                if (!path.empty()) {
                    for (const auto& edge : adjacencyList[u]) {
                        int v = (edge.u == u) ? edge.v : edge.u;
                        if (v == path.top()) {
                            totalCost += edge.cost;
                            break;
                        }
                    }
                }
            }
            cout << "\nTotal cost: " << dist[end] << endl;
        }
}