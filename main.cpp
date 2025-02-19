#include <iostream>
#include "Graph.h"

using namespace std;

// Function to print a line separator for readability
void printLine() 
{
    cout << "===============================================\n";
}

// Function to print a sub-menu separator line
void printSubMenuLine() 
{
    cout << "-----------------------------------------------\n";
}

int main() 
{
    int n, m;
    double budget;

    Graph* graph = nullptr;  // Pointer to hold the graph object

    printLine();
    cout << "  Welcome to the Electric Grid Design Program  \n";  // Display the program welcome message
    printLine();

    int choice;
    // Main menu loop to interact with the user
    do 
    {
        printSubMenuLine();
        cout << "               --- Main Menu ---               \n";  // Main menu options
        printSubMenuLine();
        cout << "1. Load an existing graph from file\n";  // Option to load an existing graph
        cout << "2. Create a new graph\n";  // Option to create a new graph
        cout << "0. Exit\n";  // Option to exit the program
        cout << "Enter your choice: ";
        cin >> choice;

        switch (choice) 
        {
            case 1: {  // Load an existing graph from file
    string filename;
    cout << "Enter filename to load the graph: ";
    cin >> filename;

    if (graph != nullptr) {
        delete graph; // Free previously allocated memory
    }

    graph = new Graph(0, 0, 0.0);  // Create an empty Graph
    graph->loadGridFromFile(filename);

    // Check if the graph is still empty after attempting to load
    if (graph->PowerLines.empty()) {
        cout << "Warning: Graph loading failed or file was empty. Returning to the main menu.\n";
        delete graph;  // Free the memory if loading failed
        graph = nullptr;
    }

    break;
}
            case 2: 
            {  // If the user selects to create a new graph
                cout << "Enter the number of substations: ";
                cin >> n;  // Input for number of substations
                cout << "Enter the number of power lines: ";
                cin >> m;  // Input for number of power lines
                cout << "Enter the budget for the Power Grid: ";
                cin >> budget;  // Input for the budget
                graph = new Graph(n, m, budget);  // Create a new Graph object with the provided details

                // Loop to add power lines to the graph
                for (int i = 0; i < m; ++i) 
                {
                    int u, v;
                    double cost;
                    cout << "Enter PowerLine " << i + 1 << " (u v cost): ";
                    cin >> u >> v >> cost;  // Input for the power line details
                    graph->addPowerLine(u, v, cost);  // Add the power line to the graph
                }
                break;
            }
            case 0:  // If the user selects to exit
                printLine();
                cout << "Exiting the program.\n";  // Exit message
                printLine();
                break;
            default:  // In case of an invalid input
                cout << "Invalid choice. Please try again.\n";
                break;
        }
    } while (choice != 0 && graph == nullptr);  // Keep asking for input until valid graph is created or exit

    if (graph != nullptr) 
    {  // If a valid graph is created, proceed to the electric grid management menu
        int menuChoice;
        do 
        {
            printSubMenuLine();
            cout << "           --- Electric Grid Management Menu ---          \n";  // Management menu options
            printSubMenuLine();
            cout << "1. Display Graph\n";  // Option to display the graph
            cout << "2. Add PowerLine\n";  // Option to add a power line
            cout << "3. Add Substation\n";  // Option to add a substation
            cout << "4. Remove PowerLine\n";  // Option to remove a power line
            cout << "5. Simulate Substation Failure\n";  // Option to simulate substation failure
            cout << "6. Calculate MST For the power grid\n";  // Option to calculate Minimum Spanning Tree (MST)
            cout << "7. Apply Budget Constraints\n";  // Option to apply budget constraints
            cout << "8. Check PowerLine Connectivity\n";  // Option to check if power lines are connected
            cout << "9. Change PowerLine cost\n";  // Option to change the cost of a power line
            cout << "10. Number of PowerLines of a Substation\n";  // Option to count power lines for a substation
            cout << "11. Minimum Distance between two substations\n";  // Option to calculate minimum distance
            cout << "12. Update Budget\n";  // Option to update the budget
            cout << "13. Save Graph to File\n";  // Option to save the graph to a file
            cout << "14. Load Graph from File\n";  // Option to load a graph from a file
            cout << "0. Exit\n";  // Option to exit the management menu
            cout << "Enter your choice: ";
            cin >> menuChoice;  // Input for the menu choice

            // Switch case to handle the menu choices
            switch (menuChoice) 
            {
                case 1: 
                {
                    // Display the graph
                    graph->displayGraph();
                    break;
                }
                
                case 2: 
                {  // Add a power line
                    int u, v;
                    double cost;
                    cout << "Enter Substation 1, Substation 2, and cost of the PowerLine: ";
                    cin >> u >> v >> cost;  // Input for the power line details
                    graph->addPowerLine(u, v, cost);  // Add the power line to the graph
                    cout << "PowerLine added successfully.\n";
                    break;
                }
                
                case 3: 
                {  // Add a substation
                    int id;
                    double demand, production;
                    cout << "Enter Substation ID, demand, and production: ";
                    cin >> id >> demand >> production;  // Input for substation details
                    graph->addSubstation(id, demand, production);  // Add the substation to the graph
                    break;
                }
                
                case 4: 
                {  // Remove a power line
                    int u, v;
                    cout << "Enter Substation 1 and Substation 2 to remove the PowerLine: ";
                    cin >> u >> v;  // Input for the power line to be removed
                    graph->removePowerLine(u, v);  // Remove the power line from the graph
                    break;
                }
                
                case 5: 
                {  // Simulate a substation failure
                    int Substation;
                    cout << "Enter the Substation to simulate failure: ";
                    cin >> Substation;  // Input for the substation to fail
                    graph->simulateSubstationFailure(Substation);  // Simulate the failure
                    break;
                }
                
                case 6: 
                {  // Calculate MST for the grid
                    graph->calculateMultipleMSTs();
                    break;
                }
                
                case 7: 
                {  // Apply budget constraints
                    graph->applyBudgetConstraints();
                    break;
                }
                
                case 8: 
                {  // Check connectivity of power lines between substations
                    int u, v;
                    cout << "Enter Substation 1 and Substation 2 to check connectivity: ";
                    cin >> u >> v;  // Input for the substations to check connectivity
                    graph->checkPowerLineConnectivity(u, v);  // Check connectivity
                    break;
                }
                
                case 9: 
                {  // Change the cost of a power line
                    graph->changePowerLinecost();
                    break;
                }
                
                case 10: 
                {  // Count the number of power lines of a substation
                    graph->countPowerLines();
                    break;
                }
                
                case 11: 
                {  // Calculate the minimum distance between two substations
                    graph->minimumdistancebetweensubstations();
                    break;
                }
                
                case 12: 
                {  // Update the budget
                    double newBudget;
                    cout << "Enter new budget: ";
                    cin >> newBudget;  // Input for the new budget
                    graph->updateBudget(newBudget);  // Update the budget
                    break;
                }
                
                case 13: 
                {  // Save the graph to a file
                    string filename;
                    cout << "Enter filename to save the graph: ";
                    cin >> filename;  // Input for the filename to save the graph
                    graph->saveGridToFile(filename);  // Save the graph data to the file
                    break;
                }
                
                case 14: 
                {  // Load a graph from a file
                    string filename;
                    cout << "Enter filename to load the graph: ";
                    cin >> filename;  // Input for the filename to load the graph
                    graph->loadGridFromFile(filename);  // Load the graph data from the file
                    break;
                }
                
                case 0:  // Exit the program
                    printLine();
                    cout << "Exiting the program.\n";  // Exit message
                    printLine();
                    break;
                
                default:  // Handle invalid menu choice
                    cout << "Invalid choice. Please try again.\n";
                    break;
            }
        } while (menuChoice != 0);  // Keep showing the menu until the user exits
    }

    return 0;
}

