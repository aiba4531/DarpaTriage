import numpy as np
import pulp
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt

def generate_random_targets(num_targets, x_range=(0, 10), y_range=(0, 10)):
    """Generate random 2D coordinates for targets."""
    x_coords = np.random.uniform(x_range[0], x_range[1], num_targets)
    y_coords = np.random.uniform(y_range[0], y_range[1], num_targets)
    return list(zip(x_coords, y_coords))

def solve_tsp_milp(targets):
    """Solve the Traveling Salesman Problem using MILP."""
    num_targets = len(targets)
    
    # Compute distance matrix
    distance_matrix = {
        (i, j): euclidean(targets[i], targets[j])
        for i in range(num_targets)
        for j in range(num_targets) if i != j
    }
    
    # Create the MILP problem
    prob = pulp.LpProblem("TSP", pulp.LpMinimize)
    
    # Decision variables: x[i][j] = 1 if edge (i, j) is used
    x = pulp.LpVariable.dicts("x", distance_matrix, cat="Binary")
    
    # Objective function: Minimize total distance
    prob += pulp.lpSum(distance_matrix[i, j] * x[i, j] for i, j in distance_matrix)
    
    # Constraints: Each node must have exactly one incoming and one outgoing edge
    for k in range(num_targets):
        prob += pulp.lpSum(x[i, k] for i in range(num_targets) if i != k) == 1, f"Incoming_{k}"
        prob += pulp.lpSum(x[k, j] for j in range(num_targets) if j != k) == 1, f"Outgoing_{k}"
    
    # Subtour elimination constraints using Miller-Tucker-Zemlin (MTZ) formulation
    u = pulp.LpVariable.dicts("u", range(num_targets), lowBound=0, upBound=num_targets - 1, cat="Continuous")
    for i in range(1, num_targets):
        for j in range(1, num_targets):
            if i != j:
                prob += u[i] - u[j] + (num_targets * x[i, j]) <= num_targets - 1, f"Subtour_{i}_{j}"
    
    # Solve the problem
    prob.solve()
    
    # Extract the solution
    if pulp.LpStatus[prob.status] == "Optimal":
        tour = []
        for i in range(num_targets):
            for j in range(num_targets):
                if i != j and pulp.value(x[i, j]) == 1:
                    tour.append((i, j))
        # Return the tour, cordinates, and total distance
        return tour, targets, pulp.value(prob.objective)
    
    else:
        print("No optimal solution found.")
        return None, None

def plot_tsp(targets, tour, show_plot=False):
    """Visualize the TSP solution."""
    if show_plot:    
        plt.figure(figsize=(8, 8))
        for i, (x, y) in enumerate(targets):
            plt.scatter(x, y, color="blue")
            plt.text(x, y, f"{i}", fontsize=12, color="red")
        
        for i, j in tour:
            x_coords = [targets[i][0], targets[j][0]]
            y_coords = [targets[i][1], targets[j][1]]
            plt.plot(x_coords, y_coords, color="green")
        
        plt.xlabel("X-coordinate")
        plt.ylabel("Y-coordinate")
        plt.title("TSP Solution")
        plt.grid(True)
        plt.show()
        plt.saveas("tsp_solution.png")
