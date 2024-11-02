import matplotlib.pyplot as plt
import numpy as np
import time
import math
import heapq

# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0  # Parent cell's row index
        self.parent_j = 0  # Parent cell's column index
        self.f = float('inf')  # Total cost of the cell (g + h)
        self.g = float('inf')  # Cost from start to this cell
        self.h = 0  # Heuristic cost from this cell to destination

# Read input from a file
def read_input(file_path):
    with open(file_path, 'r') as f:
        rows, cols = map(int, f.readline().strip().split())
        num_obstacles = int(f.readline().strip())
        grid = [[1 for _ in range(cols)] for _ in range(rows)]  # Initialize grid as unblocked
        
        # Read obstacles
        for _ in range(num_obstacles):
            obstacle = list(map(int, f.readline().strip().split()))
            grid[obstacle[0]][obstacle[1]] = 0  # Mark obstacle cells as blocked

        src = list(map(int, f.readline().strip().split()))  # Source coordinates
        dest = list(map(int, f.readline().strip().split()))  # Destination coordinates

    return rows, cols, grid, src, dest

# Check if a cell is valid (within the grid)
def is_valid(row, col, ROW, COL):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

# Check if a cell is unblocked
def is_unblocked(grid, row, col):
    return grid[row][col] == 1

# Check if a cell is the destination
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Heuristic calculations
def calculate_h_value(row, col, dest, method='euclidean'):
    if method == 'euclidean':
        return math.sqrt((row - dest[0]) ** 2 + (col - dest[1]) ** 2)
    elif method == 'manhattan':
        return abs(row - dest[0]) + abs(col - dest[1])
    elif method == 'diagonal':
        dx = abs(row - dest[0])
        dy = abs(col - dest[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)  # Diagonal distance
    return 0

# Trace the path from source to destination
def trace_path(cell_details, dest):
    path = []
    row, col = dest
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row, col = temp_row, temp_col
    path.append((row, col))
    path.reverse()  # Reverse the path to get the path from source to destination
    return path

# Implement the A* search algorithm
def a_star_search(grid, src, dest, heuristic):
    ROW, COL = len(grid), len(grid[0])
    if not is_valid(src[0], src[1], ROW, COL) or not is_valid(dest[0], dest[1], ROW, COL):
        print("Source or destination is invalid")
        return None

    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return None

    if is_destination(src[0], src[1], dest):
        print("We are already at the destination")
        return [src]

    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    i, j = src[0], src[1]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    open_list = []
    heapq.heappush(open_list, (0.0, i, j))

    found_dest = False

    while open_list:
        # Pop the cell with the lowest f value
        f, i, j = heapq.heappop(open_list)
        closed_list[i][j] = True

        for dir in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_i, new_j = i + dir[0], j + dir[1]

            if is_valid(new_i, new_j, ROW, COL) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                if is_destination(new_i, new_j, dest):
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    found_dest = True
                    return trace_path(cell_details, dest)

                # Use Euclidean distance for g(n)
                g_new = cell_details[i][j].g + math.sqrt(dir[0]**2 + dir[1]**2)
                
                # Choose heuristic based on the parameter passed
                if heuristic == 'manhattan':
                    h_new = calculate_h_value(new_i, new_j, dest, method='manhattan')
                elif heuristic == 'diagonal':
                    h_new = calculate_h_value(new_i, new_j, dest, method='diagonal')
                else:
                    h_new = 0  # Default

                f_new = g_new + h_new

                if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                    heapq.heappush(open_list, (f_new, new_i, new_j))
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j

    if not found_dest:
        print("Failed to find the destination cell")
        return None

def visualize_paths(grid, path_diag_euclidean, path_manhattan_euclidean, src, dest, total_cost_diag, total_cost_manhattan, execution_time_diag, execution_time_manhattan):
    plt.figure(figsize=(15, 8))

    # Plot for Diagonal with Euclidean g(n)
    plt.subplot(2, 2, 1)
    plt.gca().add_patch(plt.Rectangle((0, 0), len(grid[0]), len(grid), color='indigo'))
    plt.grid(which='both', color='black', linewidth=1)
    plt.xticks(np.arange(0, len(grid[0]) + 1, 1))
    plt.yticks(np.arange(0, len(grid) + 1, 1))

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 0:  # Check for obstacles
                plt.gca().add_patch(plt.Rectangle((j, i), 1, 1, color='yellow'))

    if path_diag_euclidean:
        path_x = [y + 0.5 for (x, y) in path_diag_euclidean]
        path_y = [x + 0.5 for (x, y) in path_diag_euclidean]
        plt.plot(path_x, path_y, color='red', linewidth=3)  # Solid line for path

    plt.scatter(src[1] + 0.5, src[0] + 0.5, color='red', s=100, label="Start")  # Start point
    plt.scatter(dest[1] + 0.5, dest[0] + 0.5, color='blue', s=100, label="Goal")  # Goal point

    plt.title("A* (Diagonal Heuristic)")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.xlim(0, len(grid[0]))  # Set limits for x-axis
    plt.ylim(0, len(grid))      # Set limits for y-axis
    plt.gca().invert_yaxis()    # Invert y-axis to match grid layout
    plt.legend()

    # Plot for Manhattan with Euclidean g(n)
    plt.subplot(2, 2, 2)
    plt.gca().add_patch(plt.Rectangle((0, 0), len(grid[0]), len(grid), color='indigo'))
    plt.grid(which='both', color='black', linewidth=1)
    plt.xticks(np.arange(0, len(grid[0]) + 1, 1))
    plt.yticks(np.arange(0, len(grid) + 1, 1))

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 0:  # Check for obstacles
                plt.gca().add_patch(plt.Rectangle((j, i), 1, 1, color='yellow'))

    if path_manhattan_euclidean:
        path_x = [y + 0.5 for (x, y) in path_manhattan_euclidean]
        path_y = [x + 0.5 for (x, y) in path_manhattan_euclidean]
        plt.plot(path_x, path_y, color='red', linewidth=3)  # Solid line for path

    plt.scatter(src[1] + 0.5, src[0] + 0.5, color='red', s=100, label="Start")  # Start point
    plt.scatter(dest[1] + 0.5, dest[0] + 0.5, color='blue', s=100, label="Goal")  # Goal point

    plt.title("A* (Manhattan Heuristic)")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.xlim(0, len(grid[0]))  # Set limits for x-axis
    plt.ylim(0, len(grid))      # Set limits for y-axis
    plt.gca().invert_yaxis()    # Invert y-axis to match grid layout
    plt.legend()

    # Summary Information Plot for Diagonal Heuristic
    plt.subplot(2, 2, 3)
    plt.axis('off')  # Turn off the axis
    plt.text(0.5, 0.7, f"Diagonal Heuristic", ha='center', fontsize=14, fontweight='bold')
    plt.text(0.5, 0.5, f"Final Path: {path_diag_euclidean}", ha='center', fontsize=10)
    plt.text(0.5, 0.4, f"Total Cost: {total_cost_diag}", ha='center', fontsize=12)
    plt.text(0.5, 0.3, f"Execution Time: {execution_time_diag:.6f} seconds", ha='center', fontsize=12)

    # Summary Information Plot for Manhattan Heuristic
    plt.subplot(2, 2, 4)
    plt.axis('off')  # Turn off the axis
    plt.text(0.5, 0.7, f"Manhattan Heuristic", ha='center', fontsize=14, fontweight='bold')
    plt.text(0.5, 0.5, f"Final Path: {path_manhattan_euclidean}", ha='center', fontsize=10)
    plt.text(0.5, 0.4, f"Total Cost: {total_cost_manhattan}", ha='center', fontsize=12)
    plt.text(0.5, 0.3, f"Execution Time: {execution_time_manhattan:.6f} seconds", ha='center', fontsize=12)

    plt.tight_layout()
    plt.show()

# Ensure to update the driver code as before
def main():
    # Read grid, source, and destination from input file
    rows, cols, grid, src, dest = read_input('input.txt')

    # Measure execution time for Diagonal heuristic with Euclidean gn
    start_time_diag = time.time()
    path_diag_euclidean = a_star_search(grid, src, dest, 'diagonal')
    end_time_diag = time.time()
    execution_time_diag = end_time_diag - start_time_diag
    total_cost_diag = sum(1 for _ in path_diag_euclidean) - 1 if path_diag_euclidean else float('inf')

    # Measure execution time for Manhattan heuristic with Euclidean gn
    start_time_manhattan = time.time()
    path_manhattan_euclidean = a_star_search(grid, src, dest, 'manhattan')
    end_time_manhattan = time.time()
    execution_time_manhattan = end_time_manhattan - start_time_manhattan
    total_cost_manhattan = sum(1 for _ in path_manhattan_euclidean) - 1 if path_manhattan_euclidean else float('inf')

    # Visualize both paths side by side with total costs
    visualize_paths(grid, path_diag_euclidean, path_manhattan_euclidean, src, dest, total_cost_diag, total_cost_manhattan, execution_time_diag, execution_time_manhattan)

if __name__ == "__main__":
    main()
