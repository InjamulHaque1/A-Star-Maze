import math
import tkinter as tk
from queue import PriorityQueue

# Heuristic functions
def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def diagonal_distance(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return dx + dy + (math.sqrt(2) - 2) * min(dx, dy)

# Fixed gn calculation (Euclidean distance)
def euclidean_distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

# Heuristic selection function
def heuristic_distance(a, b, heuristic):
    if heuristic == 'manhattan':
        return manhattan_distance(a, b)
    elif heuristic == 'diagonal':
        return diagonal_distance(a, b)

# Pathfinding with A* algorithm using fixed gn and selectable heuristic
def a_star_search(start, end, grid, heuristic='manhattan'):
    open_set = PriorityQueue()
    open_set.put((0, tuple(start)))
    came_from = {}
    g_score = {tuple(start): 0}
    f_score = {tuple(start): heuristic_distance(start, end, heuristic)}

    while not open_set.empty():
        current = open_set.get()[1]

        if current == tuple(end):
            return reconstruct_path(came_from, current), g_score[current]

        for neighbor in get_neighbors(current, len(grid), len(grid[0])):
            # Check if the neighbor is within grid bounds and not an obstacle
            if 0 <= neighbor[1] < len(grid) and 0 <= neighbor[0] < len(grid[0]) and grid[neighbor[1]][neighbor[0]] == 1:
                tentative_g_score = g_score[current] + euclidean_distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic_distance(neighbor, end, heuristic)
                    open_set.put((f_score[neighbor], neighbor))

    return None, None

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(current)
    path.reverse()
    return path

# Get valid neighbors
def get_neighbors(pos, rows, cols):
    neighbors = [
        (pos[0] + dx, pos[1] + dy)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]
    ]
    return [(x, y) for x, y in neighbors if 0 <= x < cols and 0 <= y < rows]

# Read input from file
def read_input(file_path):
    with open(file_path, 'r') as f:
        rows, cols = map(int, f.readline().strip().split())
        num_obstacles = int(f.readline().strip())
        grid = [[1 for _ in range(cols)] for _ in range(rows)]  # Initialize grid as unblocked
        
        # Read obstacles
        for _ in range(num_obstacles):
            obstacle = list(map(int, f.readline().strip().split()))
            grid[obstacle[1]][obstacle[0]] = 0  # Mark obstacle cells as blocked

        src = list(map(int, f.readline().strip().split()))  # Source coordinates
        dest = list(map(int, f.readline().strip().split()))  # Destination coordinates

    return rows, cols, grid, src, dest

# Tkinter interface to select heuristic and animate path
def run_gui(grid, src, dest):
    def submit_choice():
        selected_heuristic = heuristic_var.get()
        path, cost = a_star_search(src, dest, grid, selected_heuristic)
        
        if path:
            print(f"Path found with {selected_heuristic} heuristic: {path}")
            print(f"Path cost: {cost}")
            draw_grid(path, src, dest)
        else:
            print("No path found.")

    def draw_grid(path, src, dest):
        # Draw grid and path
        for i in range(rows):
            for j in range(cols):
                color = "white" if grid[i][j] == 1 else "black"
                cell = canvas.create_rectangle(i * cell_size, j * cell_size, 
                                               (i + 1) * cell_size, (j + 1) * cell_size, fill=color)
                cells[(j, i)] = cell

        # Color the source and destination
        canvas.itemconfig(cells[(src[1], src[0])], fill="green")
        canvas.itemconfig(cells[(dest[1], dest[0])], fill="red")

        # Draw path
        for (y, x) in path:
            if (y, x) != tuple(src) and (y, x) != tuple(dest):
                canvas.itemconfig(cells[(y, x)], fill="blue")

    # Set up Tkinter window
    root = tk.Tk()
    root.title("Pathfinding Heuristic Selector")

    # Adjust canvas size and cell size for better visibility
    global cell_size
    cell_size = 50  # Increase cell size for better visibility
    canvas_width = cols * cell_size
    canvas_height = rows * cell_size
    canvas = tk.Canvas(root, width=canvas_width, height=canvas_height)
    canvas.pack()

    # Draw axes (swapped)
    for i in range(rows + 1):
        canvas.create_line(0, i * cell_size, canvas_width, i * cell_size, fill='gray')  # Horizontal lines for y-axis
    for i in range(cols + 1):
        canvas.create_line(i * cell_size, 0, i * cell_size, canvas_height, fill='gray')  # Vertical lines for x-axis

    heuristic_var = tk.StringVar(value="manhattan")
    tk.Label(root, text="Choose heuristic:", font=("Helvetica", 14)).pack()
    tk.Radiobutton(root, text="Manhattan", variable=heuristic_var, value="manhattan", font=("Helvetica", 12)).pack()
    tk.Radiobutton(root, text="Diagonal", variable=heuristic_var, value="diagonal", font=("Helvetica", 12)).pack()
    
    submit_button = tk.Button(root, text="Find Path", command=submit_choice, font=("Helvetica", 14))
    submit_button.pack()

    cells = {}
    draw_grid([], src, dest)

    root.mainloop()

# Main Execution
file_path = "input.txt"
rows, cols, grid, src, dest = read_input(file_path)
run_gui(grid, src, dest)
