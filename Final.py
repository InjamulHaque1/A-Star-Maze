import tkinter as tk
import time
import math
import heapq

# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0
        self.parent_j = 0
        self.f = float('inf')
        self.g = float('inf')
        self.h = 0

# Read input from a file
def read_input(file_path):
    with open(file_path, 'r') as f:
        rows, cols = map(int, f.readline().strip().split())
        num_obstacles = int(f.readline().strip())
        grid = [[1 for _ in range(cols)] for _ in range(rows)]

        for _ in range(num_obstacles):
            obstacle = list(map(int, f.readline().strip().split()))
            grid[obstacle[0]][obstacle[1]] = 0

        src = list(map(int, f.readline().strip().split()))
        dest = list(map(int, f.readline().strip().split()))

    return rows, cols, grid, src, dest

# Check cell validity
def is_valid(row, col, ROW, COL):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

def is_unblocked(grid, row, col):
    return grid[row][col] == 1

def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Heuristic calculations
def calculate_h_value(row, col, dest, method):
    if method == 'manhattan':
        return abs(row - dest[0]) + abs(col - dest[1])
    elif method == 'diagonal':
        dx = abs(row - dest[0])
        dy = abs(col - dest[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
    return 0

def trace_path(cell_details, dest):
    path = []
    row, col = dest
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row, col = temp_row, temp_col
    path.append((row, col))
    path.reverse()
    return path

def a_star_search(grid, src, dest, heuristic):
    ROW, COL = len(grid), len(grid[0])
    if not is_valid(src[0], src[1], ROW, COL) or not is_valid(dest[0], dest[1], ROW, COL):
        print("Source or destination is invalid")
        return None

    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or destination is blocked")
        return None

    if is_destination(src[0], src[1], dest):
        print("Already at destination")
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

    while open_list:
        f, i, j = heapq.heappop(open_list)
        closed_list[i][j] = True

        for dir in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_i, new_j = i + dir[0], j + dir[1]

            if is_valid(new_i, new_j, ROW, COL) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                if is_destination(new_i, new_j, dest):
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    return trace_path(cell_details, dest)

                g_new = cell_details[i][j].g + math.sqrt(dir[0]**2 + dir[1]**2)
                h_new = calculate_h_value(new_i, new_j, dest, method=heuristic)
                f_new = g_new + h_new

                if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                    heapq.heappush(open_list, (f_new, new_i, new_j))
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j

    print("Destination not found")
    return None

class PathfindingApp:
    def __init__(self, master, grid, path_manhattan, path_diagonal, src, dest):
        self.master = master
        self.master.title("A* Pathfinding Visualization")
        self.grid = grid
        self.path_manhattan = path_manhattan
        self.path_diagonal = path_diagonal
        self.src = src
        self.dest = dest
        self.cell_size = 45

        # Create a Canvas for scrollable content and add vertical scrollbar
        self.main_canvas = tk.Canvas(self.master)
        self.main_canvas.pack(side="left", fill="both", expand=True)

        # Vertical scrollbar
        self.scroll_y = tk.Scrollbar(self.master, orient="vertical", command=self.main_canvas.yview)
        self.scroll_y.pack(side="right", fill="y")

        # Configure Canvas to use the scrollbar
        self.main_canvas.configure(yscrollcommand=self.scroll_y.set)

        # Create frame inside canvas
        self.scrollable_frame = tk.Frame(self.main_canvas)
        self.main_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")

        # Bind resizing event to update scroll region
        self.scrollable_frame.bind("<Configure>", lambda e: self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all")))

        # Make sure scrolling works with the mouse wheel
        self.main_canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        # Initialize GUI components in the scrollable frame
        self.create_heading()
        self.create_layout()
        self.create_tables()
        self.animate_paths()

    def _on_mousewheel(self, event):
        # Allows for mouse wheel scrolling
        self.main_canvas.yview_scroll(int(-1*(event.delta/120)), "units")

    def create_heading(self):
        heading_frame = tk.Frame(self.scrollable_frame)
        heading_frame.grid(row=0, column=0, columnspan=3)
        heading_label = tk.Label(heading_frame, text="Here is the dynamic Manhattan and Diagonal Heuristic respectively:", font=("Calibri", 16, "bold"))
        heading_label.pack()

    def create_layout(self):
        self.frame_manhattan = tk.Frame(self.scrollable_frame, bg='white')
        self.frame_manhattan.grid(row=1, column=0)

        self.frame_diagonal = tk.Frame(self.scrollable_frame, bg='white')
        self.frame_diagonal.grid(row=1, column=1)

        self.frame_value_table = tk.Frame(self.scrollable_frame)
        self.frame_value_table.grid(row=1, column=2)

        self.canvas_manhattan = tk.Canvas(self.frame_manhattan, width=self.cell_size * len(self.grid[0]), height=self.cell_size * len(self.grid), bg='indigo')
        self.canvas_manhattan.pack()

        self.canvas_diagonal = tk.Canvas(self.frame_diagonal, width=self.cell_size * len(self.grid[0]), height=self.cell_size * len(self.grid), bg='indigo')
        self.canvas_diagonal.pack()

        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                color = 'yellow' if self.grid[i][j] == 0 else 'indigo'
                self.canvas_manhattan.create_rectangle(j * self.cell_size, i * self.cell_size, (j + 1) * self.cell_size, (i + 1) * self.cell_size, fill=color)
                self.canvas_diagonal.create_rectangle(j * self.cell_size, i * self.cell_size, (j + 1) * self.cell_size, (i + 1) * self.cell_size, fill=color)

        for canvas in [self.canvas_manhattan, self.canvas_diagonal]:
            canvas.create_oval(self.src[1] * self.cell_size + 10, self.src[0] * self.cell_size + 10, self.src[1] * self.cell_size + 30, self.src[0] * self.cell_size + 30, fill='red')
            canvas.create_oval(self.dest[1] * self.cell_size + 10, self.dest[0] * self.cell_size + 10, self.dest[1] * self.cell_size + 30, self.dest[0] * self.cell_size + 30, fill='blue')

    def create_tables(self):
        self.table_frame = tk.Frame(self.scrollable_frame)
        self.table_frame.grid(row=2, column=0, columnspan=2)

        self.execution_time_labels = []
        self.path_labels = []
        self.total_cost_labels = []
        self.step_labels = []
        self.g_labels = []
        self.h_labels = []
        self.f_labels = []

        for i, heuristic in enumerate(["Manhattan", "Diagonal"]):
            frame = tk.Frame(self.table_frame, relief="solid")
            frame.grid(row=0, column=i, padx=10)
            tk.Label(frame, text=f"{heuristic} Steps", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=4)

            headers = ["Step", "g(n)", "h(n)", "f(n)"]
            for col, header in enumerate(headers):
                tk.Label(frame, text=header, borderwidth=1, relief="solid", padx=5, pady=5, font=("Arial", 10, "bold"), bg="#FFFF00").grid(row=1, column=col, sticky="nsew")

            num_steps = max(len(self.path_manhattan), len(self.path_diagonal))
            for j in range(num_steps):
                step_label = tk.Label(frame, text="", borderwidth=1, relief="solid", pady=3)
                step_label.grid(row=j + 2, column=0, sticky="nsew")
                self.step_labels.append(step_label)

                g_label = tk.Label(frame, text="", borderwidth=1, relief="solid", pady=3)
                g_label.grid(row=j + 2, column=1, sticky="nsew")
                self.g_labels.append(g_label)

                h_label = tk.Label(frame, text="", borderwidth=1, relief="solid", pady=3)
                h_label.grid(row=j + 2, column=2, sticky="nsew")
                self.h_labels.append(h_label)

                f_label = tk.Label(frame, text="", borderwidth=1, relief="solid", pady=3)
                f_label.grid(row=j + 2, column=3, sticky="nsew")
                self.f_labels.append(f_label)

            exec_time_label = tk.Label(frame, text="Execution Time: ", font=("Arial", 10, "bold"))
            exec_time_label.grid(row=num_steps + 3, column=0, columnspan=4, sticky="w", padx=5)
            self.execution_time_labels.append(exec_time_label)

            path_label = tk.Label(frame, text="Path: ", font=("Arial", 10, "bold"))
            path_label.grid(row=num_steps + 4, column=0, columnspan=4, sticky="w", padx=5)
            self.path_labels.append(path_label)

            cost_label = tk.Label(frame, text="Total Cost: ", font=("Arial", 10, "bold"))
            cost_label.grid(row=num_steps + 5, column=0, columnspan=4, sticky="w", padx=5)
            self.total_cost_labels.append(cost_label)

    def update_summary(self, index, execution_time, path, total_cost):
        self.execution_time_labels[index].config(text=f"Execution Time: {execution_time:.2f} seconds")
        self.path_labels[index].config(text=f"Path: {path}")
        self.total_cost_labels[index].config(text=f"Total Cost: {total_cost:.2f}")

    def animate_paths(self):
        self.animate_path(self.canvas_manhattan, self.path_manhattan, "Manhattan", 0)
        self.animate_path(self.canvas_diagonal, self.path_diagonal, "Diagonal", 1)

    def animate_path(self, canvas, path, heuristic, table_column):
        g_n = 0
        previous_point = None

        for index, (i, j) in enumerate(path):
            if previous_point is not None:
                canvas.create_line(previous_point[1] * self.cell_size + self.cell_size // 2,
                                   previous_point[0] * self.cell_size + self.cell_size // 2,
                                   j * self.cell_size + self.cell_size // 2,
                                   i * self.cell_size + self.cell_size // 2,
                                   fill='red', width=2)

            if index > 0:
                prev_i, prev_j = path[index - 1]
                g_n += math.sqrt((i - prev_i) ** 2 + (j - prev_j) ** 2)

            h_n = calculate_h_value(i, j, self.dest, method=heuristic.lower())
            f_n = g_n + h_n

            for k, val in enumerate([index + 1, round(g_n, 2), round(h_n, 2), round(f_n, 2)]):
                tk.Label(self.table_frame.winfo_children()[table_column], text=str(val)).grid(row=index + 2, column=k)

            adjacent_cells = [(i + x, j + y) for x, y in [(0, 1), (1, 0), (0, -1), (-1, 0),
                                                          (1, 1), (1, -1), (-1, 1), (-1, -1)]]
            for adj_i, adj_j in adjacent_cells:
                if is_valid(adj_i, adj_j, len(self.grid), len(self.grid[0])) and is_unblocked(self.grid, adj_i, adj_j):
                    adj_g_n = g_n + math.sqrt((adj_i - i) ** 2 + (adj_j - j) ** 2)
                    adj_h_n = calculate_h_value(adj_i, adj_j, self.dest, method=heuristic.lower())
                    adj_f_n = adj_g_n + adj_h_n

                    canvas.create_rectangle(adj_j * self.cell_size, adj_i * self.cell_size,
                                            (adj_j + 1) * self.cell_size, (adj_i + 1) * self.cell_size,
                                            fill='lightgreen')
                    canvas.create_text(adj_j * self.cell_size + self.cell_size // 2,
                                       adj_i * self.cell_size + self.cell_size // 2,
                                       text=f"g:{round(adj_g_n, 1)}\nh:{round(adj_h_n, 1)}\nf:{round(adj_f_n, 1)}",
                                       font=("Arial", 8), fill="black")

            previous_point = (i, j)
            self.master.update()
            time.sleep(0.3)

            canvas.delete("adjacent")

        canvas.delete("adjacent")

if __name__ == "__main__":
    file_path = 'input.txt'
    rows, cols, grid, src, dest = read_input(file_path)

    # Run A* for Manhattan heuristic and measure execution time
    start_time = time.time()
    path_manhattan = a_star_search(grid, src, dest, 'manhattan')
    execution_time_manhattan = time.time() - start_time

    # Calculate total cost for Manhattan path using Euclidean distance
    total_cost_manhattan = sum(
        math.sqrt((path_manhattan[i][0] - path_manhattan[i - 1][0]) ** 2 +
                  (path_manhattan[i][1] - path_manhattan[i - 1][1]) ** 2)
        for i in range(1, len(path_manhattan))
    ) if path_manhattan else 0

    # Run A* for Diagonal heuristic and measure execution time
    start_time = time.time()
    path_diagonal = a_star_search(grid, src, dest, 'diagonal')
    execution_time_diagonal = time.time() - start_time

    # Calculate total cost for Diagonal path using Euclidean distance
    total_cost_diagonal = sum(
        math.sqrt((path_diagonal[i][0] - path_diagonal[i - 1][0]) ** 2 +
                  (path_diagonal[i][1] - path_diagonal[i - 1][1]) ** 2)
        for i in range(1, len(path_diagonal))
    ) if path_diagonal else 0

    # Initialize the Tkinter app and update tables with the summary data
    root = tk.Tk()
    app = PathfindingApp(root, grid, path_manhattan, path_diagonal, src, dest)

    # Update the summary information at the bottom of each table
    app.update_summary(0, execution_time_manhattan, path_manhattan, total_cost_manhattan)
    app.update_summary(1, execution_time_diagonal, path_diagonal, total_cost_diagonal)

    root.mainloop()