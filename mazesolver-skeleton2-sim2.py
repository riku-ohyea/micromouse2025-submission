import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import time
from collections import deque

class MazeGrid:
    def __init__(self):
        self.size = 9
        self.costs = np.zeros((self.size, self.size))
        self.explored = np.zeros((self.size, self.size), dtype=bool)
        self.walls = np.ones((self.size, self.size, 4), dtype=bool)  # N,E,S,W
        self.known_walls = np.zeros((self.size, self.size, 4), dtype=bool)  # Walls that robot has seen
        self.initialize_cost_grid()

    def initialize_cost_grid(self):
        # Start with simple Manhattan distance
        center = self.size // 2
        for i in range(self.size):
            for j in range(self.size):
                self.costs[i,j] = abs(i - center) + abs(j - center)

    def update_costs_flood_fill(self):
        # Initialize all costs to infinity except goal
        new_costs = np.full((self.size, self.size), float('inf'))
        center = self.size // 2
        new_costs[center, center] = 0
        
        # Use queue for flood fill
        queue = deque([(center, center)])
        
        while queue:
            y, x = queue.popleft()
            current_cost = new_costs[y, x]
            
            # Check all four directions
            directions = [(0,-1), (1,0), (0,1), (-1,0)]  # N,E,S,W
            for i, (dy, dx) in enumerate(directions):
                new_y, new_x = y + dy, x + dx
                
                # Check bounds and walls
                if (0 <= new_x < self.size and 0 <= new_y < self.size and
                    not self.known_walls[y, x, i]):
                    new_cost = current_cost + 1
                    
                    # Update if new cost is lower
                    if new_cost < new_costs[new_y, new_x]:
                        new_costs[new_y, new_x] = new_cost
                        queue.append((new_y, new_x))
        
        # Update costs, keeping infinity values as manhattan distance
        for y in range(self.size):
            for x in range(self.size):
                if new_costs[y,x] == float('inf'):
                    center = self.size // 2
                    new_costs[y,x] = abs(y - center) + abs(x - center)
        
        self.costs = new_costs

    def mark_cell_explored(self, x, y):
        self.explored[y,x] = True

    def is_cell_explored(self, x, y):
        return self.explored[y,x]

    def set_wall(self, x, y, direction, value):
        """Set wall and maintain consistency with neighboring cells"""
        self.known_walls[y,x,direction] = value
        if direction == 0 and y < self.size-1:  # North
            self.known_walls[y+1,x,2] = value
        elif direction == 1 and x < self.size-1:  # East
            self.known_walls[y,x+1,3] = value
        elif direction == 2 and y > 0:  # South
            self.known_walls[y-1,x,0] = value
        elif direction == 3 and x > 0:  # West
            self.known_walls[y,x-1,1] = value

class DraftRobot:
    def __init__(self, grid):
        self.maze_grid = grid
        self.position = [0, 0]
        
    def scan_surrounding_walls(self):
        print(f"Scanning walls at position {self.position}")
        x, y = self.position
        return self.maze_grid.walls[y,x]
        
    def update_grid_walls(self, walls):
        print(f"Updating walls at position {self.position}: {walls}")
        x, y = self.position
        for direction in range(4):
            self.maze_grid.set_wall(x, y, direction, walls[direction])
        # Update costs based on new wall information
        self.maze_grid.update_costs_flood_fill()
        
    def get_available_moves(self):
        print(f"Getting available moves from position {self.position}")
        x, y = self.position
        moves = []
        walls = self.maze_grid.known_walls[y,x]
        
        if not walls[0] and y < self.maze_grid.size-1: moves.append((x, y+1))
        if not walls[1] and x < self.maze_grid.size-1: moves.append((x+1, y))
        if not walls[2] and y > 0: moves.append((x, y-1))
        if not walls[3] and x > 0: moves.append((x-1, y))
        
        print(f"Available moves: {moves}")
        return moves
        
    def find_lowest_cost_move(self, available_moves):
        print("Finding lowest cost move")
        if not available_moves:
            print("No moves available!")
            return self.position
            
        unexplored_moves = [move for move in available_moves 
                          if not self.maze_grid.is_cell_explored(*move)]
        moves_to_consider = unexplored_moves if unexplored_moves else available_moves
        costs = [self.maze_grid.costs[y,x] for x,y in moves_to_consider]
        best_move = moves_to_consider[np.argmin(costs)]
        
        print(f"Selected move: {best_move}")
        return best_move
        
    def move_to_cell(self, x, y):
        print(f"Moving from {self.position} to ({x}, {y})")
        self.position = [x, y]
        self.maze_grid.mark_cell_explored(x, y)

class Simulator:
    def __init__(self, robot, maze_grid):
        self.robot = robot
        self.maze_grid = maze_grid
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        
    def generate_random_maze(self):
        """Generate a maze with multiple solutions using depth-first search"""
        self.maze_grid.walls.fill(True)
        
        def get_unvisited_neighbors(x, y, visited):
            neighbors = []
            directions = [(0,1), (1,0), (0,-1), (-1,0)]
            for dx, dy in directions:
                new_x, new_y = x + dx, y + dy
                if (0 <= new_x < self.maze_grid.size and 
                    0 <= new_y < self.maze_grid.size and 
                    (new_x, new_y) not in visited):
                    neighbors.append((new_x, new_y))
            return neighbors
        
        visited = set([(0, 0)])
        stack = [(0, 0)]
        
        while stack:
            x, y = stack[-1]
            neighbors = get_unvisited_neighbors(x, y, visited)
            
            if neighbors:
                next_x, next_y = neighbors[np.random.randint(len(neighbors))]
                if next_x > x:
                    self.maze_grid.walls[y,x,1] = False
                    self.maze_grid.walls[y,next_x,3] = False
                elif next_x < x:
                    self.maze_grid.walls[y,x,3] = False
                    self.maze_grid.walls[y,next_x,1] = False
                elif next_y > y:
                    self.maze_grid.walls[y,x,0] = False
                    self.maze_grid.walls[next_y,x,2] = False
                else:
                    self.maze_grid.walls[y,x,2] = False
                    self.maze_grid.walls[next_y,x,0] = False
                    
                stack.append((next_x, next_y))
                visited.add((next_x, next_y))
            else:
                stack.pop()
        
        # Add extra paths
        for _ in range(self.maze_grid.size // 2):
            x = np.random.randint(self.maze_grid.size)
            y = np.random.randint(self.maze_grid.size)
            direction = np.random.randint(4)
            
            if direction == 0 and y < self.maze_grid.size-1:
                self.maze_grid.walls[y,x,0] = False
                self.maze_grid.walls[y+1,x,2] = False
            elif direction == 1 and x < self.maze_grid.size-1:
                self.maze_grid.walls[y,x,1] = False
                self.maze_grid.walls[y,x+1,3] = False
            elif direction == 2 and y > 0:
                self.maze_grid.walls[y,x,2] = False
                self.maze_grid.walls[y-1,x,0] = False
            elif direction == 3 and x > 0:
                self.maze_grid.walls[y,x,3] = False
                self.maze_grid.walls[y,x-1,1] = False
                
    def draw(self):
        self.ax.clear()
        
        # Draw grid
        for y in range(self.maze_grid.size):
            for x in range(self.maze_grid.size):
                # Draw actual walls in light gray
                if self.maze_grid.walls[y,x,0]:
                    self.ax.plot([x, x+1], [y+1, y+1], 'lightgray', linewidth=2)
                if self.maze_grid.walls[y,x,1]:
                    self.ax.plot([x+1, x+1], [y, y+1], 'lightgray', linewidth=2)
                if self.maze_grid.walls[y,x,2]:
                    self.ax.plot([x, x+1], [y, y], 'lightgray', linewidth=2)
                if self.maze_grid.walls[y,x,3]:
                    self.ax.plot([x, x], [y, y+1], 'lightgray', linewidth=2)
                    
                # Draw known walls in black
                if self.maze_grid.known_walls[y,x,0]:
                    self.ax.plot([x, x+1], [y+1, y+1], 'k-', linewidth=2)
                if self.maze_grid.known_walls[y,x,1]:
                    self.ax.plot([x+1, x+1], [y, y+1], 'k-', linewidth=2)
                if self.maze_grid.known_walls[y,x,2]:
                    self.ax.plot([x, x+1], [y, y], 'k-', linewidth=2)
                if self.maze_grid.known_walls[y,x,3]:
                    self.ax.plot([x, x], [y, y+1], 'k-', linewidth=2)
                    
                # Draw costs
                cost = self.maze_grid.costs[y,x]
                self.ax.text(x+0.5, y+0.5, f'{int(cost)}',
                           ha='center', va='center')
                
                # Color explored cells
                if self.maze_grid.explored[y,x]:
                    self.ax.add_patch(Rectangle((x, y), 1, 1, 
                                    facecolor='lightgray', alpha=0.3))
        
        # Draw robot
        robot_x, robot_y = self.robot.position
        self.ax.add_patch(Rectangle((robot_x, robot_y), 1, 1, 
                                  facecolor='red', alpha=0.5))
        
        # Draw goal
        center = self.maze_grid.size // 2
        self.ax.add_patch(Rectangle((center, center), 1, 1, 
                                  facecolor='green', alpha=0.3))
        
        self.ax.grid(True)
        self.ax.set_xlim(-0.5, self.maze_grid.size+0.5)
        self.ax.set_ylim(-0.5, self.maze_grid.size+0.5)
        self.ax.set_aspect('equal')
        plt.pause(0.5)

def solve_maze(robot_class=DraftRobot):
    maze_grid = MazeGrid()
    robot = robot_class(maze_grid)
    
    if robot_class == DraftRobot:
        sim = Simulator(robot, maze_grid)
        sim.generate_random_maze()
    
    goal_found = False
    center = maze_grid.size // 2
    
    while not goal_found:
        if robot_class == DraftRobot:
            sim.draw()
            
        detected_walls = robot.scan_surrounding_walls()
        robot.update_grid_walls(detected_walls)
        available_moves = robot.get_available_moves()
        
        if not available_moves:
            print("No available moves - maze is unsolvable!")
            break
            
        next_x, next_y = robot.find_lowest_cost_move(available_moves)
        robot.move_to_cell(next_x, next_y)
        
        if (next_x, next_y) == (center, center):
            goal_found = True
            
        time.sleep(0.5)
            
    if robot_class == DraftRobot:
        sim.draw()
        plt.show()
    
    return "Maze solved!" if goal_found else "Maze unsolvable!"

if __name__ == "__main__":
    solve_maze(robot_class=DraftRobot)