
from keyes_Bit_Car_Driver import *

from collections import deque
import array

class MazeGrid:
    def __init__(self):
        self.size = 9
        # Using lists instead of numpy arrays
        self.costs = [[0 for _ in range(self.size)] for _ in range(self.size)]
        self.explored = [[False for _ in range(self.size)] for _ in range(self.size)]
        # walls[y][x] contains [N,E,S,W] walls
        self.walls = [[[True for _ in range(4)] for _ in range(self.size)] for _ in range(self.size)]
        self.known_walls = [[[False for _ in range(4)] for _ in range(self.size)] for _ in range(self.size)]
        self.initialize_cost_grid()

    def initialize_cost_grid(self):
        center = self.size // 2
        for i in range(self.size):
            for j in range(self.size):
                self.costs[i][j] = abs(i - center) + abs(j - center)

    def update_costs_flood_fill(self):
        # Initialize costs to high value (999 instead of infinity)
        new_costs = [[999 for _ in range(self.size)] for _ in range(self.size)]
        center = self.size // 2
        new_costs[center][center] = 0

        # Use queue for flood fill
        queue = deque([(center, center)])

        while queue:
            y, x = queue.popleft()
            current_cost = new_costs[y][x]

            # Check all four directions
            directions = [(0,-1), (1,0), (0,1), (-1,0)]  # N,E,S,W
            for i, (dy, dx) in enumerate(directions):
                new_y, new_x = y + dy, x + dx

                # Check bounds and walls
                if (0 <= new_x < self.size and 0 <= new_y < self.size and
                    not self.known_walls[y][x][i]):
                    new_cost = current_cost + 1

                    # Update if new cost is lower
                    if new_cost < new_costs[new_y][new_x]:
                        new_costs[new_y][new_x] = new_cost
                        queue.append((new_y, new_x))

        # Update costs, using manhattan distance for unreachable cells
        for y in range(self.size):
            for x in range(self.size):
                if new_costs[y][x] == 999:
                    center = self.size // 2
                    new_costs[y][x] = abs(y - center) + abs(x - center)

        self.costs = new_costs

    def mark_cell_explored(self, x, y):
        self.explored[y][x] = True

    def is_cell_explored(self, x, y):
        return self.explored[y][x]

    def set_wall(self, x, y, direction, value):
        """Set wall and maintain consistency with neighboring cells"""
        self.known_walls[y][x][direction] = value
        if direction == 0 and y < self.size-1:  # North
            self.known_walls[y+1][x][2] = value
        elif direction == 1 and x < self.size-1:  # East
            self.known_walls[y][x+1][3] = value
        elif direction == 2 and y > 0:  # South
            self.known_walls[y-1][x][0] = value
        elif direction == 3 and x > 0:  # West
            self.known_walls[y][x-1][1] = value

class Robot:
    def __init__(self, grid):
        self.maze_grid = grid
        self.position = [0, 0]

    def scan_surrounding_walls(self):
        # This should interface with actual robot sensors
        # Return array of 4 booleans for N,E,S,W walls
        # Example implementation (to be replaced with actual sensor code):

        wall_bool = [False, False, False, False]

        # N, if wall press A. Display E
        directions = ['N','E','S','W']
        for index in range(4):
            display.scroll(directions[index])
            while True:

                if button_a.is_pressed():
                    wall_bool[index] = True
                    display.scroll("wall")
                    break
                elif button_b.is_pressed():
                    wall_bool[index] = False
                    display.scroll("no wall")
                    break

        # Or dance

        return wall_bool



    def update_grid_walls(self, walls):
        x, y = self.position
        for direction in range(4):
            self.maze_grid.set_wall(x, y, direction, walls[direction])
        self.maze_grid.update_costs_flood_fill()

    def get_available_moves(self):
        x, y = self.position
        moves = []
        walls = self.maze_grid.known_walls[y][x]

        if not walls[0] and y < self.maze_grid.size-1: moves.append((x, y+1))
        if not walls[1] and x < self.maze_grid.size-1: moves.append((x+1, y))
        if not walls[2] and y > 0: moves.append((x, y-1))
        if not walls[3] and x > 0: moves.append((x-1, y))

        return moves

    def find_lowest_cost_move(self, available_moves):
        if not available_moves:
            return self.position

        unexplored_moves = [move for move in available_moves
                          if not self.maze_grid.is_cell_explored(*move)]
        moves_to_consider = unexplored_moves if unexplored_moves else available_moves

        # Find move with lowest cost
        min_cost = 999
        best_move = moves_to_consider[0]
        for x, y in moves_to_consider:
            cost = self.maze_grid.costs[y][x]
            if cost < min_cost:
                min_cost = cost
                best_move = (x, y)

        return best_move

    def move_to_cell(self, x, y):
        # This should interface with actual robot motors
        self.position = [x, y]
        self.maze_grid.mark_cell_explored(x, y)

def solve_maze():
    maze_grid = MazeGrid()
    robot = Robot(maze_grid)

    goal_found = False
    center = maze_grid.size // 2

    while not goal_found:
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

    return "Maze solved!" if goal_found else "Maze unsolvable!"

while True:
    # solve_maze() # To start maze solving

    # Need to test scan_surrounding_walls and move_to_cell
    maze_grid = MazeGrid()
    robot = Robot(maze_grid)

    detected_walls = robot.scan_surrounding_walls()

    display.scroll(detected_walls)
