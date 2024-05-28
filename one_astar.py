import carla
import heapq
from carla import GlobalRoutePlanner, GlobalRoutePlannerDAO
import math

class Cell(object):
    def __init__(self, waypoint, reachable):
        """Initialize new cell.

        @param reachable is cell reachable? not a wall?
        @param x cell x coordinate
        @param y cell y coordinate
        @param g cost to move from the starting cell to this cell.
        @param h estimation of the cost to move from this cell
                 to the ending cell.
        @param f f = g + h
        """
        self.reachable = reachable
        self.waypoint = waypoint
        self.current_wp = waypoint.transform.location.current_wp
        self.start_wp = waypoint.transform.location.start_wp
        self.end_wp = waypoint.transform.location.end_wp
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.g < other.g


class AStar(object):
    def __init__(self):
        # open list
        self.opened = []
        heapq.heapify(self.opened)
        # visited cells list
        self.closed = set()
        # grid cells
        #cells contain
        self.cells = []
        #self.grid_height = None
        #self.grid_width = None
        self.start_wp = None
        self.end_wp = None

    def init_grid(self, current_wp, walls, start_wp, end_wp):
        """Prepare grid cells, walls.

        @param width grid's width.
        @param height grid's height.
        @param walls list of wall x,y tuples.
        @param start grid starting point x,y tuple.
        @param end grid ending point x,y tuple.

        if cells contain var, and current_wp is in cells list(contains all cells, 
        need to be evaluated), if current_wp is in walls-not reachable and marked for 
        either wall or obstacle, if reachable == true, current cell with wp is added 
to open set(sent for evaluation). Start and end wp are also initiated. 
        """
        #self.grid_height = height
        #self.grid_width = width
        if cells:
            for current_wp in cells:
                if current_wp in walls:
                    reachable = False
                else:
                    reachable = True
                    self.opened.append(Cell(current_wp, reachable))
        self.start_wp = self.get_cell(start_wp)
        self.end_wp = self.get_cell(end_wp)


    def get_heuristic(self, cell):
        one = cell.waypoint.transform.location
        two = self.end_wp.waypoint.transform.location
        return 10 * (abs(one.x - two.x) + abs(one.y - two.y) + abs(one.z - two.z))


    def get_cell(self, x, y, z):
        """Returns a cell from the cells list.

        @param x cell x coordinate
        @param y cell y coordinate
        @returns cell
        """
        index = x * self.grid_height * self.grid_depth + y * self.grid_depth + z
        return self.cells[index]


    def get_adjacent_cells(self, cell):
        opened = []
        x = cell.x
        y = cell.y
        z = cell.z
        """
        Accesses cells in the front of the current waypoint, back, up,
        left, and right
        z = depth
        y = height(in this case)
        x = width

        """

        if x < self.grid_width - 1:
            opened.append(self.get_cell(x + 1, y, z))
        if y > 0:
            opened.append(self.get_cell(x, y - 1, z))
        if x > 0:
            opened.append(self.get_cell(x - 1, y, z))
        if y < self.grid_height - 1:
            opened.append(self.get_cell(x, y + 1, z))
        if z > 0:
            opened.append(self.get_cell(x, y, z - 1))
        if z < self.grid_depth - 1:
            opened.append(self.get_cell(x, y, z + 1))

        return opened

    def get_path(self):
        cell = self.end_wp
        path = [(cell.x, cell.y, cell.z)]
        while cell.parent is not self.start_wp:
            cell = cell.parent
            path.append((cell.x, cell.y, cell.z))
        path.append((self.start_wp.x, self.start_wp.y, self.start_wp.z))
        path.reverse()
        return path

    def update_cell(self, adj, cell):
        adj.g = cell.g + 10
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g

    def solve(self, start_wp, end_wp, printing_grid):
        self.start_wp = self.get_cell_by_waypoint(start_wp)
        self.end_wp = self.get_cell_by_waypoint(end_wp)

        if not self.start_wp or not self.end_wp:
            return None

        heapq.heappush(self.opened, (self.start_wp.f, self.start_wp))
        printing_grid[self.start_wp.x, self.start_wp.y] = 2.0
        printing_grid[self.end_wp.x, self.end_wp.y] = 4.0

        while len(self.opened):
            _, cell = heapq.heappop(self.opened)
            printing_grid[cell.x, cell.y] = 3.0
            self.closed.add(cell)

            if cell is self.end_wp:
                return self.get_path()

            adj_cells = self.get_adjacent_cells(cell)
            for adj_cell in adj_cells:
                if adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        if adj_cell.g > cell.g + 10:
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))
        return None


class Main(object):
    def __init__(self):
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()
        self.map = self.world.get_map()

        dao = GlobalRoutePlannerDAO(self.map, 2)
        grp = GlobalRoutePlanner(dao)
        grp.setup()

        start = carla.Location(x=13.5, y=-13.7, z=0.0)
        end = carla.Location(x=180, y=240, z=0.5)

        self.route = grp.trace_route(start, end)
        self.waypoint_list = [route[0] for route in self.route]

    def visualize_path(self):
        for waypoint in self.waypoint_list:
            self.world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=300.0, persistent_lines=True)

    def find_path(self):
        astar = AStar()
        walls = []  # Define walls if any
        start_wp = self.waypoint_list[0]
        end_wp = self.waypoint_list[-1]

        astar.init_grid(self.waypoint_list, walls, start_wp, end_wp)

        printing_grid = []
        for i in range(200):
            row = []
            for j in range(200):
                row.append(0)
            printing_grid.append(row)

        path = astar.solve(start_wp, end_wp, printing_grid)
        if path:
            print("Path found:")
            for step in path:
                print(step)
        else:
            print("Path not found")


if __name__ == "__main__":
    main_instance = Main()
    main_instance.visualize_path()
    main_instance.find_path()
