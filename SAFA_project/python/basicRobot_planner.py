class PathPlanner:
    def __init__(self):
        self.grid = [[0, 0, 0, 0],
                     [0, 0, 0, 0],
                     [0, 0, 0, 0],
                     [0, 0, 0, 0]]
        self.goal = [0, 0]
        self.cost = 1  # the cost associated with moving from a cell to an adjacent one
        self.delta = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1]]  # go right
        self.delta_name = ['U', 'L', 'D', 'R']

    def set_initial_grid(self, initial_grid):
        self.grid = initial_grid

    def get_grid(self):
        return self.grid

    def set_grid(self, obstruction_coordinates):
        self.grid[obstruction_coordinates[0]][obstruction_coordinates[1]] = 1

    def reset_grid(self, obstruction_coordinates):
        # print obstruction_coordinates
        for i in range(obstruction_coordinates[0]):
            # print i
            self.grid[i][obstruction_coordinates[1]] = 0

    def set_goal(self, goal_coordinates):
        self.goal[0] = goal_coordinates[0]
        self.goal[1] = goal_coordinates[1]

    def optimum_policy(self):
        grid = self.grid
        goal = self.goal
        cost = self.cost
        delta = self.delta
        delta_name = self.delta_name
        value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
        policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
        change = True

        while change:
            change = False

            for x in range(len(grid)):
                for y in range(len(grid[0])):
                    if goal[0] == x and goal[1] == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            policy[x][y] = "*"
                            change = True

                    elif grid[x][y] == 0:
                        for a in range(len(delta)):
                            x2 = x + delta[a][0]
                            y2 = y + delta[a][1]

                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                                v2 = value[x2][y2] + cost
                                # print x, y, a, x2, y2, v2, value[x2][y2]
                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    policy[x][y] = delta_name[a]
                                    # print value[x][y], policy[x][y]

        # for i in range(len(value)):
        #     print policy[i]
        return policy
