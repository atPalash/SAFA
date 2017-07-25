from basicRobot_planner import PathPlanner

grid = PathPlanner()
grid.set_grid([1, 1])
grid.set_grid([0, 1])
grid.set_grid([2, 1])
grid.set_grid([3, 1])
print grid.optimum_policy()
grid.reset_grid([3,1])
print grid.optimum_policy()