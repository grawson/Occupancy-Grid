from OccGrid import OccGrid

grid = OccGrid()

print "Generating center..."
for theta in xrange(0, 360, 45):
    robot_pos = (0, 0)
    robot_curr_theta = theta
    grid.update(robot_pos, robot_curr_theta, 150)
grid.save("img/center.png")

print "Generating 135..."
grid = OccGrid()
for x in xrange(0, 100, 10):
    robot_pos = (x, x)
    robot_curr_theta = 135
    grid.update(robot_pos, robot_curr_theta, 50)
grid.save("img/135.png")

print "Generating 90..."
grid = OccGrid()
for x in xrange(0, 100, 10):
    robot_pos = (x, 0)
    robot_curr_theta = 90
    grid.update(robot_pos, robot_curr_theta, 50)
grid.save("img/90.png")
