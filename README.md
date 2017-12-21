# Occupancy Grid

You're very own occupancy grid module! Now all you need is a robot and a range sensor :)

# API

`init` to create a new instance. You can fine tune the parameters to your liking.

`update` when an obstacle is detected with the range sensor. Pass in the robot's location,
orientation from the x-axis, and obstacle distance.

`save` to store an image of the occupancy grid. Pass in the desired file path.