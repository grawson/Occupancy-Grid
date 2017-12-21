import numpy as np
import math
import matplotlib.pyplot as plt

TOLERANCE = 1
RESOLUTION = (501, 501)  # NOTE: Both numbers must be odd
BEAM_WIDTH = 30
MAX_OCC = 0.98
MAX_SON_RNGE = 600


class OccGrid():

    # Resolution = map dimensions (must be odd)
    # tolerance = offset in distance measurement
    # beam width = width of sensor beam in degrees
    # max_occ = scalar
    # max_son_range = max range of the sensor
    def __init__(self, tolerance=TOLERANCE, resolution=RESOLUTION, beam_width=BEAM_WIDTH, max_occ=MAX_OCC, max_son_range=MAX_SON_RNGE):

        # ensure dimensions are odd
        dim = (
            resolution[0] + (((resolution[0] % 2) + 1) % 2),
            resolution[1] + (((resolution[1] % 2) + 1) % 2)
        )

        self.tolerance, self.resolution, self.beam_width, self.max_occ, self.max_son_range =\
            tolerance, resolution, beam_width, max_occ, max_son_range
        self.grid = [[0.50 for y in xrange(dim[0])] for x in xrange(dim[1])]


    def find_cone_points(self, cone_origin, hit_dist, rob_curr_theta):
        priors = list()
        cart_points = list()
        grid_points = list()
        hit_y = cone_origin[1] - hit_dist

        # consider cone origin as origin
        for y in xrange(hit_y-self.resolution[1], hit_y+self.resolution[1]):  # TODO: this is where the fix was, but now the range is huge
            for x in xrange(200):

                # convert cone point to grid point
                right_grid_point = (cone_origin[0] + x, cone_origin[1] - y)
                left_grid_point = (cone_origin[0] - x, cone_origin[1] - y)

                # Check right point (uncertainty cone)
                if self.in_uncertainty_cone((x, y), hit_dist) and self.in_grid(right_grid_point):  # TODO: Change angle offset based on orientation
                    rotat_grid_point = self.rotate(right_grid_point, rob_curr_theta, True)
                    if rotat_grid_point not in grid_points:
                        cart_points.append(self.cart2pol(x, y))
                        grid_points.append(rotat_grid_point)
                        priors.append(self.grid[rotat_grid_point[1]][rotat_grid_point[0]])

                # Check left point (uncertainty cone)
                if self.in_uncertainty_cone((-x, y), hit_dist) and self.in_grid(left_grid_point):  # TODO: Change angle offset based on orientation
                    rotat_grid_point = self.rotate(left_grid_point, rob_curr_theta, True)
                    if rotat_grid_point not in grid_points:
                        cart_points.append(self.cart2pol(-x, y))
                        grid_points.append(rotat_grid_point)
                        priors.append(self.grid[rotat_grid_point[1]][rotat_grid_point[0]])

        # Update prob
        for i in xrange(len(cart_points)):
            self.update_prob(grid_points[i], cart_points[i], priors[i])


    # Check if a point is in the uncertainty cone
    def in_uncertainty_cone(self, point, hit_dist):
        cart = self.cart2pol(point[0], point[1])
        return (cart[0] > hit_dist - self.tolerance and
                cart[0] < hit_dist + self.tolerance and
                abs(90-cart[1]) <= self.beam_width/2.0 and
                cart[1] >= 0)


    # Check if a point is in the cone but not in the uncertainty cone
    def in_cone(self, point, hit_dist):
        cart = self.cart2pol(point[0], point[1])
        return (cart[0] <= hit_dist - self.tolerance and
                abs(cart[1]-90) <= self.beam_width/2.0)


    # Update the prob for a point in the uncertainty cone
    def update_prob(self, grid_point, cart_point, occ_given_s0):
        left_num = (self.max_son_range - cart_point[0]) / float(self.max_son_range)
        right_num = (self.beam_width/2.0 - abs(90-cart_point[1])) / (self.beam_width/2.0)
        s1_given_occ = ((left_num + right_num) / 2.0) * self.max_occ
        s1_given_empty = 1 - s1_given_occ

        # priors
        empty_given_s0 = 1 - occ_given_s0

        occ_given_s1 = (s1_given_occ * occ_given_s0) / float(s1_given_occ * occ_given_s0 + s1_given_empty * empty_given_s0)
        self.grid[grid_point[1]][grid_point[0]] = occ_given_s1


    # converts a coordinate (with origin in center) to occupancy grid coordinate
    # With origin in top left corner
    def coord2og(self, coord):
        return (coord[0] + self.resolution[0]/2, coord[1] + self.resolution[1]/2)


    # Reverse of function above
    def og2coord(self, coord):
        return (coord[0] - self.resolution[0]/2, coord[1] - self.resolution[1]/2)


    # Rotate a point from world coord to occupancy grid system (and backwards)
    def rotate(self, point, curr_robot_theta, og_coord_system):
        theta = math.radians(-(curr_robot_theta-90))
        rot_mat = np.array([
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ])

        # convert origin to center if necessay
        if og_coord_system:
            point = self.og2coord(point)

        pos_mat = np.array([
            [1, 0, point[0]],
            [0, 1, point[1]],
            [0, 0, 1]
        ])

        # Rotate position
        rotated_pos = np.matmul(rot_mat, pos_mat)
        pos = (rotated_pos[0][2], rotated_pos[1][2])

        # convert back to origin in top left
        pos = self.coord2og(pos)
        return (int(pos[0]), int(pos[1]))


    # Check if a point is within the grid bounds
    def in_grid(self, point):
        return (point[0] < self.resolution[0] and point[1] < self.resolution[1]
                and point[0] >= 0 and point[1] >= 0)


    # Convert cartesian point to polar coord
    def cart2pol(self, x, y):
        r = np.sqrt(x**2+y**2)
        theta = math.degrees(np.arctan2(y, x))
        return(r, theta)


    # PUBLIC ##################################################################


    # Update the occupancy grid when the robot has seen an object
    # robot_pos = (x, y) tuple
    # robot_curr_theta = orientation from x axis in degrees
    # hit_dist = distance at which an obstacle was detected
    def update(self, robot_pos, robot_curr_theta, hit_dist):
        assert(hit_dist > 0)
        pos = self.rotate(robot_pos, robot_curr_theta, False)
        self.find_cone_points(pos, hit_dist, robot_curr_theta)


    # Save the grid as a greyscale image
    # filename = file path
    def save(self, filename):
        img = np.array(self.grid)
        img = np.dot(255, self.grid)
        print img[0]
        plt.imsave(filename, img, cmap="gray")
