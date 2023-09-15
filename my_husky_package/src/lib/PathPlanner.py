import sys
from math import cos


from numpy import deg2rad, rad2deg
from shapely import Polygon, LineString, Point

import matplotlib.pyplot as plt
import numpy as np


class PathPlanner:

    def __init__(self, path_width: float, vertex_1: tuple, vertex_2: tuple,
                 bounding_vertices: list):

        # Input: Path width, the two vertices that corresponds to the base line
        # and the vertices that bounds the polygon as a list of tuples
        self.path_width = path_width
        self.vertex_1 = vertex_1
        self.vertex_2 = vertex_2
        self.bounding_vertices = bounding_vertices

        # Creating the polygon
        self.poly = Polygon(bounding_vertices)

        # Creating the path that will be exported. This is a list of tuples
        self.path = list()

    def get_path(self):

      
        #Creating the vector so we can get the angle
        # We want the vector to be from the lowest point to the highest point
        x_1 = self.vertex_1[0]
        y_1 = self.vertex_1[1]
        x_2 = self.vertex_2[0]
        y_2 = self.vertex_2[1]
        
        if y_1 < y_2:
            self.delta_x = x_2 - x_1
            self.delta_y = y_2 - y_1
        else:
            self.delta_x = x_1 - x_2
            self.delta_y = y_1 - y_2
            
        self.vertex_vector = [self.delta_x, self.delta_y]
                
        # The slope of the vector is:
        #   Distance traveled along the y-axis
        # -------------divided by-----------------------
        #   Distance traveled along the x-axis
        if self.delta_x == 0:
            self.slope = 0
        else:
            self.slope = self.delta_y/self.delta_x

        # The intercept along the y axis.
        self.y_intercept = self.vertex_1[1] - self.slope * self.vertex_1[0]

        # Initializing the unit vectors of the x and y-axis respectively
        self.unit_vector_i = [1, 0]
        self.unit_vector_j = [0, 1]

        # Getting the angle between the x-plane and the baseplane that is vertex_vector
        self.theta = rad2deg(self.angle_between(self.vertex_vector, self.unit_vector_i))

        # The angle of the right-angled triangle created between parallel lines
        self.phi = 90 - self.theta
        

        
        

        # Getting the translation along the x- or y-axis for creating the parallel lines
        # We will create parallel lines along the baseline with a width of half the path width.
        
        if self.theta == 90 or self.theta == 180 or self.theta == 0:
            self.translation = self.path_width / 2
        else:
            self.translation = (self.path_width / 2) / cos(deg2rad(self.phi))

        # Creating the inner polygon. A negative number indicates shrinking
        self.poly_inner = self.poly.buffer((self.path_width*-1)/2)

        # Getting the max parameters the polygons
        self.minx_1, self.miny_1, self.maxx_1, self.maxy_1 = self.poly.bounds
        self.minx_2, self.miny_2, self.maxx_2, self.maxy_2 = self.poly_inner.bounds

        # Creating a polygon that represents the extreme boundaries of the area
        self.poly_outer_extremes = Polygon([(self.minx_1, self.miny_1), (self.minx_1, self.maxy_1), (self.maxx_1, self.maxy_1), (self.maxx_1, self.miny_1)])

        
        
        # For the horizontal movement
        # Finding if we are going left to right, or right to left.

        self.left_to_right = bool

        if max(self.vertex_2[0], self.vertex_1[0]) == self.maxx_1:
            self.left_to_right = False
        else:
            self.left_to_right = True
            
        # For the vertical movement
        self.down_to_up = bool
        
        if max(self.vertex_2[1], self.vertex_1[1]) == self.maxy_1:
            self.down_to_up = False
        else:
            self.down_to_up = True


        # # Checks the orientation of the starting line and takes the appropriate action
        # # There is a big enough difference in the angles to justify this, as i am not a smart enough man to make an algorithm that works for all cases
        # if self.phi > 0 and self.phi < 45 and self.left_to_right:
        #     self.left_to_right_acute() # Theta 63, left to right acute
        # elif self.phi > 0 and self.phi < 45 and not(self.left_to_right):
        #     self.right_to_left_acute() # Theta 63, right to left acute, actually acute
        # elif self.phi < 0 and self.phi > -45 and self.left_to_right:
        #     self.left_to_right_obuse() # Theta 116, left to right obuse
        # elif self.phi < 0 and self.phi > -45 and not(self.left_to_right):
        #     self.right_to_left_obuse() # Theta 116, right to left acute, Actually obuse
        # elif self.phi == 0 and self.left_to_right:
        #     self.left_to_right_right_angled()   # Theta 90, left to right right angled
        # elif self.phi == 0 and not(self.left_to_right):
        #     self.right_to_left_right_angled() # Theta 90, right to left right angled
        # elif self.phi > 45 and self.phi < 90 and self.down_to_up:
        #     self.down_to_up_acute() # Theta 18, bottom to top acute.
        # elif self.phi > 45 and self.phi < 90 and not(self.down_to_up):
        #     self.up_to_down_acute() # Theta 18, top to bottom obuse, acually acute. 
        # elif self.phi < -45 and self.phi > -90 and self.down_to_up:
        #     self.down_to_up_obuse() # Theta 161, bottom to top acute, actually obuse
        # elif self.phi < -45 and self.phi > -90 and not(self.down_to_up):
        #     self.up_to_down_obuse() # Theta 161, top to bottom obuse
        # elif self.phi == 45 and self.down_to_up:
        #     pass    # Theta 180, bottom to top right angled
        # elif self.phi == 45 and not(self.down_to_up):
        #     pass    # Theta 180, bottom to top right angled
        
        # Checks the orientation of the starting line and takes the appropriate action
        if self.theta > 0 and self.theta < 45:
            # Startign line is acute and horizontal
            if self.down_to_up:
                self.down_to_up_acute() # Done
            else:
                self.up_to_down_acute() # Done
        elif self.theta == 45:
            # Starting line is acute and between vertical and horizontal
            pass # Not implemented yet
        elif self.theta > 45 and self.theta < 90:
            # Starting line 
            if self.left_to_right:
                self.left_to_right_acute() # - Done
            else:
                self.right_to_left_acute() # - Done
        elif self.theta == 90:
            #Starting line is purely vertical
            if self.left_to_right:
                self.left_to_right_right_angled()
            else:
                self.right_to_left_right_angled()
        elif self.theta > 90 and self.theta < 135:
            #Starting line is obuse and horizontal
            if self.left_to_right:
                self.left_to_right_obuse() # Done
            else:
                self.right_to_left_obuse() # Done
        elif self.theta == 135:
            #Starting line is obtuse and between vertical and horizontal
            pass # Not implemented yet
        elif self.theta > 135 and self.theta < 180:
            #Starting line is obuse and vertical
            if self.down_to_up:
                self.down_to_up_obuse() # Done
            else:
                self.up_to_down_obuse() # Done
        elif self.theta == 180:
            #Starting line is purely horizontal
            if self.down_to_up:
                self.down_to_up_right_angled() # Done
            else:
                self.up_to_down_right_angled()  


        # This is for vizualising what is happening
        # Plotting original area
        x, y = self.poly.exterior.xy
        ##plt.plot(x, y)
        # Plotting inner area
        x, y = self.poly_inner.exterior.xy
        ##plt.plot(x, y)
        # Plotting outer ekstremes
        x, y = self.poly_outer_extremes.exterior.xy
        ##plt.plot(x, y)
        # plotting lines
        ##plt.plot(*self.line_base_line_intersection.xy)
        # #plt.plot(*self.line_intersection_outer_extreme.xy)
        # #plt.plot(*line_paralell_horizontal_movement.xy)
        # Plotting the path
        ##plt.plot(*zip(*self.path))
        
        #Plotting the given vertices
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        

        ##plt.show()

        #Write path to file
        with open('sweeping_path.txt', 'w') as file:
            for i in range(len(self.path)):
                file.write(f"{self.path[i][0]},{self.path[i][1]}\n")
                
        return self.path


    def left_to_right_acute(self):
        
        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + (-100), self.y_intercept + self.slope * (self.vertex_1[0] + (-100))),
             (self.vertex_2[0] + 100, self.y_intercept + self.slope * (self.vertex_2[0] + 100))])
        ##plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_x_inf = -1000000
        max_x_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_x = Polygon([(min_x_inf, self.miny_1), (min_x_inf, self.maxy_1), (max_x_inf, self.maxy_1), (max_x_inf, self.miny_1)])
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_x.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.x2_outer_extreme + self.translation, self.maxy_1),
                          (self.x1_outer_extreme + self.translation, self.miny_1)])

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])

        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.x2_outer_extreme + 2 * self.translation, self.maxy_1),
                        (self.x1_outer_extreme + 2 * self.translation, self.miny_1)])

        # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y2_intersection_base_line),
                                (self.x2_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))

        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.x2_outer_extreme + i, self.maxy_1), 
                            (self.x1_outer_extreme + i, self.miny_1)])

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y2_up_down))
                self.path.append((x1_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.x2_outer_extreme + i + self.translation, self.maxy_1),
                                (self.x1_outer_extreme + i + self.translation, self.miny_1)])

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y1_up_down),
                                       (x1_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y1_down_up))
                    self.path.append((x2_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.x2_outer_extreme + i + 2 * self.translation, self.maxy_1),
                                    (self.x1_outer_extreme + i + 2 * self.translation, self.miny_1)])

                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y2_down_up),
                                       (x2_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation

        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxx of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            x_value = poly_inner_exterior_points[0][i]
            if x_value == self.maxx_2:
                y_value = poly_inner_exterior_points[1][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_down_up, y1_down_up),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                #print(path)
                pass


    def right_to_left_obuse(self):

        self.translation = self.translation * -1

        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + 100, self.y_intercept + self.slope * (self.vertex_1[0] + 100)),
             (self.vertex_2[0] - 100, self.y_intercept + self.slope * (self.vertex_2[0] - 100))])
        ##plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_x_inf = -1000000
        max_x_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_x = Polygon([(min_x_inf, self.miny_1), (min_x_inf, self.maxy_1), (max_x_inf, self.maxy_1), (max_x_inf, self.miny_1)])
        
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_x.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.x1_outer_extreme + self.translation, self.maxy_1),
                          (self.x2_outer_extreme + self.translation, self.miny_1)])
        # #plt.plot(*self.line_intersection_outer_extreme.xy)
        
        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.poly_inner.intersection(self.line_intersection_outer_extreme).boundary.bounds

        # Saves to path
        self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])

        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.x1_outer_extreme + 2 * self.translation, self.maxy_1),
                        (self.x2_outer_extreme + 2 * self.translation, self.miny_1)])
            
        ##plt.plot(*self.line_paralell_horizontal_movement.xy)

        # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y2_intersection_base_line),
                                (self.x1_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))

        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.x2_outer_extreme + i, self.miny_1), (self.x1_outer_extreme + i, self.maxy_1)])
                
            ##plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x1_up_down, y2_up_down))
                self.path.append((x2_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.x2_outer_extreme + i + self.translation, self.miny_1),
                                (self.x1_outer_extreme + i + self.translation, self.maxy_1)])
                ##plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_up_down, y1_up_down),
                                       (x2_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x2_down_up, y1_down_up))
                    self.path.append((x1_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.x2_outer_extreme + i + 2 * self.translation, self.miny_1),
                                    (self.x1_outer_extreme + i + 2 * self.translation, self.maxy_1)])
                    ##plt.plot(*line_up_down_second.xy)

                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_down_up, y2_down_up),
                                       (x1_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation

        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxx of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            x_value = poly_inner_exterior_points[0][i]
            if x_value == self.minx_2:
                y_value = poly_inner_exterior_points[1][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))



        else:

            # Creating the crossing points:
            crossing_points = [(x1_down_up, y1_down_up),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


    def left_to_right_obuse(self):
        
                # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + (-100), self.y_intercept + self.slope * (self.vertex_1[0] + (-100))),
             (self.vertex_2[0] + 100, self.y_intercept + self.slope * (self.vertex_2[0] + 100))])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_x_inf = -1000000
        max_x_inf = 1000000
        
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_x = Polygon([(min_x_inf, self.miny_1), (min_x_inf, self.maxy_1), (max_x_inf, self.maxy_1), (max_x_inf, self.miny_1)])
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_x.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.x2_outer_extreme + self.translation, self.miny_1),
                          (self.x1_outer_extreme + self.translation, self.maxy_1)])
        # #plt.plot(*self.line_intersection_outer_extreme.xy)
        
        # # Gives the line a buffer, so it can better intersect with the inner polygon
        # self.line_intersection_outer_extreme.buffer(200)

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        # self.path.append((self.x1_intersection_base_line, self.y2_intersection_base_line))
        # self.path.append((self.x2_intersection_base_line, self.y1_intersection_base_line))
        

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])
        # #plt.plot(*self.line_base_line_intersection.xy)

        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.x1_outer_extreme + 2 * self.translation, self.maxy_1),
                        (self.x2_outer_extreme + 2 * self.translation, self.miny_1)])
        ##plt.plot(*self.line_paralell_horizontal_movement.xy)

        # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x1_intersection_base_line, self.y2_intersection_base_line),
                                (self.x2_horizontal_line, self.y1_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))

        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.x2_outer_extreme + i, self.miny_1), (self.x1_outer_extreme + i, self.maxy_1)])
            # #plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y1_up_down))
                self.path.append((x1_up_down, y2_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.x2_outer_extreme + i + self.translation, self.miny_1),
                                (self.x1_outer_extreme + i + self.translation, self.maxy_1)])
                # #plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y2_up_down),
                                       (x1_down_up, y2_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y2_down_up))
                    self.path.append((x2_down_up, y1_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.x2_outer_extreme + i + 2 * self.translation, self.miny_1),
                                    (self.x1_outer_extreme + i + 2 * self.translation, self.maxy_1)])
                    # #plt.plot(*line_up_down_second.xy)

                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_down_up, y1_down_up),
                                       (x2_up_down_second, y1_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation

        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxx of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            x_value = poly_inner_exterior_points[0][i]
            if x_value == self.maxx_2:
                y_value = poly_inner_exterior_points[1][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_down_up, y1_down_up),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                #print(path)
                pass

    def right_to_left_acute(self):
        
        self.translation = self.translation * -1

        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + 100, self.y_intercept + self.slope * (self.vertex_1[0] + 100)),
             (self.vertex_2[0] - 100, self.y_intercept + self.slope * (self.vertex_2[0] - 100))])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_x_inf = -1000000
        max_x_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_x = Polygon([(min_x_inf, self.miny_1), (min_x_inf, self.maxy_1), (max_x_inf, self.maxy_1), (max_x_inf, self.miny_1)])
        
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_x.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.x1_outer_extreme + self.translation, self.maxy_1),
                          (self.x2_outer_extreme + self.translation, self.miny_1)])
        # #plt.plot(*self.line_intersection_outer_extreme.xy)
        
        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.poly_inner.intersection(self.line_intersection_outer_extreme).boundary.bounds

        # Saves to path
        # self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        # self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])

        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.x1_outer_extreme + 2 * self.translation, self.miny_1),
                        (self.x2_outer_extreme + 2 * self.translation, self.maxy_1)])
            
        # #plt.plot(*self.line_paralell_horizontal_movement.xy)

        # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(np.NaN, np.NaN),
                                (self.x2_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))

        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.x2_outer_extreme + i, self.maxy_1), (self.x1_outer_extreme + i, self.miny_1)])
                
            # #plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y2_up_down))
                self.path.append((x1_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.x2_outer_extreme + i + self.translation, self.maxy_1),
                                (self.x1_outer_extreme + i + self.translation, self.miny_1)])
                ##plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y1_up_down),
                                       (x1_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y1_down_up))
                    self.path.append((x2_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.x2_outer_extreme + i + 2 * self.translation, self.maxy_1),
                                    (self.x1_outer_extreme + i + 2 * self.translation, self.miny_1)])
                    ##plt.plot(*line_up_down_second.xy)

                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y2_down_up),
                                       (x2_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation

        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxx of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            x_value = poly_inner_exterior_points[0][i]
            if x_value == self.minx_2:
                y_value = poly_inner_exterior_points[1][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))



        else:

            # Creating the crossing points:
            crossing_points = [(x1_down_up, y1_down_up),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

    def left_to_right_right_angled(self):
        
        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0], self.vertex_1[1] - 100),
             (self.vertex_2[0], self.vertex_2[1] + 100)])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_x_inf = -1000000
        max_x_inf = 1000000
        #Creating a new outer extreme polygon, with "infinite" length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_x = Polygon([(min_x_inf, self.miny_1), (min_x_inf, self.maxy_1), (max_x_inf, self.maxy_1), (max_x_inf, self.miny_1)])
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_x.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.x2_outer_extreme + self.translation, self.maxy_1),
                          (self.x1_outer_extreme + self.translation, self.miny_1)])

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])

        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.x2_outer_extreme + 2 * self.translation, self.maxy_1),
                        (self.x1_outer_extreme + 2 * self.translation, self.miny_1)])

        # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y2_intersection_base_line),
                                (self.x2_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))

        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.x2_outer_extreme + i, self.maxy_1), (self.x1_outer_extreme + i, self.miny_1)])

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y2_up_down))
                self.path.append((x1_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.x2_outer_extreme + i + self.translation, self.maxy_1),
                                (self.x1_outer_extreme + i + self.translation, self.miny_1)])

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y1_up_down),
                                       (x1_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y1_down_up))
                    self.path.append((x2_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.x2_outer_extreme + i + 2 * self.translation, self.maxy_1),
                                    (self.x1_outer_extreme + i + 2 * self.translation, self.miny_1)])

                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y2_down_up),
                                       (x2_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation

        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxx of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            x_value = poly_inner_exterior_points[0][i]
            if x_value == self.maxx_2:
                y_value = poly_inner_exterior_points[1][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_down_up, y1_down_up),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                #print(path)
                pass
        
        
    def right_to_left_right_angled(self):
        
        self.translation = self.translation * -1

        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0], self.vertex_1[1] + 100),
             (self.vertex_2[0], self.vertex_2[1] - 100)])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_x_inf = -1000000
        max_x_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_x = Polygon([(min_x_inf, self.miny_1), (min_x_inf, self.maxy_1), (max_x_inf, self.maxy_1), (max_x_inf, self.miny_1)])
        
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_x.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.x1_outer_extreme + self.translation, self.maxy_1),
                          (self.x2_outer_extreme + self.translation, self.miny_1)])
        # #plt.plot(*self.line_intersection_outer_extreme.xy)
        
        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.poly_inner.intersection(self.line_intersection_outer_extreme).boundary.bounds

        # Saves to path
        # self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        # self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])

        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.x1_outer_extreme + 2 * self.translation, self.miny_1),
                        (self.x2_outer_extreme + 2 * self.translation, self.maxy_1)])
            
        # #plt.plot(*self.line_paralell_horizontal_movement.xy)

        # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(np.NaN, np.NaN),
                                (self.x2_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))

        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.x2_outer_extreme + i, self.maxy_1), (self.x1_outer_extreme + i, self.miny_1)])
                
            # #plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y2_up_down))
                self.path.append((x1_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.x2_outer_extreme + i + self.translation, self.maxy_1),
                                (self.x1_outer_extreme + i + self.translation, self.miny_1)])
                ##plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y1_up_down),
                                       (x1_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y1_down_up))
                    self.path.append((x2_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.x2_outer_extreme + i + 2 * self.translation, self.maxy_1),
                                    (self.x1_outer_extreme + i + 2 * self.translation, self.miny_1)])
                    ##plt.plot(*line_up_down_second.xy)

                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y2_down_up),
                                       (x2_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation

        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxx of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            x_value = poly_inner_exterior_points[0][i]
            if x_value == self.minx_2:
                y_value = poly_inner_exterior_points[1][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))



        else:

            # Creating the crossing points:
            crossing_points = [(x1_down_up, y1_down_up),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

    def down_to_up_acute(self):
        
        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + (-100), self.y_intercept + self.slope * (self.vertex_1[0] + (-100))),
             (self.vertex_2[0] + 100, self.y_intercept + self.slope * (self.vertex_2[0] + 100))])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_y_inf = -1000000
        max_y_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_y = Polygon([(self.minx_1, min_y_inf), (self.maxx_1, min_y_inf), (self.maxx_1, max_y_inf), (self.minx_1, max_y_inf)])
        ##plt.plot(*self.poly_outer_extremes_inf_y.exterior.xy)
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_y.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.maxx_1, self.y2_outer_extreme + self.translation),
                          (self.minx_1, self.y1_outer_extreme + self.translation)])
        # #plt.plot(*self.line_intersection_outer_extreme.xy)

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        # self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        # self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))
        
        self.x2_intersection_base_line = np.NaN
        self.y2_intersection_base_line = np.NaN

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])
        # #plt.plot(*self.line_base_line_intersection.xy)
        
        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.maxx_1, self.y2_outer_extreme + 2* self.translation),
                        (self.minx_1, self.y1_outer_extreme + 2* self.translation)])
        # #plt.plot(*self.line_paralell_horizontal_movement.xy)
        
         # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y1_intersection_base_line),
                                (self.x2_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))
            
        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.maxx_1, self.y2_outer_extreme + i),
                            (self.minx_1, self.y1_outer_extreme + i)])
            # #plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y2_up_down))
                self.path.append((x1_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.maxx_1, self.y2_outer_extreme + i + self.translation),
                                (self.minx_1, self.y1_outer_extreme + i + self.translation)])
                ##plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y1_up_down),
                                       (x1_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y1_down_up))
                    self.path.append((x2_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.maxx_1, self.y2_outer_extreme + i + 2 * self.translation),
                                    (self.minx_1, self.y1_outer_extreme + i + 2 * self.translation)])
                    # #plt.plot(*line_up_down_second.xy)
                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y2_down_up),
                                       (x2_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation


        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxy of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            y_value = poly_inner_exterior_points[1][i]
            if y_value == self.maxy_2:
                x_value = poly_inner_exterior_points[0][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            # horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                #print(path)
                pass

    def up_to_down_acute(self): 
        
        self.translation = self.translation * -1
        
        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + (-100), self.y_intercept + self.slope * (self.vertex_1[0] + (-100))),
             (self.vertex_2[0] + 100, self.y_intercept + self.slope * (self.vertex_2[0] + 100))])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_y_inf = -1000000
        max_y_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_y = Polygon([(self.minx_1, min_y_inf), (self.maxx_1, min_y_inf), (self.maxx_1, max_y_inf), (self.minx_1, max_y_inf)])
        ##plt.plot(*self.poly_outer_extremes_inf_y.exterior.xy)
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_y.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.minx_1, self.y1_outer_extreme + self.translation),
                          (self.maxx_1, self.y2_outer_extreme + self.translation)])
        # #plt.plot(*self.line_intersection_outer_extreme.xy)

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        # self.path.append((self.x1_intersection_base_line, self.y2_intersection_base_line))
        # self.path.append((self.x2_intersection_base_line, self.y1_intersection_base_line))
        self.x1_intersection_base_line = np.NaN
        self.y1_intersection_base_line = np.NaN

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])
        #plt.plot(*self.line_base_line_intersection.xy)
        
        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.minx_1, self.y1_outer_extreme + 2* self.translation),
                        (self.maxx_1, self.y2_outer_extreme + 2* self.translation)])
        #plt.plot(*self.line_paralell_horizontal_movement.xy)
        
         # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y2_intersection_base_line),
                                (self.x2_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))
            
        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.minx_1, self.y1_outer_extreme + i),
                            (self.maxx_1, self.y2_outer_extreme + i)])
            #plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y2_up_down))
                self.path.append((x1_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.minx_1, self.y1_outer_extreme + i + self.translation),
                                (self.maxx_1, self.y2_outer_extreme + i + self.translation)])
                #plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y1_up_down),
                                       (x1_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y1_down_up))
                    self.path.append((x2_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.minx_1, self.y1_outer_extreme + i + 2 * self.translation),
                                    (self.maxx_1, self.y2_outer_extreme + i + 2 * self.translation)])
                    #plt.plot(*line_up_down_second.xy)
                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y2_down_up),
                                       (x2_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation


        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxy of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            y_value = poly_inner_exterior_points[1][i]
            if y_value == self.miny_2:
                x_value = poly_inner_exterior_points[0][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                #print(path)
                pass


    def down_to_up_obuse(self):
        
        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + (-100), self.y_intercept + self.slope * (self.vertex_1[0] + (-100))),
             (self.vertex_2[0] + 100, self.y_intercept + self.slope * (self.vertex_2[0] + 100))])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_y_inf = -1000000
        max_y_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_y = Polygon([(self.minx_1, min_y_inf), (self.maxx_1, min_y_inf), (self.maxx_1, max_y_inf), (self.minx_1, max_y_inf)])
        ##plt.plot(*self.poly_outer_extremes_inf_y.exterior.xy)
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_y.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.maxx_1, self.y1_outer_extreme + self.translation),
                          (self.minx_1, self.y2_outer_extreme + self.translation)])
        #plt.plot(*self.line_intersection_outer_extreme.xy)

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        self.path.append((self.x2_intersection_base_line, self.y1_intersection_base_line))
        self.path.append((self.x1_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y2_intersection_base_line),
                        (self.x2_intersection_base_line, self.y1_intersection_base_line)])
        #plt.plot(*self.line_base_line_intersection.xy)
        
        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.maxx_1, self.y1_outer_extreme + 2* self.translation),
                        (self.minx_1, self.y2_outer_extreme + 2* self.translation)])
        #plt.plot(*self.line_paralell_horizontal_movement.xy)
        
         # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x1_intersection_base_line, self.y2_intersection_base_line),
                                (self.x1_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))
            
        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.maxx_1, self.y1_outer_extreme + i),
                            (self.minx_1, self.y2_outer_extreme + i)])
            #plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x1_up_down, y2_up_down))
                self.path.append((x2_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.maxx_1, self.y1_outer_extreme + i + self.translation),
                                (self.minx_1, self.y2_outer_extreme + i + self.translation)])
                #plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_up_down, y1_up_down),
                                       (x2_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x2_down_up, y1_down_up))
                    self.path.append((x1_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.maxx_1, self.y1_outer_extreme + i + 2 * self.translation),
                                    (self.minx_1, self.y2_outer_extreme + i + 2 * self.translation)])
                    #plt.plot(*line_up_down_second.xy)
                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_down_up, y2_down_up),
                                       (x1_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation


        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxy of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            y_value = poly_inner_exterior_points[1][i]
            if y_value == self.maxy_2:
                x_value = poly_inner_exterior_points[0][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                #print(path)
                pass

    def up_to_down_obuse(self):
        
        self.translation = self.translation * -1
        
        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + (-100), self.y_intercept + self.slope * (self.vertex_1[0] + (-100))),
             (self.vertex_2[0] + 100, self.y_intercept + self.slope * (self.vertex_2[0] + 100))])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_y_inf = -1000000
        max_y_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_y = Polygon([(self.minx_1, min_y_inf), (self.maxx_1, min_y_inf), (self.maxx_1, max_y_inf), (self.minx_1, max_y_inf)])
        ##plt.plot(*self.poly_outer_extremes_inf_y.exterior.xy)
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_y.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.maxx_1, self.y1_outer_extreme + self.translation),
                          (self.minx_1, self.y2_outer_extreme + self.translation)])
        #plt.plot(*self.line_intersection_outer_extreme.xy)

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])
        #plt.plot(*self.line_base_line_intersection.xy)
        
        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.maxx_1, self.y1_outer_extreme + 2* self.translation),
                        (self.minx_1, self.y2_outer_extreme + 2* self.translation)])
        #plt.plot(*self.line_paralell_horizontal_movement.xy)
        
         # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y1_intersection_base_line),
                                (self.x2_horizontal_line, self.y1_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))
            
        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.maxx_1, self.y1_outer_extreme + i),
                            (self.minx_1, self.y2_outer_extreme + i)])
            #plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y1_up_down))
                self.path.append((x1_up_down, y2_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.maxx_1, self.y1_outer_extreme + i + self.translation),
                                (self.minx_1, self.y2_outer_extreme + i + self.translation)])
                #plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y2_up_down),
                                       (x1_down_up, y2_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y2_down_up))
                    self.path.append((x2_down_up, y1_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.maxx_1, self.y1_outer_extreme + i + 2 * self.translation),
                                    (self.minx_1, self.y2_outer_extreme + i + 2 * self.translation)])
                    #plt.plot(*line_up_down_second.xy)
                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y1_down_up),
                                       (x2_up_down_second, y1_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation


        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxy of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            y_value = poly_inner_exterior_points[1][i]
            if y_value == self.maxy_2:
                x_value = poly_inner_exterior_points[0][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                #print(path)
                pass

    def down_to_up_right_angled(self):
        self.translation = self.translation
        
        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + (-100), self.vertex_1[1]),
             (self.vertex_2[0] + 100, self.vertex_2[1]) ])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_y_inf = -1000000
        max_y_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_y = Polygon([(self.minx_1, min_y_inf), (self.maxx_1, min_y_inf), (self.maxx_1, max_y_inf), (self.minx_1, max_y_inf)])
        ##plt.plot(*self.poly_outer_extremes_inf_y.exterior.xy)
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_y.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.maxx_1, self.y1_outer_extreme + self.translation),
                          (self.minx_1, self.y2_outer_extreme + self.translation)])
        #plt.plot(*self.line_intersection_outer_extreme.xy)

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])
        #plt.plot(*self.line_base_line_intersection.xy)
        
        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.maxx_1, self.y1_outer_extreme + 2* self.translation),
                        (self.minx_1, self.y2_outer_extreme + 2* self.translation)])
        #plt.plot(*self.line_paralell_horizontal_movement.xy)
        
         # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y1_intersection_base_line),
                                (self.x2_horizontal_line, self.y1_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))
            
        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.maxx_1, self.y1_outer_extreme + i),
                            (self.minx_1, self.y2_outer_extreme + i)])
            #plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y1_up_down))
                self.path.append((x1_up_down, y2_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.maxx_1, self.y1_outer_extreme + i + self.translation),
                                (self.minx_1, self.y2_outer_extreme + i + self.translation)])
                #plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y2_up_down),
                                       (x1_down_up, y2_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y2_down_up))
                    self.path.append((x2_down_up, y1_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.maxx_1, self.y1_outer_extreme + i + 2 * self.translation),
                                    (self.minx_1, self.y2_outer_extreme + i + 2 * self.translation)])
                    #plt.plot(*line_up_down_second.xy)
                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y1_down_up),
                                       (x2_up_down_second, y1_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation


        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxy of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            y_value = poly_inner_exterior_points[1][i]
            if y_value == self.maxy_2:
                x_value = poly_inner_exterior_points[0][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                #print(path)
                pass
    
    def up_to_down_right_angled(self):
        self.translation = self.translation * -1
        
        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(self.vertex_1[0] + (-100),self.vertex_1[1]),
             (self.vertex_2[0] + 100, self.vertex_2[1])])
        # #plt.plot(*self.line_vertex_elongated.xy)
        
        #defining a max and min value for the x-direction, 1 000 000 is bigger than the range of UTM coordinates
        min_y_inf = -1000000
        max_y_inf = 1000000
        #Creating a new outer extreme polygon, with infinite length in x-direction while mainting the y-values
        self.poly_outer_extremes_inf_y = Polygon([(self.minx_1, min_y_inf), (self.maxx_1, min_y_inf), (self.maxx_1, max_y_inf), (self.minx_1, max_y_inf)])
        ##plt.plot(*self.poly_outer_extremes_inf_y.exterior.xy)
        
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes_inf_y.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.maxx_1, self.y1_outer_extreme + self.translation),
                          (self.minx_1, self.y2_outer_extreme + self.translation)])
        #plt.plot(*self.line_intersection_outer_extreme.xy)

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])
        #plt.plot(*self.line_base_line_intersection.xy)
        
        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.maxx_1, self.y1_outer_extreme + 2* self.translation),
                        (self.minx_1, self.y2_outer_extreme + 2* self.translation)])
        #plt.plot(*self.line_paralell_horizontal_movement.xy)
        
         # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y1_intersection_base_line),
                                (self.x2_horizontal_line, self.y1_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))
            
        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.maxx_1, self.y1_outer_extreme + i),
                            (self.minx_1, self.y2_outer_extreme + i)])
            #plt.plot(*line_up_down.xy)

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # #plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y1_up_down))
                self.path.append((x1_up_down, y2_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.maxx_1, self.y1_outer_extreme + i + self.translation),
                                (self.minx_1, self.y2_outer_extreme + i + self.translation)])
                #plt.plot(*line_down_up.xy)

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # #plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y2_up_down),
                                       (x1_down_up, y2_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y2_down_up))
                    self.path.append((x2_down_up, y1_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.maxx_1, self.y1_outer_extreme + i + 2 * self.translation),
                                    (self.minx_1, self.y2_outer_extreme + i + 2 * self.translation)])
                    #plt.plot(*line_up_down_second.xy)
                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y1_down_up),
                                       (x2_up_down_second, y1_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation


        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxy of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            y_value = poly_inner_exterior_points[1][i]
            if y_value == self.maxy_2:
                x_value = poly_inner_exterior_points[0][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                #print(path)
                pass

    
    def unit_vector(self, vector):
        # Takes a list with two floats as input
        # Returns the unit vector those floats represents
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        # Returns the angle between the vectors v1 and v2 in radians
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        # clip limits the unit vectors: -1 to 1
        # arccos is the arcosine of the vectors. Tha angle between them
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
            #Returns the angle between the vectors v1 and v2 in radians
        
        
        # v1_u = self.unit_vector(v1)
        # v2_u = self.unit_vector(v2)
        # angle_rad = np.arctan2(v2_u[1], v2_u[0]) - np.arctan2(v1_u[1], v1_u[0])
        # if angle_rad < 0:
        #     angle_rad += 2 * np.pi
        # return angle_rad
        
        
    def get_sides_from_polygon(self, poly: Polygon):
        # Input: Shapely Polygon
        # Output: All sides as a list of Shapely LineSting
        sides = []

        # Gets the points in XY coord that bounds the polygon
        x, y = poly.exterior.xy

        #T akes the bounding of the polygon and makes new lines.
        for i in range(len(x) - 1):
            sides.append(LineString([(x[i], y[i]), (x[i + 1], y[i + 1])]))

        return sides

    def get_closest_side(self, point: Point, sides: list):
        # Input: Shapely Point and a list of LineString
        # Output: LineString that is closest to the side

        closest_side = None

        # Initializes an infinitely far away point as starting point
        distance_closest_side = float('inf')

        # Checks all individual sides to get the closest one
        # Shapely LineString has a function to check the closest distance from a Point
        for side in sides:
            if side.distance(point) < distance_closest_side:
                closest_side = side
                distance_closest_side = side.distance(point)

        return closest_side

    def get_line_from_polygon_within_two_points(self, points: list, poly: Polygon):
        # Get the line connecting two points on a polygon.
        # Input: list of tuples of two cartesian points, Shapely Polygon
        # Output:

        # Creating the path to be returned
        path_ = list()
        # Creating the points
        # Verbose for readability
        x1 = points[0][0]
        y1 = points[0][1]
        x2 = points[1][0]
        y2 = points[1][1]

        point_1 = Point(x1, y1)
        point_2 = Point(x2, y2)

        # initializes empty LineString that will be filled
        side1 = LineString([])
        side2 = LineString([])

        # Get all the individual sides from the polygon
        sides = self.get_sides_from_polygon(poly)

        # Get the sides of the polygon the points are on
        # Here they can both be the same side. This is fine
        for side in sides:
            if side.distance(point_1)<1e-8:
                side1 = side
            if side.distance(point_2)<1e-8:
                side2 = side

        # Check if it is the same side
        # If it is the same side, return only that side
        if side1.equals(side2):
            path_.append([(x1, y1)])
            path_.append([(x2, y2)])
        else:
            if side1.intersects(side2):
                intersection = side1.intersection(side2)
                x = intersection.x
                y = intersection.y

                path_.append([(x1, y1)])
                path_.append([(x, y)])
                path_.append([(x2, y2)])
            else:
                path_.append([(x1, y1)])
                path_.append([(x2, y2)])

        return path_


#All within main are for testing purposes only.

if __name__ == "__main__":

    test_width = 0.1
    
    
    #vertices = [(2., 0.),(10., 0.5), (25., 3.),  (30., 7.), (10., 10), (3., 7.)]
    # vertices = [(423236.3895171815 , 6769335.20677714), (423233.38284477545, 6769328.5425624)
    #             , (423239.71184318454, 6769326.379024858), (423243.20974025177 , 6769332.30897843)]
    # vertex_1 = (423233.38284477545, 6769328.5425624)
    # vertex_2 = (423236.3895171815 , 6769335.20677714)
    #poly = Polygon(vertices)
    #x, y, = poly.exterior.xy
    ##plt.plot(x, y)
    
    #Start position: Easting: 385049 ----  Northing: 6784787
    
    # easting_origo = 385049
    # northing_origo = 6784787
    
    # vertices = [(easting_origo + 2, northing_origo), (easting_origo + 10, northing_origo + 0.5), (easting_origo + 25, northing_origo + 3)
    #             , (easting_origo + 30, northing_origo + 7), (easting_origo + 10, northing_origo + 10), (easting_origo + 3, northing_origo + 7)]
    
    # vertex_1 = (easting_origo + 2, northing_origo + 0)
    # vertex_2 = (easting_origo + 3, northing_origo + 7)
    
    #Creating polygons for testing
    #I need polygons that are not convex and that represents all the different cases.
    #The cases are:
    #Starting line along the y line on the minimal y value, obtuse and acute angled
    #Starting line along the y line on the maximal y value, obtuse and acute angled
    #Starting line along the x line on the minimal x value, obtuse and acute angled
    #Starting line along the x line on the maximal x value, obtuse and acute angled
    #Polygons can represent several cases at once and also be rectangles.
        
    #Rectangle with obtuse angles both on the minimal and maximal y value and the minimal and maximal x value
    vertices_obtuse = [(2, 1), (1, 3), (4, 4), (5 , 2)]
    #Vertices for the starting line on the y axis
    vertex_1_minimal_y_obtuse = (2, 1)
    vertex_2_minimal_y_obtuse = (1, 3)
    vertex_1_maximal_y_obtuse = (4, 4)
    vertex_2_maximal_y_obtuse = (5, 2)
    #Vertices for the starting line on the x axis
    vertex_1_minimal_x_obtuse = (2, 1)
    vertex_2_minimal_x_obtuse = (5, 2)
    vertex_1_maximal_x_obtuse = (1, 3)
    vertex_2_maximal_x_obtuse = (4, 4)
    
    #Rectangle with acute angles both on the minimal and maximal y value and the minimal and maximal x value
    vertices_acute = [(1, 2),(2, 4), (5, 3), (4, 1)]
    #Vertices for the starting line on the y axis
    vertex_1_minimal_y_acute = (1, 2)
    vertex_2_minimal_y_acute = (2, 4)
    vertex_1_maximal_y_acute = (5, 3)
    vertex_2_maximal_y_acute = (4, 1)
    #Vertices for the starting line on the x axis
    vertex_1_minimal_x_acute = (1, 2)
    vertex_2_minimal_x_acute = (4, 1)
    vertex_1_maximal_x_acute = (2, 4)
    vertex_2_maximal_x_acute = (5, 3)
    
    #Rectangle with right angles both on the minimal and maximal y value and the minimal and maximal x value
    vertices_right_angles = [(1, 1), (1, 3), (4, 3), (4, 1)]
    #Vertices for the startng line on the y axis
    vertex_1_minimal_y_right = (1, 1)
    vertex_2_minimal_y_right = (1, 3)
    vertex_1_maximal_y_right = (4, 3)
    vertex_2_maximal_y_right = (4, 1)
    #Vertices for the starting line on the x axis
    vertex_1_minimal_x_right = (1, 1)
    vertex_2_minimal_x_right = (4, 1)
    vertex_1_maximal_x_right = (1, 3)
    vertex_2_maximal_x_right = (4, 3)

    #Creating the polygons
    poly_obtuse = Polygon(vertices_obtuse)
    poly_acute = Polygon(vertices_acute)
    poly_right = Polygon(vertices_right_angles)
    
    # #Plotting the polygons
    # x, y = poly_obtuse.exterior.xy
    # #plt.plot(x, y)
    
    # x, y = poly_acute.exterior.xy
    # #plt.plot(x, y)
    
    #Creating the different path planners, comment, uncomment for different cases
    
    #path_planner = PathPlanner(test_width, vertex_1, vertex_2, vertices)
    #Obtuse rectengles
    # Left to right, obtuse - Done
    # path_planner = PathPlanner(test_width, vertex_1_minimal_y_obtuse, vertex_2_minimal_y_obtuse, vertices_obtuse)
    
    # Right to left, obtuse - Done
    # path_planner = PathPlanner(test_width, vertex_1_maximal_y_obtuse, vertex_2_maximal_y_obtuse, vertices_obtuse)
    
    # Bottom to top, acute - Done
    # path_planner = PathPlanner(test_width, vertex_1_minimal_x_obtuse, vertex_2_minimal_x_obtuse, vertices_obtuse)
    
    # Top to bottom, obtuse - Done
    # path_planner = PathPlanner(test_width, vertex_1_maximal_x_obtuse, vertex_2_maximal_x_obtuse, vertices_obtuse)
    
    
    #Acute rectangles
    # Left to right, acute - Done
    # path_planner = PathPlanner(test_width, vertex_1_minimal_y_acute, vertex_2_minimal_y_acute, vertices_acute)
    
    # Right to left, acute - This is actually obtuse for this configuration. - Done
    # path_planner = PathPlanner(test_width, vertex_1_maximal_y_acute, vertex_2_maximal_y_acute, vertices_acute)
    
    # Bottom to top, acute - Done
    # path_planner = PathPlanner(test_width, vertex_1_minimal_x_acute, vertex_2_minimal_x_acute, vertices_acute)
    
    # Top to bottom, acute - Done
    # path_planner = PathPlanner(test_width, vertex_1_maximal_x_acute, vertex_2_maximal_x_acute, vertices_acute)
    
    #Right angled rectangles
    # Left to right, right angled - Done
    # path_planner = PathPlanner(test_width, vertex_1_minimal_y_right, vertex_2_minimal_y_right, vertices_right_angles)
    
    # Right to left, right angled
    # path_planner = PathPlanner(test_width, vertex_1_maximal_y_right, vertex_2_maximal_y_right, vertices_right_angles)
    
    # Bottom to top, right angled - Done
    # path_planner = PathPlanner(test_width, vertex_1_minimal_x_right, vertex_2_minimal_x_right, vertices_right_angles)
    
    # Top to bottom, right angled - Done
    # path_planner = PathPlanner(test_width, vertex_1_maximal_x_right, vertex_2_maximal_x_right, vertices_right_angles)


    # Testing with utm points.
    
    

    path = path_planner.get_path()
    
    # This is for vizualising what is happening
    # Plotting original area
    #x, y = self.poly.exterior.xy
    #plt.plot(x, y)
    # Plotting inner area
    #x, y = self.poly_inner.exterior.xy
    ##plt.plot(x, y)
    # Plotting outer ekstremes
    #x, y = self.poly_outer_extremes.exterior.xy
    ##plt.plot(x, y)
    # plotting lines
    ##plt.plot(*self.line_base_line_intersection.xy)
    # #plt.plot(*self.line_intersection_outer_extreme.xy)
    # #plt.plot(*line_paralell_horizontal_movement.xy)
    # Plotting the path
    plt.plot(*zip(*path))
    
    #Plotting the given vertices
    # #plt.plot(*self.line_vertex_elongated.xy)
    
    

    plt.show()
    
    #plt.show()