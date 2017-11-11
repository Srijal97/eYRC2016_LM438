import numpy as np
import cv2
import math
import hashlib
import serial
import time
import copy

'''
* Team Id: LM#438
* Authors: Srijal Poojari, Aditya Nair
* Filename: task5-main.py
* Theme: eYRC-Launch a Module, 2016
* Description: This python program constantly processes frames from an overhead camera
*              and first finds the optimal solution for any given configuration and then
*              sends commands to the robot using XBee modules(like pickup, move, turn, etc).
*              As the robot moves, it makes movement errors which are detected using the
*              pink strips on the robot and commands are sent to make tiny corrections and
*              keep it from straying away.
'''

'''
NOTE:
Each box in the arena grid will be referred as a cell
Every Cell has a Cell Number or Co-ordinate(cord) in tuple format --> (x, y), from (1, 1) to (9, 6)
Every Cell also has a unique Node Number --> 00 to 69 as follows,
       1    2    3    4    5    6    7    8    9
    +----+----+----+----+----+----+----+----+----+
  A | 00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 |
    +----+----+----+----+----+----+----+----+----+
  B | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
    +----+----+----+----+----+----+----+----+----+
  C | 20 | 21 | 22 | 23 | 24 | 25 | 26 | 27 | 28 |
    +----+----+----+----+----+----+----+----+----+
  D | 30 | 31 | 32 | 33 | 34 | 35 | 36 | 37 | 38 |
    +----+----+----+----+----+----+----+----+----+
  E | 40 | 41 | 42 | 43 | 44 | 45 | 46 | 47 | 48 |
    +----+----+----+----+----+----+----+----+----+
  F | 50 | 51 | 52 | 53 | 54 | 55 | 56 | 57 | 58 |
    +----+----+----+----+----+----+----+----+----+

Bot facing directions are classified as North, South, East and West.
                          N
                          |
                    W ----|---- E
                          |
                          S

The position stickers on the bot will be referred as bot_marker/ indicator

Location_coordinates or coordinates which are the cell coordinates are not to be
confused with pixel coordinates which are points on image itself
'''

task_start = time.clock()

# Initialize serial
port = "COM4"
baud = 9600
ser = serial.Serial(port,baud,timeout=None)

# If reposition taken during run, the already deposited objects have to be ignored.
# The idea was to save the deposited locations in the text file as well and read
# it during the run. This has not been implemented in this program.
enable_reposition = False

door_reposition_vals = {
    0: 0,
    10: 0,
    20: 0,
    30: 0,
    40: 0,
    50: 0
}

# Get calibrated values from text file
values_file = open('values.txt', 'r+')
values = {}

for line in values_file:  # Read existing values
    content = line.split('.', 2)
    if len(content) == 2:
        values[content[0]] = int(content[1])

values_file.close()

# print values
if values['reposition'] == 1 and enable_reposition:
    door_reposition_vals[0] = values['0']
    door_reposition_vals[10] = values['10']
    door_reposition_vals[20] = values['20']
    door_reposition_vals[30] = values['30']
    door_reposition_vals[40] = values['40']
    door_reposition_vals[50] = values['50']


# -------------------------------- global variables ---------------------------------- #
door_area_list = []  # List storing parameters of door area markers
object_list = []  # List storing parameters of objects
obstacle_list = []  # List storing locations(node numbers) of obstacles

find_solution_quickly = True
carrying_capacity = 1

matches = {}  # Dictionary storing door:[objects list] matches as key:value pairs
path_array = []  # stores path for every match
depth_costs = {}
raw_solutions = []

area_permissible_error = 35

'''
For the best matching result, we do it for 30 frames and take the result occurring maximum amount of times.
We do this by storing every result and its MD5 hash value in a dictionary(matches_array) and incrementing count
each time it occurs again. This is further explained ahead.
'''
matches_array = {}  # Dictionary storing hash:[count, matches, door_list, object_list, obstacle_list] as key:value pair
object_cords_pt = None  # The pixel coordinates of current object's centroid, using for sending commands
object_loc_node = None  # The node number of current object, using for sending commands

boundary_pts = []  # The pixel points of inner boundary in a captured frame. We store this during inital matching
                   # and use it for bot position and orientation detection as it may cut the boundary at
                   # certain positions

repeat_count = 0  # counter to count 30 frames to repeat matching
camera_port = 1  # camera port for cv2.VideoCapture(camera_port)

bot_location = 24  # Location node number
bot_face = 'W'  # North:N, South:S, East:E or West:W
bot_cords = (5, 3)  # bot location coordinates, initally (5, 3) --> 5C

turn_cost_90 = 510  # Turn cost used for a* algorithm, obtained by measuring time it takes for bot to turn
turn_cost_180 = 700
cell_to_cell_cost = 200   # Cost for movement from cell to cell, used for a* algorithm

# All nodes initially are traversable, we'll modify it later as required.
travesable_nodes = [
     0,  1,  2,  3,  4,  5,  6,  7,  8,
    10, 11, 12, 13, 14, 15, 16, 17, 18,
    20, 21, 22, 23, 24, 25, 26, 27, 28,
    30, 31, 32, 33, 34, 35, 36, 37, 38,
    40, 41, 42, 43, 44, 45, 46, 47, 48,
    50, 51, 52, 53, 54, 55, 56, 57, 58
]

# Stores character values needed to be sent for each command. Same is mapped in bot code.
command_dict = {
        'test': 'w',

        'forward': 'w',
        'forward1': '1',
        'forward2': '2',
        'forward3': '3',
        'forward4': '4',
        'forward5': '5',
        'forward6': '6',
        'forward7': '7',
        'forward8': '8',

        'reverse': 's',
        'reverse1': '!',
        'reverse2': '"',
        'reverse3': '#',
        'reverse4': '$',
        'reverse5': '%',
        'reverse6': '&',
        'reverse7': '\'',
        'reverse8': ')',

        'left': 'a',
        'right': 'd',
        'pickup': 'z',
        'pickup-store': 'c',
        'pickup-store2': 'M',

        'drop': 'x',
        'drop-store': 'C',
        'drop-store2': 'N',

        'tiny-left': 'o',
        'tiny-right': 'p',
        'small-left': 'k',
        'small-right': 'l',

        'slow-left': '<',
        'slow-right': '>',

        'small-forward': 't',
        'small-reverse': 'g',
        'tiny-forward': 'y',
        'tiny-reverse': 'h',

        'slow-forward': ':',
        'slow-reverse': ';',

        'turn-180-left': 'A',
        'turn-180-right': 'D',

        'open-front-jaws': 'u',
        'close-front-jaws': 'i',
        'open-back-jaws': 'U',
        'close-back-jaws': 'I',

        'buzzer-on': ',',
        'buzzer-off': '.',

        'stop': ' '
    }

# Pixel values of cell rows and columns on arena
# As arena dimensions and our image dimensions after getting inner boundary image
# are fixed, these values are same
arena_centers_x = [32, 88, 163, 237, 312, 389, 463, 537, 602]  # columns
arena_centers_y = [21, 93, 171, 247, 322, 394]  # rows

# -------------------------------------------------------------------------------------- #

'''
* Function Name: breakpoint()
* Input: None
* Output: None
* Logic: Wait till 'q' is pressed. Used for debugging
* Example Call: breakpoint()
'''
def breakpoint():
    while not (cv2.waitKey(1) & 0xFF == ord('q')):
        continue
    return

'''
* Function Name: determine_turn()
* Input: (current_direction, current_node, next_node)
* Output: Required turn for given change in node, cost for turn, bot direction after turn
* Logic: Based on various possible combinations of turns and faces, output is read
*        from a dictionary
* Example Call: determine_turn('E', 23, 24)
'''
def determine_turn(current_dir, current_node, next_node):

    if current_node - next_node == 1:  # e.g. 24 - 23 = 1, movement towards West
        next_dir = 'W'
    elif current_node - next_node == -1:
        next_dir = 'E'
    elif current_node - next_node == 10:
        next_dir = 'N'
    else:
        next_dir = 'S'

    decider = current_dir + next_dir

    options = {  # [turn_required, turn_cost]
        'NN' : ['forward', 0],
        'SS' : ['forward', 0],
        'EE' : ['forward', 0],
        'WW' : ['forward', 0],

        'NS' : ['reverse', 0],
        'SN' : ['reverse', 0],
        'EW' : ['reverse', 0],
        'WE' : ['reverse', 0],

        'NW' : ['left-forward', turn_cost_90],
        'WS' : ['left-forward', turn_cost_90],
        'SE' : ['left-forward', turn_cost_90],
        'EN' : ['left-forward', turn_cost_90],

        'NE' : ['right-forward', turn_cost_90],
        'ES' : ['right-forward', turn_cost_90],
        'SW' : ['right-forward', turn_cost_90],
        'WN' : ['right-forward', turn_cost_90],
    }

    turn_required = options[decider][0]
    turn_cost = options[decider][1]

    if turn_required == 'reverse':  # For reverse, bot_direction doesnt change.
        next_dir = current_dir

    return turn_required, turn_cost, next_dir
# ------------------------------------------------------------------- #

'''
* Function Name: cord2node() --> Co-ordinate to Node Number
* Input: (cord_tuple) --> cord tuple to be converted to Node Number
* Output: The Node Number for the Cell as an integer
* Logic: Node Number is easier to work with in some cases,
*        whereas, Cords are easier in some. So they can
*        be easily interchanged.
*        Subtract 1 and combine the x and y parts
* Example Call: cord2node((3, 4))
'''
def cord2node(cord_tuple):
    x, y = cord_tuple
    return ((y - 1) * 10 + x - 1)
# ------------------------------------------------------------------- #

'''
* Function Name: node2cord() --> Node Number to Co-ordinate
* Input: (node_num) --> Node Number to be converted
* Output: The Cell Number(Co-ordinates) as a tuple
* Logic: Node Number is easier to work with in some cases,
*        whereas, Cords are easier in some. So they can
*        be easily interchanged.
*        Separate the digits and add 1 to each.
* Example Call: node2cord(43)
'''
def node2cord(node_num):
    x = node_num % 10 + 1
    y = node_num / 10 + 1
    return (x, y)
# ------------------------------------------------------------------- #

'''
* Function Name: node_distance()
* Input: (node1, node2) --> the 2 Node Numbers
* Output: The distance between the two nodes, which is the number of
*         steps/jumps from node1 to node2 as an integer
* Logic: Convert to cords. Distance is sum of steps taken in x-axis and
*        steps take on y-axis.
* Example Call: node_distance(72, 37)
'''
def node_distance(node1, node2):
    a1, b1 = node2cord(node1)
    a2, b2 = node2cord(node2)

    return abs(a1 - a2) + abs(b1 - b2)
# ------------------------------------------------------------------- #

'''
* Function Name: get_neighbours() --> Finds the adjacent nodes for a passed node
* Input: (node) --> The Node Number whose neighbours have to be found out.
* Output: Node numbers of adjacent cells as a list (Even if not traversable)
* Logic: Convert node number to cords, separate as x and y.
*        Adjacent cells are simply obtained by adding/subtracting 1
*        to the x and y parts, one at a time, if the result is a valid
*        cell number.
* Example Call: get_neighbours(82)
'''
def get_neighbours(node):

    x, y = node2cord(node)
    neighbours = []

    if not x - 1 < 1:  # If exists
        neighbours.append(cord2node((x - 1, y)))
    if not x + 1 > 10:
        neighbours.append(cord2node((x + 1, y)))
    if not y - 1 < 1:
        neighbours.append(cord2node((x, y - 1)))
    if not y + 1 > 10:
        neighbours.append(cord2node((x, y + 1)))

    return neighbours
# ------------------------------------------------------------------- #

'''
* Function Name: modified_a_star_search() --> Performs A* path finding for the passed nodes
* Input: (start_node, goal_node, traversable_nodes, current_dir) -->
*         Start of the path, Goal of the path, List of passable(traversable) nodes and current facing direction
* Output: -1 if no path is found, else: path cost, one node before goal(for pickup), path, final facing direction
* Logic: The famed A* Algorithm performed on the passed nodes as start and goal.
*        References --> https://goo.gl/8Qpxxo
*                   --> https://goo.gl/NkfJXQ
*                   --> and the some more on the internet...
*        Standard algorithm is modified in which g_cost is determined from the node_to_node and turn costs, and
*        directions are stored along with path for the next loop run
* Example Call: modified_a_star_search(32, 18, traversable_nodes, 'W')
'''
def modified_a_star_search(start_node, goal_node, traversable_nodes, current_dir):
    open_nodes = []
    closed_nodes = []

    traversable_nodes.append(start_node)
    traversable_nodes.append(goal_node)
    path_list = []  # Final output list of cords of the path

    node_parents = {}  # Stores node : parent(node from which it comes) used as a key : pair in dictionary
    f_cost = {}  # Stores node : f_cost used as a key : pair in dictionary
    g_cost = {}  # Stores node : g_cost used as a key : pair in dictionary
    node_directions = {}  # Direction for every node
    directions = []  # Directions for bot

    # f_cost = g_cost + h_cost #
    f_cost[start_node] = 0 + node_distance(start_node, goal_node) * cell_to_cell_cost
    g_cost[start_node] = 0

    # Stores direction of each node in the given path.
    node_directions[start_node] = current_dir

    open_nodes.append(start_node)

    while (1):
        f_cost_compare = 9999  # A large value to compare for every loop
        current_node = 0

        if len(open_nodes) == 0:  # Object not reachable
            traversable_nodes.remove(start_node)  # rectify the modified list
            traversable_nodes.remove(goal_node)
            return -1

        for node in open_nodes:  # Set node with least f_cost as current
            if f_cost[node] < f_cost_compare:
                current_node = node
                f_cost_compare = f_cost[node]

        open_nodes.remove(current_node)  # Transfer from open to closed
        closed_nodes.append(current_node)

        if current_node == goal_node:  # If reached goal
            break

        neighbours = get_neighbours(current_node)

        # -----For every neighbour get f_cost and add node to open_list--- #
        for i in neighbours:
            if i not in traversable_nodes or i in closed_nodes:
                continue

            # get the turn_required, costs and directions to get from current_node to neighbour
            turn_required, turn_cost, next_dir = determine_turn(node_directions[current_node][0], current_node, i)
            tentative_g_cost = g_cost[current_node] + turn_cost + node_distance(current_node, i) * cell_to_cell_cost

            if i not in open_nodes:
                open_nodes.append(i)
            elif tentative_g_cost >= g_cost[i]:  # This isn't a better path, skip it!
                continue

            # Store costs and update parent, directions
            g_cost[i] = tentative_g_cost
            f_cost[i] = g_cost[i] + node_distance(i, goal_node) * cell_to_cell_cost
            node_parents[i] = current_node
            node_directions[i] = [next_dir, turn_required]

    # ------------------------------- #
    path_node = goal_node
    path_list.append(goal_node)

    # Form path list by going parent to parent of each node
    while path_node != start_node:
        path_node = node_parents[path_node]
        path_list.append(path_node)  # Store as co-ordinate

    path_list.reverse()

    # create a directions_list for the path, so that it can be sent to the bot
    for i in range(0, len(path_list)-1):
        contents = node_directions[path_list[i+1]]
        directions.append(contents[1])

    pickup_node = path_list[-2]
    final_facing_dir = node_directions[goal_node][0]  # bot_facing_direction at the end of path

    if directions[-1] == 'reverse':  # If its reverse at the end of path, turn 180 degrees
        directions[-1] = 'turn-180'
        if final_facing_dir == 'N':
            final_facing_dir = 'S'
        elif final_facing_dir == 'W':
            final_facing_dir = 'E'
        elif final_facing_dir == 'S':
            final_facing_dir = 'N'
        else:
            final_facing_dir = 'W'
        cost = f_cost[goal_node]
        cost += turn_cost_180
        f_cost[goal_node] = cost

    # Remove the extra 'forward' at the end of path
    elif directions[-1] == 'left-forward':
        directions[-1] = 'left'
    elif directions[-1] == 'right-forward':
        directions[-1] = 'right'
    elif directions[-1] == 'forward':
        directions.pop()

    # del path_list[0]  # remove starting co-ordinate from path
    traversable_nodes.remove(start_node)  # rectify the modified list
    traversable_nodes.remove(goal_node)

    return f_cost[goal_node], pickup_node, directions, final_facing_dir
# ------------------------------------------------------------------- #

'''
* Function Name: get_node_center_cords() --> Finds node center cordinate in image.
*                                            Used for writing results on image only.
* Input: (node_number)
* Output: cords of point on image where results can be written
* Logic: split node number into cell offsets and multiply with cell widths to get image points
* Example Call: get_node_center_cords(24)
'''
def get_node_center_cords(node_num):
    cell_heigth = 420/6
    cell_width = 625/9

    node_x_part = node_num % 10
    node_y_part = int(node_num / 10)

    x = cell_width/4 + node_x_part * cell_width
    y = cell_heigth/2 + node_y_part * cell_heigth

    return x,y
# ------------------------------------------------------------------- #

'''
* Function Name: percent_variation()
* Input: (value 1, value 2)
* Output: Percentage variation in the 2 input values
* Logic: (change in values/first value) * 100
* Example Call: percent_variation(243, 256)
'''
def percent_variation(val_1, val_2):
    d_val = abs(val_1 - val_2)
    return (d_val/val_1) * 100
# ------------------------------------------------------------------- #

'''
* Function Name: assign_node()
* Input: (contour)
* Output: node number in which the contour is found
* Logic: Centroid of contour is found out and compared with cell width offsets
*        to find where it fits in. X and Y parts are then combined.
* Example Call: assign_node(contours_list[3])
'''
def assign_node(cnt):
    M = cv2.moments(cnt)
    Cx = int(M['m10'] / M['m00'])
    Cy = int(M['m01'] / M['m00'])
    p,q = 0,0
    x = 1
    y = 1
    col_width = 420/6
    row_width = 625/9
    for a in range(0,8):
        if Cx >= p and Cx <= (p+row_width):
            break
        else:
            x = x + 1
            p = p + row_width

    for b in range(0,5):
        if Cy >= q and Cy <= (q + col_width):
            break
        else:
            y = y + 1
            q = q + col_width

    return Cx, Cy, ((y - 1) * 10 + x - 1)
# ------------------------------------------------------------------- #

'''
* Function Name: assign_node_pt()
* Input: (point tuple)
* Output: node number in which the point is found
* Logic: Just like the assign_node() function above, this gets the points
*        directly so no need to get centroid using contour
* Example Call: assign_node_pt((320, 250))
'''
def assign_node_pt(center):
    print center
    Cx = center[0]
    Cy = center[1]
    p,q = 0,0
    x = 1
    y = 1
    col_width = 420/6
    row_width = 625/9
    for a in range(0,8):
        if Cx >= p and Cx <= (p+row_width):
            break
        else:
            x = x + 1
            p = p + row_width

    for b in range(0,5):
        if Cy >= q and Cy <= (q + col_width):
            break
        else:
            y = y + 1
            q = q + col_width

    return ((y - 1) * 10 + x - 1)
# -------------------------------------------------------------------#

'''
* Function Name: get_cw_points()
* Input: (list_of_4_points)
* Output: List of 4 points arranged in clockwise order w.r.t image
* Logic: Points' x and y parts are compared and arranged using if statements
* Example Call: get_cw_points([[1,0], [0,0], [0,1], [1,1]])
*            Output---> [[0,0], [1,0], [1,1], [0,1]] (x and y axis are as per OpenCV conventions)
'''
def get_cw_points(pts_list):
    # arrange points in clockwise direction
    pts_list.sort()
    min_2_col = pts_list[:2]
    max_2_col = pts_list[-2:]
    for pt in pts_list:  # reverse every point (x,y) = (y,x)
        pt = pt.reverse()
    pts_list.sort()
    for pt in pts_list:  # reverse again
        pt = pt.reverse()
    min_2_row = pts_list[:2]
    max_2_row = pts_list[-2:]

    cw_pts_list = [-1, -1, -1, -1]  # Clockwise points list
    for pt in pts_list:
        if pt in min_2_col and pt in min_2_row:
            cw_pts_list[0] = pt
        elif pt in max_2_col and pt in min_2_row:
            cw_pts_list[1] = pt
        elif pt in max_2_col and pt in max_2_row:
            cw_pts_list[2] = pt
        elif pt in min_2_col and pt in max_2_row:
            cw_pts_list[3] = pt

    return cw_pts_list
# -------------------------------------------------------------------#

'''
* Function Name: get_boundary_img()
* Input: (source_image)
* Output: return_value, image with black boundary and outer part removed, boundary points list
* Logic: Adaptive threshold is applied and contours are drawn. Innermost contour with perimeter > 1000
*        is chosen. ApproxPolyDP points are compared with bounding rectangle points to find 4 best
*        points which define the boundary. These points are given to cv2.warpPerspective() to fit inner
*        image into a fixed dimension of 625 x 420 pixels
* Example Call: get_boundary_img(src_img)
'''
def get_boundary_img(src_img):
    img_gray = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)

    img_gray = cv2.medianBlur(img_gray, values['blur_val'])
    if values['thresh_mode'] == 0:
        ret1, img_thresh = cv2.threshold(img_gray, values['lower_thresh'], values['upper_thresh'], cv2.THRESH_BINARY)
    else:
        img_thresh = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
                                           values['block_size'], values['C_val'])
    # cv2.imshow('thresh', img_thresh)
    # breakpoint()

    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ''' hierarchy = [[[NextContour, PrevContour, FirstChild, Parent]]] '''

    # cv2.drawContours(src_img, contours, -1, (255, 255, 0), 2)
    # cv2.imshow('source', src_img)

    # For all contours with perimeter > 1000, take the lowest. This would be the inner boundary of arena.
    perimeter_list = []
    for i in range(0, len(contours)):
        perimeter = cv2.arcLength(contours[i], True)
        if perimeter > 1000:
            perimeter_list.append((perimeter, i))

    if len(perimeter_list) == 0:
        return -1, src_img

    perimeter_list.sort()
    boundary_cnt = perimeter_list[0][1]

    # print perimeter_list
    # print boundary_cnt

    # Draw bounding rectangle for this inner boundary.
    rect = cv2.minAreaRect(contours[boundary_cnt])
    box = list(cv2.cv.BoxPoints(rect))
    box_pts = []

    # convert float cords to int and tuple to list
    for tuples in box:
        box_pts.append( list( (int(tuples[0]), int(tuples[1])) ) )

    box_pts = get_cw_points(box_pts)  # convert in clockwise order

    # print "bounding rectangle points: " + str(box_pts)
    # box = np.int32(box)
    # img = cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

    # Get 4 corner points for boundary contour and arrange in clockwise order
    epsilon = 0.01* cv2.arcLength(contours[boundary_cnt], True)
    approx = cv2.approxPolyDP(contours[boundary_cnt], epsilon, True)

    # cv2.drawContours(src_img, approx, -1, (255, 255, 0), 2)

    approx_pts_list = []
    final_pts_list = []
    for pt in approx:
        approx_pts_list.append(list(pt[0])) # store points as a list, not tuple.

    # print "approx polydp points: " + str(approx_pts_list)
    # pts_list = get_cw_points(pts_list)
    # print "approx polydp points: " + str(pts_list)


    '''
     Now we have 2 points' lists: approxPolyDP points and boundingRect points
     approxPolyDP may sometimes return more than 4 points but bounding rectangle
     always has 4 points.
     So we take 4 approxPolyDP points which are geometrically closest to the bounding
     rectangle points
    '''
    for pt1 in box_pts: # For every point in bounding rect, find closest approx poly dp point
        closest_pt = 0
        prev_distance = 99999999
        for pt2 in approx_pts_list:
            dx = pt1[0] - pt2[0]
            dy = pt1[1] - pt2[1]
            distance = int(math.sqrt(dx*dx + dy*dy))
            if distance < prev_distance:
                closest_pt = pt2
                prev_distance = distance
        final_pts_list.append(closest_pt)

    final_pts_list = get_cw_points(final_pts_list)
    #print "final matched points: " + str(final_pts_list)

    if len(final_pts_list) != 4:  # Error in image
        return -2, src_img

    rows = 420
    cols = 625
    '''
        Actual inner boundary dimensions are: 70'x47'
        70/47 = 1.4893
        625/420 = 1.4880 --Maintaining Aspect Ratio
    '''

    pts1 = np.float32(final_pts_list)
    pts2 = np.float32([[0, 0], [cols, 0], [cols, rows], [0, rows]])
    M = cv2.getPerspectiveTransform(pts1, pts2)

    dst = cv2.warpPerspective(src_img, M, (cols, rows))

    return 0, dst, final_pts_list
# -------------------------------------------------------------------#

'''
* Function Name: remove_background()
* Input: (source image)
* Output: same image with white portions(background) removed
* Logic: Threshold and mask image with white values and return result.
* Example Call: remove_background(src_img)
'''
def remove_background(src_img):
    #cv2.imshow('src_img', src_img)

    img_gray = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
    ret1, img_thresh = cv2.threshold(img_gray, values['lower_thresh_bgrnd'], 255, cv2.THRESH_BINARY_INV)

    # img_thresh = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 5)
    # img_thresh = cv2.medianBlur(img_thresh, values['blur_val_bgrnd'])

    #cv2.imshow('thresh', img_thresh)

    res = cv2.bitwise_and(src_img, src_img, mask=img_thresh)

    #cv2.imshow('res', res)

    hsv = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, values['sat_min'], 0])
    upper_white = np.array([255, values['sat_max'], 255])

    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    #cv2.imshow('mask_white', mask_white)
    final_mask = cv2.bitwise_and(res, res, mask=mask_white)

    #cv2.imshow('final_mask', final_mask)
    return final_mask
# -------------------------------------------------------------------#

'''
* Function Name: get_image()
* Input: (camera_object)
* Output: image taken from camera
* Logic: Uses camera.read() and returns image.
* Example Call: get_image(cap)
'''
def get_image(camera):
    retval, image = camera.read()
    return image
# -------------------------------------------------------------------#

'''
* Function Name: sound_buzzer()
* Input: None
* Output: None
* Logic: Sends command to bot for beeping buzzer for >5secs
* Example Call: sound_buzzer()
'''
def sound_buzzer():
    task_end = time.clock()
    print "Beeeeep! Task done!, time taken: " + str(task_end - task_start)
    command_xbee(command_dict['buzzer-on'])
    time.sleep(5.1)
    command_xbee(command_dict['buzzer-off'])
    values_file = open('values.txt', 'r+')
    values_file.write('reposition.' + '0' + '\n')
    values_file.write('*\n')
    values_file.close()
# -------------------------------------------------------------------#

'''
* Function Name: write_result()  --> Display matches result on image for easy debugging/view
* Input: (matches array, door_list, object_list, obstacle_list, image on which result is written)
* Output: image with result written (It's not returned, directly written on given image)
* Logic: Get cordinates of each entity on image and write its parameters with cv2.putText()
* Example Call: write_result(matches, door_list, object_list, obstacle_list, image)
'''
def write_result(matches, door_list, object_list, obstacle_list, image):
    # Also draw the centre lines
    for axis in arena_centers_y:
        image[axis, 0:624] = [255, 255, 0]
    for axis in arena_centers_x:
        image[0:419, axis] = [255, 255, 0]

    # Write result on image
    for entity in door_list:
        write_text = str(entity[0]) + str(entity[2:4])
        write_cord = get_node_center_cords(entity[0])
        if entity[0] in matches:
            write_text = write_text + "-" + str(matches[entity[0]])
        font = cv2.FONT_HERSHEY_PLAIN
        cv2.putText(image, write_text, write_cord, font, 0.7, (0, 0, 0), 1)
    for entity in object_list:
        write_text = str(entity[0]) + str(entity[2:4])
        write_cord = get_node_center_cords(entity[0])
        font = cv2.FONT_HERSHEY_PLAIN
        cv2.putText(image, write_text, write_cord, font, 0.7, (0, 0, 0), 1)
    for entity in obstacle_list:
        write_text = 'Obst'
        write_cord = get_node_center_cords(entity[0])
        font = cv2.FONT_HERSHEY_PLAIN
        cv2.putText(image, write_text, write_cord, font, 1, (0, 0, 0), 1)
# -------------------------------------------------------------------#

'''
* Function Name: mask_in_thresh()  --> Mask objects and obstacles once they are detected to avoid
*                                      inteference with bot detection
* Input: (type:object/obstacle, centroid points, threshold image)
* Output: threshold image with specified area masked
* Logic: object/obstacle dimensions in pixels are fixed so we set that roi to 0 (black)
* Example Call: mask_in_thresh(0, (83, 162), thresh)
'''
def mask_in_thresh(type, (cy, cx), thresh):
    if type == 0:  # Object is roughly 28x28 pixels
        pt1_x = cx - 14
        if pt1_x < 0:
            pt1_x = 0
        pt1_y = cy - 14
        if pt1_y < 0:
            pt1_y = 0
        pt2_x = cx + 14
        if pt2_x > 420:
            pt2_x = 420
        pt2_y = cy + 14
        if pt2_y > 625:
            pt2_y = 625
        thresh[pt1_x:pt2_x, pt1_y:pt2_y] = 0
    elif type == 1:  # Obstacle is roughly 48x48 pixels
        pt1_x = cx - 24
        if pt1_x < 0:
            pt1_x = 0
        pt1_y = cy - 24
        if pt1_y < 0:
            pt1_y = 0
        pt2_x = cx + 24
        if pt2_x > 420:
            pt2_x = 420
        pt2_y = cy + 24
        if pt2_y > 625:
            pt2_y = 625
        thresh[pt1_x:pt2_x, pt1_y:pt2_y] = 0

    return thresh
# -------------------------------------------------------------------#

'''
* Function Name: get_paths()
* Input: (dictionary of matches)
* Output: array(list) of paths for each match
* Logic: To find all possible combinations of complete task solution and
*        take the net least expensive path. Uses recursion to take different
*        branches for different objects.
* Example Call: get_paths(matches)
'''
def get_paths(matches_list, path_till_now, bot_location, bot_face, depth, rem_markers):
    # structure for matches_list: [(marker, [obj1, obj2])]
    # print matches_list, path_till_now
    # print "depth: " + str(depth)

    if len(matches_list) == 0:
        path_till_now_copy = copy.deepcopy(path_till_now)
        completion_ratio = 1.0 - rem_markers / float(total_matched_markers)
        path_till_now_copy.append(completion_ratio)

        path_array.append(path_till_now_copy)
    elif len(matches_list) == 1 or carrying_capacity == 1:
        for match_tuple in matches_list:
            for obj in match_tuple[1]:

                marker_loc = match_tuple[0]

                # print marker_loc, obj, travesable_nodes

                # modified_a_star_algorithm() returns (movement_cost, pickup_node, directions, final_facing_dir)
                # bot to object
                # ----------------------------------------------------- #
                path_till_now_copy = copy.deepcopy(path_till_now)

                ret_val = modified_a_star_search(bot_location, obj, travesable_nodes, bot_face)

                if ret_val == -1:  # No path!
                    completion_ratio = 1.0 - rem_markers / float(total_matched_markers)

                    path_till_now_copy.append(completion_ratio)
                    path_array.append(path_till_now_copy)
                    continue
                else:
                    cost1, pickup_node, directions1, face1 = ret_val
                    directions1.append('pickup')

                # object to marker calculations
                # ----------------------------------------------------- #
                travesable_nodes.append(obj)
                ret_val = modified_a_star_search(pickup_node, marker_loc, travesable_nodes, face1)

                if ret_val == -1:  # No path!
                    travesable_nodes.remove(obj)
                    completion_ratio = 1.0 - rem_markers / float(total_matched_markers)
                    path_till_now_copy.append(completion_ratio)
                    path_array.append(path_till_now_copy)
                    continue
                else:
                    cost2, bot_final_loc, directions2, face2 = ret_val
                    directions2.append('drop')
                # ----------------------------------------------------- #

                cost = cost1 + cost2

                contents = [cost, obj, pickup_node,
                            directions1, directions2, bot_final_loc, marker_loc, face2]

                if len(path_till_now_copy) == 0:
                    total_cost = cost
                    paths = [contents]
                    path_till_now_copy = [total_cost, paths]
                else:
                    cost = path_till_now_copy[0] + cost
                    path_till_now_copy[0] = cost
                    path_till_now_copy[1].append(contents)

                if find_solution_quickly:
                    if depth_costs.has_key(depth):
                        existing_cost = depth_costs[depth]
                        if existing_cost < cost:
                            travesable_nodes.remove(obj)
                            continue
                        else:
                            depth_costs[depth] = cost
                    else:
                        depth_costs[depth] = cost

                matches_copy = copy.deepcopy(matches_list)
                # print matches_list
                matches_copy.remove(match_tuple)
                # print "new matches copy: " + str(matches_copy)

                # If same object present in other matches, remove it for inner levels
                for i in range(0, len(matches_copy)):
                    entity = matches_copy[i]
                    marker = entity[0]
                    obj_matches = entity[1]

                    if obj in obj_matches:
                        obj_matches.remove(obj)
                        matches_copy[i] = (marker, obj_matches)

                depth += 1
                get_paths(matches_copy, path_till_now_copy, bot_final_loc, face2, depth, rem_markers - 1)
                depth -= 1
                travesable_nodes.remove(obj)

    elif len(matches_list) == 2 or carrying_capacity == 2:
        for match_tuple1 in matches_list:
            for match_tuple2 in matches_list:
                if match_tuple1 != match_tuple2:
                    for obj1 in match_tuple1[1]:
                        for obj2 in match_tuple2[1]:
                            if obj1 == obj2:
                                continue

                            path_till_now_copy = copy.deepcopy(path_till_now)

                            first_obj = obj1
                            first_dest = match_tuple1[0]
                            second_obj = obj2
                            second_dest = match_tuple2[0]

                            # path from bot to first_obj
                            # -------------------------------------------------------- #
                            ret_val = modified_a_star_search(bot_location, first_obj, travesable_nodes,
                                                             bot_face)

                            if ret_val == -1:  # No path!
                                matches_copy = copy.deepcopy(matches_list)
                                matches_copy.remove(match_tuple1)

                                depth += 1
                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face, depth, rem_markers)
                                depth -= 1
                                continue
                            else:
                                cost1, pickup_node1, directions1, face1 = ret_val
                                directions1.append('pickup-store')

                            # path from first_obj to second_obj
                            # -------------------------------------------------------- #
                            travesable_nodes.append(first_obj)
                            ret_val = modified_a_star_search(pickup_node1, second_obj, travesable_nodes,
                                                             face1)

                            if ret_val == -1:  # No path!
                                travesable_nodes.remove(first_obj)

                                matches_copy = copy.deepcopy(matches_list)
                                matches_copy.remove(match_tuple2)

                                depth += 1
                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face, depth, rem_markers)
                                depth -= 1

                                continue
                            else:
                                cost2, pickup_node2, directions2, face2 = ret_val
                                directions2.append('pickup')

                            # path from second_obj to second_dest
                            # -------------------------------------------------------- #
                            travesable_nodes.append(second_obj)
                            ret_val = modified_a_star_search(pickup_node2, second_dest, travesable_nodes,
                                                             face2)

                            if ret_val == -1:  # No path!
                                travesable_nodes.remove(second_obj)
                                travesable_nodes.remove(first_obj)

                                matches_copy = copy.deepcopy(matches_list)
                                matches_copy.remove(match_tuple2)

                                depth += 1
                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face, depth, rem_markers)
                                depth -= 1

                                continue
                            else:
                                cost3, bot_final_loc1, directions3, final_face1 = ret_val
                                directions3.append('drop')

                            # path from second_dest to first_dest
                            # -------------------------------------------------------- #
                            ret_val = modified_a_star_search(bot_final_loc1, first_dest, travesable_nodes, final_face1)

                            if ret_val == -1:  # No path!
                                travesable_nodes.remove(second_obj)
                                travesable_nodes.remove(first_obj)

                                matches_copy = copy.deepcopy(matches_list)
                                matches_copy.remove(match_tuple1)

                                depth += 1
                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face, depth, rem_markers)
                                depth -= 1

                                continue
                            else:
                                cost4, bot_final_loc2, directions4, final_face2 = ret_val
                                directions4.append('drop-store')

                            cost = cost1 + cost2 + cost3 + cost4

                            contents = [cost, obj1, obj2, pickup_node1, pickup_node2,
                                        directions1, directions2, directions3, directions4,
                                        bot_final_loc1, bot_final_loc2, first_dest, second_dest,
                                        final_face1, final_face2]

                            if len(path_till_now_copy) == 0:
                                total_cost = cost
                                paths = [contents]
                                path_till_now_copy = [total_cost, paths]
                            else:
                                cost = path_till_now_copy[0] + cost
                                path_till_now_copy[0] = cost
                                path_till_now_copy[1].append(contents)

                            if find_solution_quickly:
                                if depth_costs.has_key(depth):
                                    existing_cost = depth_costs[depth]
                                    if existing_cost < cost:
                                        travesable_nodes.remove(second_obj)
                                        travesable_nodes.remove(first_obj)
                                        continue
                                    else:
                                        depth_costs[depth] = cost
                                else:
                                    depth_costs[depth] = cost

                            # print obj1, match_tuple1, obj2, match_tuple2

                            matches_copy = copy.deepcopy(matches_list)
                            # print matches_list
                            matches_copy.remove(match_tuple1)
                            matches_copy.remove(match_tuple2)

                            # If same object present in other matches, remove it for inner levels
                            for i in range(0, len(matches_copy)):
                                entity = matches_copy[i]
                                marker = entity[0]
                                obj_matches = entity[1]

                                if obj1 in obj_matches:
                                    obj_matches.remove(obj1)
                                    matches_copy[i] = (marker, obj_matches)
                                elif obj2 in obj_matches:
                                    obj_matches.remove(obj2)
                                    matches_copy[i] = (marker, obj_matches)

                            depth += 1
                            get_paths(matches_copy, path_till_now_copy, bot_final_loc2, final_face2, depth,
                                      rem_markers -2)
                            depth -= 1

                            travesable_nodes.remove(second_obj)
                            travesable_nodes.remove(first_obj)
    elif carrying_capacity == 3:
        for match_tuple1 in matches_list:
            for match_tuple2 in matches_list:
                if match_tuple1 != match_tuple2:
                    for match_tuple3 in matches_list:
                        if match_tuple3 != match_tuple1 and match_tuple3 != match_tuple2:
                            for obj1 in match_tuple1[1]:
                                for obj2 in match_tuple2[1]:
                                    for obj3 in match_tuple3[1]:
                                        if obj1 != obj2 and obj2 != obj3 and obj1 != obj3:

                                            path_till_now_copy = copy.deepcopy(path_till_now)

                                            first_obj = obj1
                                            first_dest = match_tuple1[0]

                                            second_obj = obj2
                                            second_dest = match_tuple2[0]

                                            third_obj = obj3
                                            third_dest = match_tuple3[0]

                                            # print obj1, obj2, obj3, travesable_nodes

                                            # path from bot to first_obj
                                            # -------------------------------------------------------- #
                                            ret_val = modified_a_star_search(bot_location, first_obj, travesable_nodes,
                                                                             bot_face)

                                            if ret_val == -1:  # No path!
                                                matches_copy = copy.deepcopy(matches_list)

                                                matches_copy.remove(match_tuple1)

                                                depth += 1
                                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face,
                                                          depth, rem_markers)
                                                depth -= 1
                                                continue
                                            else:
                                                cost1, pickup_node1, directions1, face1 = ret_val
                                                directions1.append('pickup-store2')

                                            # path from first_obj to second_obj
                                            # -------------------------------------------------------- #
                                            travesable_nodes.append(first_obj)
                                            ret_val = modified_a_star_search(pickup_node1, second_obj,
                                                                             travesable_nodes,
                                                                             face1)

                                            if ret_val == -1:  # No path!

                                                travesable_nodes.remove(first_obj)

                                                matches_copy = copy.deepcopy(matches_list)
                                                matches_copy.remove(match_tuple2)

                                                depth += 1
                                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face,
                                                          depth, rem_markers)
                                                depth -= 1
                                                continue
                                            else:
                                                cost2, pickup_node2, directions2, face2 = ret_val
                                                directions2.append('pickup-store')

                                            # path from second_obj to third_obj
                                            # -------------------------------------------------------- #
                                            travesable_nodes.append(second_obj)
                                            ret_val = modified_a_star_search(pickup_node2, third_obj,
                                                                             travesable_nodes,
                                                                             face2)

                                            if ret_val == -1:  # No path!
                                                travesable_nodes.remove(second_obj)
                                                travesable_nodes.remove(first_obj)

                                                matches_copy = copy.deepcopy(matches_list)
                                                matches_copy.remove(match_tuple3)

                                                depth += 1
                                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face,
                                                          depth, rem_markers)
                                                depth -= 1
                                                continue
                                            else:
                                                cost3, pickup_node3, directions3, face3 = ret_val
                                                directions3.append('pickup')

                                            # path from third_obj to third_dest
                                            # -------------------------------------------------------- #
                                            travesable_nodes.append(third_obj)
                                            ret_val = modified_a_star_search(pickup_node3, third_dest,
                                                                             travesable_nodes,
                                                                             face3)

                                            if ret_val == -1:  # No path!
                                                travesable_nodes.remove(third_obj)
                                                travesable_nodes.remove(second_obj)
                                                travesable_nodes.remove(first_obj)

                                                matches_copy = copy.deepcopy(matches_list)
                                                matches_copy.remove(match_tuple3)

                                                depth += 1
                                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face,
                                                          depth, rem_markers)
                                                depth -= 1

                                                continue
                                            else:
                                                cost4, bot_final_loc1, directions4, final_face1 = ret_val
                                                directions4.append('drop')

                                            # path from third_dest to second_dest
                                            # -------------------------------------------------------- #
                                            ret_val = modified_a_star_search(bot_final_loc1, second_dest,
                                                                             travesable_nodes, final_face1)

                                            if ret_val == -1:  # No path!
                                                # print "no path"
                                                travesable_nodes.remove(third_obj)
                                                travesable_nodes.remove(second_obj)
                                                travesable_nodes.remove(first_obj)

                                                matches_copy = copy.deepcopy(matches_list)
                                                matches_copy.remove(match_tuple2)

                                                depth += 1
                                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face,
                                                          depth, rem_markers)
                                                depth -= 1
                                                continue
                                            else:
                                                cost5, bot_final_loc2, directions5, final_face2 = ret_val
                                                directions5.append('drop-store')

                                            # path from second_dest to first_dest
                                            # -------------------------------------------------------- #
                                            ret_val = modified_a_star_search(bot_final_loc2, first_dest,
                                                                             travesable_nodes, final_face2)

                                            if ret_val == -1:  # No path!
                                                travesable_nodes.remove(third_obj)
                                                travesable_nodes.remove(second_obj)
                                                travesable_nodes.remove(first_obj)

                                                matches_copy = copy.deepcopy(matches_list)
                                                matches_copy.remove(match_tuple1)

                                                depth += 1
                                                get_paths(matches_copy, path_till_now_copy, bot_location, bot_face,
                                                          depth, rem_markers)
                                                depth -= 1

                                                continue
                                            else:
                                                cost6, bot_final_loc3, directions6, final_face3 = ret_val
                                                directions6.append('drop-store2')

                                            cost = cost1 + cost2 + cost3 + cost4 + cost5 + cost6

                                            contents = [cost, obj1, obj2, obj3,
                                                        pickup_node1, pickup_node2, pickup_node3,
                                                        directions1, directions2, directions3,
                                                        directions4, directions5, directions6,
                                                        bot_final_loc1, bot_final_loc2, bot_final_loc3,
                                                        first_dest, second_dest, third_dest,
                                                        final_face1, final_face2, final_face3]

                                            if len(path_till_now_copy) == 0:
                                                total_cost = cost
                                                paths = [contents]
                                                path_till_now_copy = [total_cost, paths]
                                            else:
                                                cost = path_till_now_copy[0] + cost
                                                path_till_now_copy[0] = cost
                                                path_till_now_copy[1].append(contents)

                                            if find_solution_quickly:
                                                if depth_costs.has_key(depth):
                                                    existing_cost = depth_costs[depth]
                                                    if existing_cost < cost:
                                                        travesable_nodes.remove(third_obj)
                                                        travesable_nodes.remove(second_obj)
                                                        travesable_nodes.remove(first_obj)
                                                        continue
                                                    else:
                                                        depth_costs[depth] = cost
                                                else:
                                                    depth_costs[depth] = cost

                                            matches_copy = copy.deepcopy(matches_list)
                                            # print matches_list
                                            matches_copy.remove(match_tuple1)
                                            matches_copy.remove(match_tuple2)
                                            matches_copy.remove(match_tuple3)

                                            # If same object present in other matches, remove it for inner levels
                                            for i in range(0, len(matches_copy)):
                                                entity = matches_copy[i]
                                                marker = entity[0]
                                                obj_matches = entity[1]

                                                if obj1 in obj_matches:
                                                    obj_matches.remove(obj1)
                                                    matches_copy[i] = (marker, obj_matches)
                                                if obj2 in obj_matches:
                                                    obj_matches.remove(obj2)
                                                    matches_copy[i] = (marker, obj_matches)
                                                if obj3 in obj_matches:
                                                    obj_matches.remove(obj3)
                                                    matches_copy[i] = (marker, obj_matches)

                                            depth += 1
                                            get_paths(matches_copy, path_till_now_copy, bot_final_loc3, final_face3,
                                                      depth, rem_markers - 3)
                                            depth -= 1

                                            travesable_nodes.remove(third_obj)
                                            travesable_nodes.remove(second_obj)
                                            travesable_nodes.remove(first_obj)
# -------------------------------------------------------------------#

'''
* Function Name: ellipse_centre_angle()
* Input: (contour)
* Output: (centroid, angle)
* Logic: Fits an ellipse to the contour(which is always a rectangle).
*        cv2.fitEllipse() returns dimensions and inclination of the rectangle in which
*        the bounding rectangle is inscribed.
* Example Call: ellipse_centre_angle(contour)
'''
def ellipse_center_angle(cnt):
    (p1, q1), (x, y), angle = cv2.fitEllipse(cnt)

    # print "Ellipse center: " + str((x, y)) + ", p1 & q1: " + str((p1, q1)) + ", Ellipse Angle: " + str(angle)

    # x and y components of major and minor axes
    angle = math.radians(angle)
    MAx, MAy = int(0.5 * x * math.sin(angle)), int(0.5 * y * math.cos(angle))
    angle = math.degrees(angle)

    MAxtop = int(p1 + MAx)
    MAytop = int(q1 + MAy)
    MAxbot = int(p1 - MAx)
    MAybot = int(q1 - MAy)

    x_center = (MAxtop + MAxbot) / 2
    y_center = (MAytop + MAybot) / 2

    return (x_center, y_center), angle
# -------------------------------------------------------------------#

'''
* Function Name: get_pos_orientation() --> position and orientation of bot
* Input: (camera object)
* Output: (left_bot_indicator_center, bot_center, right_bot_indicator_center, angle)
* Logic: Captures image, removes boundary based on previous boundary points(because bot may cut boundary during the run),
*        masks the pink colour of the indicator, thresholds.
*        Then uses mask_in_thresh() to mask the areas of objects and obstacles.
*        Draws contours and takes 2 contours with max areas as bot indicators.
*        Finds centers and angles. Here's how angles were reported in openCV:
                     (180)N(0)
                          |
                (90)W ----|---- E(90)
                          |
                       (0)S(180)
*        This was problematic as for N and S, angles would overflow from 180 to 0 at random times.
*        We take the angle sine and sine_inverse to convert 0 to 180 ---> 0 to 90
                       (0)N(0)
                          |
                (90)W ----|---- E(90)
                          |
                       (0)S(0)
*        Then take average of the 2 indicator angles as bot angle, and midpoint of 2 centers as center
* Example Call: get_pos_orientation(cap)
'''
def get_pos_orientation(cap):
    img_raw = get_image(cap)
    # cv2.imshow('raw', img_raw)

    rows = 420
    cols = 625
    pts1 = np.float32(boundary_pts)
    pts2 = np.float32([[0, 0], [cols, 0], [cols, rows], [0, rows]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    img = cv2.warpPerspective(img_raw, M, (cols, rows))

    img = cv2.bilateralFilter(img, 15, 75, 75)  # Bilateral filter to blur while maintaining edges
    # cv2.imshow('img', img)

    no_bgrnd = remove_background(img)  # Function to remove white background

    # cv2.imshow('no_background', no_bgrnd)

    hsv_no_bgrnd = cv2.cvtColor(no_bgrnd, cv2.COLOR_BGR2HSV)

    lower_pink = np.array([values['p_hue_min'], values['p_sat_min'], values['p_val_min']])
    upper_pink = np.array([values['p_hue_max'], values['p_sat_max'], values['p_val_max']])

    mask_pink = cv2.inRange(hsv_no_bgrnd, lower_pink, upper_pink)

    # cv2.imshow('mask_pink', mask_pink)

    final_mask = cv2.bitwise_and(no_bgrnd, no_bgrnd, mask=mask_pink)

    no_bgrnd_gray = cv2.cvtColor(final_mask, cv2.COLOR_BGR2GRAY)

    ret_val, no_bgrnd_thresh = cv2.threshold(no_bgrnd_gray, 10, 255, cv2.THRESH_BINARY)

    # cv2.imshow('old_thresh', no_bgrnd_thresh)

    for obj in object_list:
        no_bgrnd_thresh = mask_in_thresh(0, obj[1], no_bgrnd_thresh)

    for obj in door_area_list:
        no_bgrnd_thresh = mask_in_thresh(1, obj[1], no_bgrnd_thresh)

    for obst in obstacle_list:
        no_bgrnd_thresh = mask_in_thresh(1, obst[1], no_bgrnd_thresh)

    # cv2.imshow('new_thresh', no_bgrnd_thresh)

    mask = cv2.bitwise_not(no_bgrnd_thresh)
    objects_masked = cv2.bitwise_and(no_bgrnd, no_bgrnd, mask=no_bgrnd_thresh)

    contours, hierarchy = cv2.findContours(no_bgrnd_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # cv2.drawContours(objects_masked, contours, -1, (255, 255, 0), 2)

    contour_list = []

    for cnt in contours:
        cnt_area = cv2.contourArea(cnt)
        # print cnt_area

        if cnt_area > 600:
            continue
        elif cnt_area < 10:
            continue
        contour_list.append([cnt_area, cnt])

    if len(contour_list) < 2:
        cv2.imshow('bot', objects_masked)
        print "Contour list number Error!!"
        breakpoint()

    # print contour_list

    '''
       sorting contour_list directly using sort() causes an error on some occasions, so we
       find the 2 contours with the maximum area manually here.
    '''
    max_area = 0
    left_marker_cnt = None
    for entity in contour_list:
        area = entity[0]
        if area > max_area:
            left_marker_cnt = entity[1]
            max_area = area
    contour_list.remove([max_area, left_marker_cnt])
    # print max_area

    max_area = 0
    right_marker_cnt = None
    for entity in contour_list:
        area = entity[0]
        if area > max_area:
            right_marker_cnt = entity[1]
            max_area = area
    contour_list.remove([max_area, right_marker_cnt])
    # print max_area

    center1, angle1 = ellipse_center_angle(left_marker_cnt)
    center2, angle2 = ellipse_center_angle(right_marker_cnt)

    angle1 = math.radians(angle1)
    angle2 = math.radians(angle2)

    # sine() and asin() takes angles in radians
    sin_angle1 = math.sin(angle1)
    sin_angle2 = math.sin(angle2)

    sin_avg = (sin_angle1 + sin_angle2) / 2
    final_angle = math.asin(sin_avg)

    final_angle = int(math.degrees(final_angle))

    center = int((center1[0] + center2[0]) / 2), int((center1[1] + center2[1]) / 2)

    # cv2.circle(objects_masked, center, 1, (255, 255, 255), 2)
    # cv2.circle(objects_masked, center1, 1, (255, 255, 0), 2)
    # cv2.circle(objects_masked, center2, 1, (0, 255, 255), 2)
    #
    # write_text = 'Angle: ' + str(final_angle)
    # write_cord = center
    # font = cv2.FONT_HERSHEY_PLAIN
    # cv2.putText(objects_masked, write_text, write_cord, font, 1, (255, 255, 0), 1)
    #
    # cv2.imshow('objects_masked', objects_masked)
    # breakpoint()

    return center1, center, center2, final_angle
# -------------------------------------------------------------------#

'''
* Function Name: angle_correct()
* Input: (left_correct_req, right_correct_req, final_angle: 0/90)
* Output: None
* Logic: Sends command to bot for the required angle corrections.
*        Based on the inputs, it sends tiny corrections so that bot reaches 0 or 90 degrees.
*        If it over-corrects, i.e. reaches the mirror angle on the other side without reaching 0 or 90,
*        sign of (angle before correction - angle after correction) changes. In such a case, direction of
*        correction is changed.
* Example Call: angle_correct(left_correct_req, right_correct_req, final_angle)
'''
def angle_correct(left_correct_req, right_correct_req, final_angle):
    print left_correct_req, right_correct_req
    if final_angle == 0:
        # repeat = 2
        # while repeat > 0:
        #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
        #     repeat -= 1
        while left_correct_req or right_correct_req:  # repeat until any corrections are required

            prev_angle = 90  # Just a starting value
            correct_time = 0.0
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            while bot_angle > 3 and left_correct_req:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                loop_time = time.clock()
                if loop_time - correct_time < 0.2:  # Basically a delay of 0.2s before the next correction
                    continue

                command_xbee(command_dict['tiny-left'])  # correct left
                res = await_response(2)  # wait till bot completes it
                correct_time = time.clock()
                # time.sleep(0.1)
                print "Left angle correct, bot angle: " + str(bot_angle) + ", prev angle: " + str(prev_angle)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

                if prev_angle - bot_angle < 0:  # If angle starts increasing, we're correcting in wrong direction!
                    right_correct_req = True
                    break
                prev_angle = bot_angle

            left_correct_req = False
            prev_angle = 90
            correct_time = 0.0
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            while bot_angle > 3 and right_correct_req:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                loop_time = time.clock()
                if loop_time - correct_time < 0.2:  # Basically a delay of 0.2s before the next correction
                    continue

                command_xbee(command_dict['tiny-right'])
                res = await_response(2)
                correct_time = time.clock()
                # time.sleep(0.1)
                print "Left angle correct, bot angle: " + str(bot_angle) + ", prev angle: " + str(prev_angle)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

                if prev_angle - bot_angle < 0:
                    left_correct_req = True
                    break
                prev_angle = bot_angle

            right_correct_req = False

    elif final_angle == 90:
        # repeat = 2
        # while repeat > 0:
        #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
        #     repeat -= 1
        while left_correct_req or right_correct_req:
            prev_angle = 0
            correct_time = 0.0
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            while bot_angle < 87 and left_correct_req:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                loop_time = time.clock()
                if loop_time - correct_time < 0.2:  # Basically a delay of 0.2s before the next correction
                    continue

                command_xbee(command_dict['tiny-left'])  # correct right
                res = await_response(2)
                correct_time = time.clock()
                # time.sleep(0.1)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

                if prev_angle - bot_angle > 0: # If angle starts decreasing, we're correcting in wrong direction!
                    right_correct_req = True
                    break
                prev_angle = bot_angle

            left_correct_req = False
            prev_angle = 0
            correct_time = 0.0
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            while bot_angle < 87 and right_correct_req:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                loop_time = time.clock()
                if loop_time - correct_time < 0.2:  # Basically a delay of 0.2s before the next correction
                    continue

                command_xbee(command_dict['tiny-right'])  # correct left
                #print "correct right, prev angle: " +  str(prev_angle) + str(bot_angle)
                res = await_response(2)
                correct_time = time.clock()
                # time.sleep(0.1)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

                if prev_angle - bot_angle > 0:
                    left_correct_req = True
                    break
                prev_angle = bot_angle

            right_correct_req = False
# -------------------------------------------------------------------#

'''
* Function Name: get_deviation_direction()
* Input: (left_center, right_center, bot_face)
* Output: (left_correction_required, right_correction_required)  --> Boolean values telling which correction is needed
* Logic: From the bot marker centers, it creates a line and finds its intercepts with arena boundary.
*        Sign of the difference between intercepts tells us the turn needed
* Example Call: get_deviation_direction(left_center, right_center, bot_face)
'''
def get_deviation_direction(left_center, right_center, bot_face):
    # print left_center, right_center, bot_face
    x1 = left_center[1]
    y1 = left_center[0]

    x2 = right_center[1]
    y2 = right_center[0]

    left_correct_req = False
    right_correct_req = False

    if x2 == x1:
        left_correct_req = False
        right_correct_req = False
    else:
        m = float(y2 - y1) / float(x2 - x1)
        if m == 0:
            left_correct_req = False
            right_correct_req = False
        elif bot_face == 'N' or bot_face == 'S':
            x_intercept1 = x1 - float(y1) / m
            x_intercept2 = x1 + float(625 - y1) / m
            delta_x = x_intercept1 - x_intercept2
            if delta_x > 3:
                left_correct_req = False
                right_correct_req = True
            elif delta_x < -3:
                left_correct_req = True
                right_correct_req = False
        elif bot_face == 'W' or bot_face == 'E':
            y_intercept1 = float(y1) - m * x1
            y_intercept2 = float(y1) + m * float(420 - x1)
            delta_y = y_intercept1 - y_intercept2
            if delta_y > 3:
                left_correct_req = True
                right_correct_req = False
            elif delta_y < -3:
                left_correct_req = False
                right_correct_req = True

    return left_correct_req, right_correct_req
# -------------------------------------------------------------------#

'''
* Function Name: set_timeout()
* Input: (timeout_value)
* Output: The timeout value to be checked while sending the characters to the Firebird V.
* Logic: The amount of expected time(in secs) which is taken by the Firebird V to move a
*        distance of 1-9 cells in the arena is added.
* Example Call: set_timeout(5)
'''
def set_timeout(param):
    timeout_values = {
        1: 4,
        2: 5,
        3: 6,
        4: 7,
        5: 8,
        6: 9,
        7: 10,
        8: 11,
        9: 12,
    }

    current_time = time.clock()

    return current_time + timeout_values[param]
# -------------------------------------------------------------------#

'''
* Function Name: command_xbee()
* Input: (command to be sent as a string)
* Output: None
* Logic: If serial port is open(which was initialized at the start), send the single character command
*        using serial.write()
* Example Call: command_xbee('forward')
'''
def command_xbee(command):
    if ser.isOpen():
        for x in command:
            print "Sent command: " + x
            ser.write(x)
    else:
        print "Send-Serial not open!"
# -------------------------------------------------------------------#

'''
* Function Name: send_command() --> Performs all the required corrections and positioning for
*                                   each command and sends it to command_xbee()
* Input: (command to be sent as a string, count[for forward and reverse], bot_facing_direction)
* Output: None
* Logic: Logic for each command is explained separately
* Example Call: send_command('forward', 5, 'E')  --> go forward 5 steps towards East
'''
def send_command(command, param, face):
    print command, param

    # if command != 'test':
    #     print command, param
    #     return

    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

    bot_cords = node2cord(assign_node_pt(bot_center))  # bot cell coordinates
    print "bot_cords: " + str(bot_cords) + " " + str(bot_center)

    initial_center_x = arena_centers_x[bot_cords[0] - 1]
    initial_center_y = arena_centers_y[bot_cords[1] - 1]

    center_x = initial_center_x
    center_y = initial_center_y

    ser.reset_input_buffer()

    if command == 'forward':
        '''
            General structure for any forward/reverse command:
            -final point is taken from arena_centers list
            -send command to bot based on number of steps forward/reverse
            -while bot is moving(bot sends a done character at the end)
                -get bot location
                -if it has moved off by 2 pixels, send correction...recheck every 400ms to prevent
                 a burst shot of corrections
                 It also checks the direction in which bot is moving, like if the bot is moving off to the right,
                 and we correct left...it moves towards left and no need to send further left corrections.
            -after it has reached, correct bot forward/back to final point which was found in step 1
            -lastly correct angle to 0 or 90 based on direction. This has been set to correct right by default
             as out bot's left wheel often slips and curves towards left. If a left correction is required,
             angle_correct() automatically detects this and changes direction.

            Here 'forward' and 'reverse' sections a highly unoptimized in the sense that it directly uses
            tons of If-Else statements. This happened because I was just copy-pasting and changing values for quick
            results. So warning! Keep blocks folded to save scrolling!
        '''
        if face == 'N':

            final_center_y = arena_centers_y[bot_cords[1] - 1 - param]
            final_center_x = initial_center_x
            command_xbee(command_dict[command + str(param)])
            prev_center = initial_center_x
            loop_time = time.clock()
            correct_time = loop_time
            timeout_at = set_timeout(param)

            while ser.in_waiting == 0 and loop_time < timeout_at:
                # If no character has arrived(done character), bot is moving
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]
                loop_time = time.clock()

                # print "bot center: " + str(bot_center)
                # (prev_center - center_x) gives us the direction in which bot moves.
                if initial_center_x - center_x > 2  and (prev_center - center_x > 0) \
                        and (loop_time - correct_time) > 0.4:

                    command_xbee(command_dict['small-right'])
                    res = await_response(2)

                    correct_time = time.clock()
                elif initial_center_x - center_x < -2 and (prev_center - center_x < 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-left'])
                    res = await_response(2)
                    correct_time = time.clock()
                prev_center = center_x

            ser.reset_input_buffer()  # clear serial buffer

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            while (final_center_y - center_y > 2) or (final_center_y - center_y < -2):
                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)

            return

        if face == 'W':
            final_center_y = initial_center_y
            final_center_x = arena_centers_x[bot_cords[0] - 1 - param]
            command_xbee(command_dict[command + str(param)])
            prev_center = initial_center_y
            loop_time = time.clock()
            correct_time = loop_time
            timeout_at = set_timeout(param)

            while ser.in_waiting == 0 and loop_time < timeout_at:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]
                loop_time = time.clock()

                if initial_center_y - center_y < -2 and (prev_center - center_y < 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-right'])
                    res = await_response(2)
                    correct_time = time.clock()

                elif (initial_center_y - center_y) > 2 and (prev_center - center_y > 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-left'])
                    res = await_response(2)
                    correct_time = time.clock()
                prev_center = center_y
            ser.reset_input_buffer()

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            while (final_center_x - center_x < -2) or (final_center_x - center_x > 2):
                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)



            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)

            return

        if face == 'S':
            final_center_y = arena_centers_y[bot_cords[1] - 1 + param]
            final_center_x = initial_center_x
            command_xbee(command_dict[command + str(param)])
            prev_center = initial_center_x
            loop_time = time.clock()
            correct_time = loop_time
            timeout_at = set_timeout(param)

            while ser.in_waiting == 0 and loop_time < timeout_at:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]
                loop_time = time.clock()

                if initial_center_x - center_x > 2  and (prev_center - center_x > 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-left'])
                    res = await_response(2)
                    correct_time = time.clock()
                elif initial_center_x - center_x < -2 and (prev_center - center_x < 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-right'])
                    res = await_response(2)
                    correct_time = time.clock()
                prev_center = center_x
            ser.reset_input_buffer()

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            while (final_center_y - center_y > 2) or (final_center_y - center_y < -2):
                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)



            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)

            return

        if face == 'E':
            final_center_y = initial_center_y
            final_center_x = arena_centers_x[bot_cords[0] - 1 + param]
            command_xbee(command_dict[command + str(param)])
            prev_center = initial_center_y
            loop_time = time.clock()
            correct_time = loop_time
            timeout_at = set_timeout(param)

            while ser.in_waiting == 0 and loop_time < timeout_at:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]
                print center_y
                loop_time = time.clock()

                if initial_center_y - center_y < -2 and (prev_center - center_y < 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-left'])
                    res = await_response(2)
                    correct_time = time.clock()

                elif (initial_center_y - center_y) > 2 and (prev_center - center_y > 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-right'])
                    res = await_response(2)
                    correct_time = time.clock()
                prev_center = center_y
            ser.reset_input_buffer()

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            while (final_center_x - center_x < -2) or (final_center_x - center_x > 2):
                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)

            return

    if command == 'reverse':
        '''
            General structure for any forward/reverse command:
            -final point is taken from arena_centers list
            -send command to bot based on number of steps forward/reverse
            -while bot is moving(bot sends a done character at the end)
                -get bot location
                -if it has moved off by 2 pixels, send correction...recheck every 400ms to prevent
                 a burst shot of corrections
                 It also checks the direction in which bot is moving, like if the bot is moving off to the right,
                 and we correct left...it moves towards left and no need to send further left corrections.
            -after it has reached, correct bot forward/back to final point which was found in step 1
            -lastly correct angle to 0 or 90 based on direction. This has been set to correct right by default
             as out bot's left wheel often slips and curves towards left. If a left correction is required,
             angle_correct() automatically detects this and changes direction.

            Here 'forward' and 'reverse' sections a highly unoptimized in the sense that it directly uses
            tons of If-Else statements. This happened because I was just copy-pasting and changing values for quick
            results.
        '''
        if face == 'N':
            final_center_y = arena_centers_y[bot_cords[1] - 1 + param]
            final_center_x = initial_center_x
            command_xbee(command_dict[command + str(param)])
            prev_center = initial_center_x
            loop_time = time.clock()
            correct_time = loop_time
            timeout_at = set_timeout(param)

            while ser.in_waiting == 0 and loop_time < timeout_at:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]
                loop_time = time.clock()

                if initial_center_x - center_x < -2  and (prev_center - center_x < 0)\
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-right'])
                    res = await_response(2)
                    correct_time = time.clock()
                elif initial_center_x - center_x > 2 and (prev_center - center_x > 0)\
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-left'])
                    res = await_response(2)
                    correct_time = time.clock()
                prev_center = center_x
            ser.reset_input_buffer()

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     # print "bot center: " + str(bot_center)
            #     repeat -= 1

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            while (final_center_y - center_y > 2) or (final_center_y - center_y < -2):
                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)


            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)

            return

        if face == 'W':
            final_center_y = initial_center_y
            final_center_x = arena_centers_x[bot_cords[0] - 1 + param]
            command_xbee(command_dict[command + str(param)])
            prev_center = initial_center_y
            loop_time = time.clock()
            correct_time = loop_time
            timeout_at = set_timeout(param)

            while ser.in_waiting == 0 and loop_time < timeout_at:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]
                loop_time = time.clock()

                if initial_center_y - center_y > 2 and (prev_center - center_y > 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-right'])
                    res = await_response(2)
                    correct_time = time.clock()

                elif (initial_center_y - center_y) < -2 and (prev_center - center_y < 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-left'])
                    res = await_response(2)
                    print "correct-left" + str(res)
                    correct_time = time.clock()

                prev_center = center_y
            ser.reset_input_buffer()

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            while (final_center_x - center_x < -2) or (final_center_x - center_x > 2):
                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)


            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)

            return

        if face == 'S':
            final_center_y = arena_centers_y[bot_cords[1] - 1 - param]
            final_center_x = initial_center_x
            command_xbee(command_dict[command + str(param)])
            prev_center = initial_center_x
            loop_time = time.clock()
            correct_time = loop_time
            timeout_at = set_timeout(param)

            while ser.in_waiting == 0 and loop_time < timeout_at:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]
                loop_time = time.clock()

                if initial_center_x - center_x < -2  and (prev_center - center_x < 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-left'])
                    res = await_response(2)
                    correct_time = time.clock()

                elif initial_center_x - center_x > 2 and (prev_center - center_x > 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-right'])
                    res = await_response(2)
                    correct_time = time.clock()

                prev_center = center_x
            ser.reset_input_buffer()

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            while (final_center_y - center_y > 2) or (final_center_y - center_y < -2):
                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)


            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)

            return

        if face == 'E':
            final_center_y = initial_center_y
            final_center_x = arena_centers_x[bot_cords[0] - 1 - param]
            command_xbee(command_dict[command + str(param)])
            prev_center = initial_center_y
            loop_time = time.clock()
            correct_time = loop_time
            timeout_at = set_timeout(param)

            while ser.in_waiting == 0 and loop_time < timeout_at:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]
                loop_time = time.clock()

                if initial_center_y - center_y > 2 and (prev_center - center_y > 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-left'])
                    res = await_response(2)
                    correct_time = time.clock()

                elif (initial_center_y - center_y) < -2 and (prev_center - center_y < 0) \
                        and (loop_time-correct_time) > 0.4:

                    command_xbee(command_dict['small-right'])
                    res = await_response(2)
                    correct_time = time.clock()

                prev_center = center_y
            ser.reset_input_buffer()

            # repeat = 2
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            while (final_center_x - center_x < -2) or (final_center_x - center_x > 2):
                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)
            return

    if command == 'left':
        '''
         for a 90 degree turn while maintaining bot location, bot moves forward by 4 pixels from the reference
         center taken from arena_centers list, turns 90 degrees, corrects angle using angle_correct() and moves
         back to the reference cell point
        '''

        if face == 'W' or face == 'E':
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            node_center_x = arena_centers_x[bot_cords[0] - 1]

            if face == 'W':
                while node_center_x - center_x < 4:  # Move ahead from reference by 4 pixels
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]
            if face == 'E':
                while node_center_x - center_x > -4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

            command_xbee(command_dict['left'])
            res = await_response(4)

            # repeat = 3
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                print "Bot Angle: " + str(bot_angle)
                if face == 'W':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'S')
                    angle_correct(left_correct_req, right_correct_req, 0)
                if face == 'E':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'N')
                    angle_correct(left_correct_req, right_correct_req, 0)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            final_center_y = arena_centers_y[bot_cords[1] - 1]

            if face == 'W':  # Move back to cell center
                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]
            if face == 'E':
                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

            return

        if face == 'N' or face == 'S':
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            node_center_y = arena_centers_y[bot_cords[1] - 1]

            if face == 'N':
                while node_center_y - center_y < 4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]
            if face == 'S':
                while node_center_y - center_y > -4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

            command_xbee(command_dict['left'])
            res = await_response(4)

            # repeat = 3
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                print "Bot Angle: " + str(bot_angle)
                if face == 'N':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'W')
                    angle_correct(left_correct_req, right_correct_req, 90)
                if face == 'S':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'E')
                    angle_correct(left_correct_req, right_correct_req, 90)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            final_center_x = arena_centers_x[bot_cords[0] - 1]

            if face == 'N':
                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]
            if face == 'S':
                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]
            return

    if command == 'right':
        '''
         for a 90 degree turn while maintaining bot location, bot moves forward by 4 pixels from the reference
         center taken from arena_centers list, turns 90 degrees, corrects angle using angle_correct() and moves
         back to the reference cell point
        '''
        if face == 'W' or face == 'E':
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            node_center_x = arena_centers_x[bot_cords[0] - 1]

            if face == 'W':
                while node_center_x - center_x < 4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]
            if face == 'E':
                while node_center_x - center_x > -4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

            command_xbee(command_dict['right'])
            res = await_response(4)

            # repeat = 3
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                print "Bot Angle: " + str(bot_angle)
                if face == 'W':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'N')
                    angle_correct(left_correct_req, right_correct_req, 0)
                if face == 'E':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'S')
                    angle_correct(left_correct_req, right_correct_req, 0)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            final_center_y = arena_centers_y[bot_cords[1] - 1]

            if face == 'W':
                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]
            if face == 'E':
                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

            return

        if face == 'N' or face == 'S':
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            node_center_y = arena_centers_y[bot_cords[1] - 1]

            if face == 'N':
                while node_center_y - center_y < 4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]
            if face == 'S':
                while node_center_y - center_y > -4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]


            command_xbee(command_dict['right'])
            res = await_response(4)

            # repeat = 3
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1


            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                print "Bot Angle: " + str(bot_angle)
                if face == 'N':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'E')
                    angle_correct(left_correct_req, right_correct_req, 90)
                if face == 'S':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'W')
                    angle_correct(left_correct_req, right_correct_req, 90)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            final_center_x = arena_centers_x[bot_cords[0] - 1]

            if face == 'N':
                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]
            if face == 'S':
                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

            return

    if command == 'turn-180-left' or command == 'turn-180-right':
        '''
          Similar to a 90 turn except that it turns 180.
        '''
        if face == 'N' or face == 'S':
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            node_center_y = arena_centers_y[bot_cords[1] - 1]

            if face == 'N':
                while node_center_y - center_y < 4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]
            if face == 'S':
                while node_center_y - center_y > -4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

            command_xbee(command_dict[command])
            res = await_response(6)

            # repeat = 3
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                print "Bot Angle: " + str(bot_angle)
                if face == 'N':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'S')
                    angle_correct(left_correct_req, right_correct_req, 0)
                if face == 'S':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'N')
                    angle_correct(left_correct_req, right_correct_req, 0)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_y = bot_center[1]

            final_center_y = arena_centers_y[bot_cords[1] - 1]

            if face == 'N':
                while node_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

            if face == 'S':
                while node_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

        if face == 'E' or face == 'W':
            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            node_center_x = arena_centers_x[bot_cords[0] - 1]

            if face == 'W':
                while node_center_x - center_x < 4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]
            if face == 'E':
                while node_center_x - center_x > -4:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

            command_xbee(command_dict[command])
            res = await_response(6)

            # repeat = 3
            # while repeat > 0:
            #     left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            #     repeat -= 1

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                print "Bot Angle: " + str(bot_angle)
                if face == 'E':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'W')
                    angle_correct(left_correct_req, right_correct_req, 90)
                if face == 'W':
                    left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, 'E')
                    angle_correct(left_correct_req, right_correct_req, 90)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]

            final_center_x = arena_centers_x[bot_cords[0] - 1]

            if face == 'W':
                while node_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

            if face == 'E':
                while node_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

    if command == 'pickup' or command == 'pickup-store' or command == 'pickup-store2':
        '''
         Before pickup, the bot adjusts its position so that it is 74 to 80 pixels away from object center
         which was optimum based on our arm design.
         It then corrects angle, picks up the object and moves mack to cell reference center
        '''
        object_cords = node2cord(object_loc_node)  # object cell coordinates, required for
        print "object cords: " + str(object_cords)
        left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
        center_x = bot_center[0]
        center_y = bot_center[1]

        final_center_x = arena_centers_x[bot_cords[0] - 1]
        final_center_y = arena_centers_y[bot_cords[1] - 1]

        # print final_center_x

        if face == 'W':
            while (final_center_x - center_x < -2) or (final_center_x - center_x > 2):
                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

        if face == 'E':
            while (final_center_x - center_x < -2) or (final_center_x - center_x > 2):
                while final_center_x - center_x > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                while final_center_x - center_x < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_x = bot_center[0]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

        if face == 'S':
            while (final_center_y - center_y > 2) or (final_center_y - center_y < -2):
                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

        if face == 'N':
            while (final_center_y - center_y > 2) or (final_center_y - center_y < -2):
                while final_center_y - center_y > 2:
                    command_xbee(command_dict['tiny-reverse'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                while final_center_y - center_y < -2:
                    command_xbee(command_dict['tiny-forward'])
                    res = await_response(2)
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    center_y = bot_center[1]

                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

        time.sleep(0.05)
        command_xbee(command_dict[command])
        res = await_response(10)

        return

    if command == 'drop' or command == 'drop-store' or command == 'drop-store2':
        '''
        Similar to pickup, but here optimum drop distance is 50 to 65 pixels.
        '''
        object_cords = node2cord(object_loc_node)  # object cell coordinates, required for
        print "object cords: " + str(object_cords)
        left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
        center_x = bot_center[0]
        center_y = bot_center[1]

        final_center_x = arena_centers_x[bot_cords[0] - 1] + 5
        final_center_y = arena_centers_y[bot_cords[1] - 1]

        if face == 'W':
            while final_center_x - 1 - center_x < -2:
                command_xbee(command_dict['tiny-forward'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]

            while final_center_x - 1 - center_x > 2:
                command_xbee(command_dict['tiny-reverse'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            # time.sleep(0.05)
            command_xbee(command_dict[command])
            res = await_response(10)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]
            center_y = bot_center[1]

            while final_center_x - center_x < -2:
                command_xbee(command_dict['tiny-forward'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]

            while final_center_x - center_x > 2:
                command_xbee(command_dict['tiny-reverse'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)


        if face == 'E':
            while final_center_x + 1 - center_x > 2:
                command_xbee(command_dict['tiny-forward'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]

            while final_center_x + 1 - center_x < -2:
                command_xbee(command_dict['tiny-reverse'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            # time.sleep(0.05)
            command_xbee(command_dict[command])
            res = await_response(10)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]
            center_y = bot_center[1]

            while final_center_x - center_x > 2:
                command_xbee(command_dict['tiny-forward'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]

            while final_center_x - center_x < -2:
                command_xbee(command_dict['tiny-reverse'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_x = bot_center[0]

            if bot_angle < 88:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 90)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)


        if face == 'S':
            while final_center_y - 1 - center_y < -2:
                command_xbee(command_dict['tiny-reverse'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]

            while final_center_y - 1 - center_y > 2:
                command_xbee(command_dict['tiny-forward'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            # time.sleep(0.05)
            command_xbee(command_dict[command])
            res = await_response(10)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]
            center_y = bot_center[1]
            while final_center_y - center_y < -2:
                command_xbee(command_dict['tiny-reverse'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]

            while final_center_y - center_y > 2:
                command_xbee(command_dict['tiny-forward'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)


        if face == 'N':
            while final_center_y + 1 - center_y > 2:
                command_xbee(command_dict['tiny-reverse'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]

            while final_center_y + 1 - center_y < -2:
                command_xbee(command_dict['tiny-forward'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

            # time.sleep(0.05)
            command_xbee(command_dict[command])
            res = await_response(10)

            left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
            center_x = bot_center[0]
            center_y = bot_center[1]
            while final_center_y - center_y > 2:
                command_xbee(command_dict['tiny-reverse'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]

            while final_center_y - center_y < -2:
                command_xbee(command_dict['tiny-forward'])
                res = await_response(2)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                center_y = bot_center[1]

            if bot_angle > 2:
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                left_correct_req, right_correct_req = get_deviation_direction(left_center, right_center, bot_face)

                angle_correct(left_correct_req, right_correct_req, 0)
                left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)

        if enable_reposition:
            values_file = open('values.txt', 'r+')
            values_file.write('reposition.' + '0' + '\n')
            values_file.write('0.0\n')
            values_file.write('10.0\n')
            values_file.write('20.0\n')
            values_file.write('30.0\n')
            values_file.write('40.0\n')
            values_file.write('50.0\n')
            values_file.write('*\n')
            values_file.close()

        return
# -------------------------------------------------------------------#

'''
* Function Name: await_response()
* Input: None
* Output: (response received from bot as char)
* Logic: If serial port is open(which was initialized at the start), wait till a character
*        arrives
* Example Call: await_response()
'''
def await_response(timeout_at):
    received = None
    timeout_time = time.clock() + timeout_at
    if ser.isOpen():
        loop_time = time.clock()
        while ser.in_waiting == 0 and loop_time < timeout_time:
            loop_time = time.clock()
        if loop_time > timeout_time:
            print "timeout!"
            return 't'
        else:
            received = ser.read(size=1)
            print "Received: " + received

    else:
        print "Read-Serial not open!"

    return received
# -------------------------------------------------------------------#

'''
* Function Name: turn_decider_180()
* Input: (facing_direction, location_as_node_number)
* Output: required turn to be taken
* Logic: A 180 degree turn could be taken from either the left or right sides.
*        So as to avoid collision with obstacles, it would be better if the
*        robot turns in the direction which has an empty cell, rather than an
*        obstacle. Various conditions are taken into picture and the best turn
*        is decided.
* Example Call: turn_decider_180('N', 35)
'''
def turn_decider_180(face, location):
    turn_180_decider = {
        'N00': 'turn-180-left',
        'N01': 'turn-180-right',
        'N10': 'turn-180-left',
        'N11': 'turn-180-left',

        'S00': 'turn-180-left',
        'S01': 'turn-180-right',
        'S10': 'turn-180-left',
        'S11': 'turn-180-left',

        'W00': 'turn-180-left',
        'W01': 'turn-180-right',
        'W10': 'turn-180-left',
        'W11': 'turn-180-left',

        'E00': 'turn-180-left',
        'E01': 'turn-180-right',
        'E10': 'turn-180-left',
        'E11': 'turn-180-left'
    }

    if face == 'N':
        left_val = (location - 1) in travesable_nodes
        right_val = (location + 1) in travesable_nodes

        if not (left_val or right_val):
            obstacle_nodes = []
            for entity in obstacle_list:
                obstacle_nodes.append(entity[0])
            left_val = (location - 1) not in obstacle_nodes
            right_val = (location + 1) not in obstacle_nodes
        left_val = str(int(left_val))
        right_val = str(int(right_val))
        return turn_180_decider[face + left_val + right_val]

    if face == 'S':
        left_val = (location + 1) in travesable_nodes
        right_val = (location - 1) in travesable_nodes

        if not (left_val or right_val):
            obstacle_nodes = []
            for entity in obstacle_list:
                obstacle_nodes.append(entity[0])
            left_val = (location + 1) not in obstacle_nodes
            right_val = (location - 1) not in obstacle_nodes
        left_val = str(int(left_val))
        right_val = str(int(right_val))
        return turn_180_decider[face + left_val + right_val]

    if face == 'W':
        left_val = (location + 10) in travesable_nodes
        right_val = (location - 10) in travesable_nodes

        if not (left_val or right_val):
            obstacle_nodes = []
            for entity in obstacle_list:
                obstacle_nodes.append(entity[0])
            left_val = (location + 10) not in obstacle_nodes
            right_val = (location - 10) not in obstacle_nodes
        left_val = str(int(left_val))
        right_val = str(int(right_val))
        # print left_val, right_val
        return turn_180_decider[face + left_val + right_val]

    if face == 'E':
        left_val = (location - 10) in travesable_nodes
        right_val = (location + 10) in travesable_nodes

        if not (left_val or right_val):
            obstacle_nodes = []
            for entity in obstacle_list:
                obstacle_nodes.append(entity[0])
            left_val = (location - 10) not in obstacle_nodes
            right_val = (location + 10) not in obstacle_nodes
        left_val = str(int(left_val))
        right_val = str(int(right_val))
        return turn_180_decider[face + left_val + right_val]
# -------------------------------------------------------------------#

'''
* Function Name: process_image()
* Input: (number_of_frames, camera_object)
* Output: Number of times the result has occured out of the total frames, list of empty nodes.
* Logic: Applies various image processing techniques to identify all the objects and obstacles.
*        This is done for multiple frames and the result occuring the maximum number of times is
*        taken.
* Example Call: process_image(30, camera_object)
'''
def process_image(rep_count, cap):
    '''
    Now, we perform shape, color and location detection of objects and color markers
    for the next 30 frames and take the best result.
    '''
    trav_node = travesable_nodes[:]
    repeat_count = 0
    while repeat_count < rep_count:

        global obstacle_list
        global object_list
        global door_area_list
        global matches
        global matches_array
        global boundary_pts

        img_raw = get_image(cap)
        ret_val, img, pts = get_boundary_img(img_raw)

        if ret_val == -1:
            print "Failed to find perimeter"
            continue
        elif ret_val == -2:
            print "Final border points list error!"
            continue

        boundary_pts = pts  # Store the boundary points to be used during bot position/orientation detection

        img = cv2.bilateralFilter(img, 15, 75, 75)  # Bilateral filter to blur while maintaining edges
        no_bgrnd = remove_background(img)  # Function to remove white background
        no_bgrnd_gray = cv2.cvtColor(no_bgrnd, cv2.COLOR_BGR2GRAY)

        ret1, no_bgrnd_thresh = cv2.threshold(no_bgrnd_gray, 10, 255, cv2.THRESH_BINARY)
        # no_bgrnd_thresh = cv2.adaptiveThreshold(no_bgrnd_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 5)

        cv2.imshow('thresh', no_bgrnd_thresh)

        # Draw contours in 'no background' image
        contours, hierarchy = cv2.findContours(no_bgrnd_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(no_bgrnd, contours, -1, (255, 255, 0), 2)

        # --------------------------- Shape Detection Begins -------------------------------- #
        for cnt in contours:
            # Find area for every contour and discard contours with area < 10 (Noise)
            cnt_area = cv2.contourArea(cnt)
            if cnt_area < 10:
                continue

            cx, cy, node_num = assign_node(cnt)  # Find node number for the contour

            if cnt_area > 1000:  # If area >1000, its an obstacle!
                obstacle_list.append([node_num, (cx, cy)])
                if node_num in trav_node:
                    trav_node.remove(node_num)
                continue  # No need to find other parameters

            if node_num == bot_location:  # Skip for cell containing bot
                continue

            '''
            Draw bounding rectangle and find ratio of contour area
            to bounding rectangle area. For squares, this ratio is close to 1
            '''

            x, y, w, h = cv2.boundingRect(cnt)
            rect_area = w * h
            extent = float(cnt_area) / rect_area

            '''
            Code assigned for each shape- shape_code:
            Square = 'S'
            Circle = 'C'
            Triangle = 'T'
            '''
            if extent > 0.75:
                # img = cv2.rectangle(no_bgrnd, (x, y), (x + w, y + h), (0, 255, 0), 2) # Draw rectangle
                shape_code = 'S'
                # print "Node Number: " + str(node_num) + ", Rect Extent: " + str(extent)
            else:
                '''
                If not a square, then draw ellipse and find ratio of contour area
                to bounding ellipse area. For circles, this is very close to 1
                '''
                if len(cnt) < 5:
                    continue

                (x, y), (h, w), angle = cv2.fitEllipse(cnt)  # function return (x,y) position and (h,w) dimensions
                # of rectangle in which ellipse is inscribed
                major_axis = h / 2
                minor_axis = w / 2
                ellipse_area = 3.14159 * (major_axis) * (minor_axis)  # Area of ellipse = pi*a*b
                ellipse_extent = float(cnt_area) / ellipse_area
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)
                cir_area = 3.14159 * radius * radius
                cir_extent = float(cnt_area) / cir_area
                # print "Node Number: " + str(node_num) + ", Ellipse Extent: " + str(ellipse_extent)\
                #       + ", Circle Extent: " + str(cir_extent)
                if ellipse_extent > 0.90 and cir_extent > 0.75:
                    shape_code = 'C'
                else:
                    shape_code = 'T'

            if int(node_num % 10) == 0:  # if node number a multiple of 10, its door area!
                door_area_list.append([node_num, (cx, cy), shape_code, cnt_area])
                if node_num in trav_node:
                    trav_node.remove(node_num)
            else:
                object_list.append([node_num, (cx, cy), shape_code, cnt_area])
                if node_num in trav_node:
                    trav_node.remove(node_num)

        # --------------------------- Shape Detection Ends -------------------------------- #
        # --------------------------------------------------------------------------------- #
        # --------------------------- Color Detection Begins ------------------------------ #

        hsv = cv2.cvtColor(no_bgrnd, cv2.COLOR_BGR2HSV)

        lower_green = np.array([values['g_hue_min'], values['g_sat_min'], values['g_val_min']])
        upper_green = np.array([values['g_hue_max'], values['g_sat_max'], values['g_val_max']])

        lower_blue = np.array([values['b_hue_min'], values['b_sat_min'], values['b_val_min']])
        upper_blue = np.array([values['b_hue_max'], values['b_sat_max'], values['b_val_max']])

        # we detect green and blue colors, remaining is red
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        blue_contours, blue_hierarchy = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, green_hierarchy = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        found_blue = {}
        for cnt in blue_contours:
            cnt_area = cv2.contourArea(cnt)
            if cnt_area > 10:
                cx, cy, node_num = assign_node(cnt)
                if found_blue.has_key(node_num):  # If node already defined, which means object split into two
                    split_cnt_area = found_blue[node_num]
                    found_blue[node_num] = split_cnt_area + cnt_area
                else:
                    found_blue[node_num] = cnt_area

        found_green = {}
        for cnt in green_contours:
            cnt_area = cv2.contourArea(cnt)
            if cnt_area > 10:
                cx, cy, node_num = assign_node(cnt)
                if found_green.has_key(node_num):  # If node already defined, which means object split into two
                    split_cnt_area = found_green[node_num]
                    found_green[node_num] = split_cnt_area + cnt_area
                else:
                    found_green[node_num] = cnt_area

        # if a particular node has blue and green parts, consider colour with larger area
        for blue_node in found_blue.keys():
            for green_node in found_green.keys():
                if blue_node == green_node:
                    blue_area = found_blue[blue_node]
                    green_area = found_green[green_node]

                    if blue_area > green_area:
                        found_green.pop(green_node, None)
                    else:
                        found_blue.pop(blue_node, None)

        # print found_blue
        # print found_green

        for i in range(0, len(object_list)):
            if object_list[i][0] in found_blue.keys():
                object_list[i].insert(3, 'B')  # Blue
            elif object_list[i][0] in found_green.keys():
                object_list[i].insert(3, 'G')  # Green
            else:
                object_list[i].insert(3, 'R')  # Red

        for i in range(0, len(door_area_list)):
            if door_area_list[i][0] in found_blue.keys():
                door_area_list[i].insert(3, 'B')  # Blue
            elif door_area_list[i][0] in found_green.keys():
                door_area_list[i].insert(3, 'G')  # Green
            else:
                door_area_list[i].insert(3, 'R')  # Red

        # ----------------------------- Color Detection Ends ------------------------------ #
        # --------------------------------------------------------------------------------- #
        # ------------------ Find matching 'objects' with 'door color markers' ------------------- #

        for marker in door_area_list:
            for object in object_list:
                # [NodeNumber, Shape, Color, Area] #
                if marker[2:4] == object[2:4]:  # If shape and color match then
                    if percent_variation(marker[4], object[4]) < area_permissible_error:  # If Areas match by a margin then
                        # Match found!
                        if marker[0] in matches:  # If key exists, update it. Else create new key
                            contents = matches[marker[0]]  # Take the contents and update it
                            contents.append(object[0])
                            matches[marker[0]] = contents
                        else:
                            matches[marker[0]] = [object[0]]

        # --------------------------------------------------------------------------------------- #
        cv2.imshow('no_bgrnd', no_bgrnd)

        '''
        For the 30 frames, we get 30 different results. We need the result which is repeated
        maximum number of times.
        To get this, we create a MD5 Hash of the result and store it in a dictionary- matches_array
        with (hash : [count, matches, door_list, object_list, obstacle_list]) as (key:tuple) pair.
        Thus, when a result is repeated we get the same MD5 has and increase the count value.
        '''

        matches_string = str(matches)
        hash_object = hashlib.md5(matches_string)
        hash = hash_object.hexdigest()

        if hash in matches_array:  # if hash exists, increase count
            contents = matches_array[hash]
            contents[0] += 1
            matches_array[hash] = contents
        else:  # else create a new key
            matches_array[hash] = [1, matches.copy(), door_area_list[:], object_list[:], obstacle_list[:]]

        # Cleaning up for the next run!
        del object_list[:]
        del door_area_list[:]
        del obstacle_list[:]
        matches.clear()

        repeat_count += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):  # if 'q' pressed, exit immediately
            break

    '''
    matches_array = {'hash1': [count, matches, door, objects, obstacles]
                     'hash2': [count, matches, door, objects, obstacles]
                     'hash3': [count, matches, door, objects, obstacles]
                     ...
                     }
    matches_array.items() returns a list of [(key1, val1), (key2, val2), ...] tuples

    '''
    max_count = 0
    max_count_hash = 0
    for entity in matches_array.items():
        # Here entity is a tuple = ('hashN', [count, matches, door, objects, obstacles])
        if entity[1][0] > max_count:
            max_count = entity[1][0]
            max_count_hash = entity[0]

    # -------------------------- Final results -------------------------------- #
    door_area_list = matches_array[max_count_hash][2]
    object_list = matches_array[max_count_hash][3]
    obstacle_list = matches_array[max_count_hash][4]
    matches = matches_array[max_count_hash][1]

    matches_array.clear()

    return max_count, trav_node
# ------------------------------------------------------------------- #

cap = cv2.VideoCapture(camera_port) # Create camera object for camera

for i in range(0, 10):  # discard the first 10 frames while camera adjusts to light conditions
    discard_frame = get_image(cap)

max_count, travesable_nodes = process_image(20, cap)

total_matched_markers = len(matches)
rem_markers = total_matched_markers

print "Max count: " + str(max_count)  # Number of time that result occured

# Display result on image
img_raw = get_image(cap)
ret, img, pts = get_boundary_img(img_raw)
write_result(matches, door_area_list, object_list, obstacle_list, img)
cv2.imshow('result', img)

cv2.destroyAllWindows()

# Next, we send directions to the bot for each match

start_time = time.clock()
get_paths(matches.items(), [], bot_location, bot_face, 0, rem_markers)  # Get paths for all matches
end_time = time.clock()

print "Time taken: " + str(end_time - start_time)

for entity in path_array:
    if len(entity) != 0:
        entity.reverse()
        raw_solutions.append(entity)
print "-------------------------"

if len(raw_solutions) == 0:
    sound_buzzer()
    quit()
else:
    raw_solutions.sort()  # Sort in ascending order of completion ratios

max_completion_ratio = raw_solutions[-1][0]
print "Maximum completion ratio: " + str(max_completion_ratio)

if max_completion_ratio == 0:
    print "Max xompletion ratio is 0!"
    sound_buzzer()

solutions_list = []

for solution in raw_solutions:
    if solution[0] == max_completion_ratio:
        solution.reverse()
        solutions_list.append(solution)

print "Total solutions: " + str(len(solutions_list))
# print "Solutions list: " + str(solutions_list)
solutions_list.sort()  # Sort in ascenting order of cost

best_solution = None
for solution in solutions_list:
    if len(solution) > 1:
        best_solution = solution
        break

if best_solution == None:
    print "Best solution is 'None' type!"
    sound_buzzer()

print "Best solution: " + str(best_solution)
print "-------------------------"

best_solution_paths = best_solution[1]

for path in best_solution_paths:
    if len(path) == 22:
        # Contains 6 directions from index 5 to 8 #
        obj1 = path[1]
        obj2 = path[2]
        obj3 = path[3]
        dest1 = path[16]
        dest2 = path[17]
        dest3 = path[18]

        for i in range(7, 13):
            if i == 7:
                object_loc_node = obj1
            elif i == 8:
                object_loc_node = obj2
            elif i == 9:
                object_loc_node = obj3
            elif i == 10:
                object_loc_node = dest1
            elif i == 11:
                object_loc_node = dest2
            elif i == 12:
                object_loc_node = dest3

            directions = path[i]
            print directions
            # send each direction one by one

            for j in range(0, len(directions)):
                cmd = directions[j]

                # combine forwards and reverse which occur repeatedly
                if (cmd == 'forward' or cmd == 'reverse') and (directions[j - 1] != cmd or j < 1):
                    count = 0
                    for k in range(j, len(directions)):
                        if directions[k] == cmd:
                            count += 1
                        else:
                            break
                    send_command(cmd, count, bot_face)
                elif cmd == 'left-forward':
                    send_command('left', 1, bot_face)
                    if bot_face == 'N':  # after turn, change face
                        bot_face = 'W'
                    elif bot_face == 'W':
                        bot_face = 'S'
                    elif bot_face == 'S':
                        bot_face = 'E'
                    else:
                        bot_face = 'N'

                    send_command('forward', 1, bot_face)

                elif cmd == 'right-forward':
                    send_command('right', 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'E'
                    elif bot_face == 'W':
                        bot_face = 'N'
                    elif bot_face == 'S':
                        bot_face = 'W'
                    else:
                        bot_face = 'S'
                    send_command('forward', 1, bot_face)

                elif cmd == 'left':
                    send_command(cmd, 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'W'
                    elif bot_face == 'W':
                        bot_face = 'S'
                    elif bot_face == 'S':
                        bot_face = 'E'
                    else:
                        bot_face = 'N'

                elif cmd == 'right':
                    send_command(cmd, 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'E'
                    elif bot_face == 'W':
                        bot_face = 'N'
                    elif bot_face == 'S':
                        bot_face = 'W'
                    else:
                        bot_face = 'S'

                elif cmd == 'turn-180':
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    bot_cords = assign_node_pt(bot_center) # bot cell coordinates

                    command = turn_decider_180(bot_face, bot_cords)
                    send_command(command, 1, bot_face)

                    if bot_face == 'N':
                        bot_face = 'S'
                    elif bot_face == 'W':
                        bot_face = 'E'
                    elif bot_face == 'S':
                        bot_face = 'N'
                    else:
                        bot_face = 'W'

                elif cmd == 'pickup' or cmd == 'pickup-store' or cmd == 'pickup-store2':
                    send_command(cmd, 1, bot_face)

                    for obj in object_list:  # object picked up, remove from list so that the node is not masked further
                        if obj[0] == object_loc_node:
                            print "Object removed: " + str(object_loc_node)
                            object_list.remove(obj)

                elif cmd == 'drop' or cmd == 'drop-store' or cmd == 'drop-store2':
                    send_command(cmd, 1, bot_face)

    elif len(path) == 15:
        # Contains 4 directions from index 5 to 8 #
        obj1 = path[1]
        obj2 = path[2]
        dest1 = path[11]
        dest2 = path[12]

        for i in range(5, 9):
            if i == 5:
                object_loc_node = obj1
            elif i == 6:
                object_loc_node = obj2
            elif i == 7:
                object_loc_node = dest2
            elif i == 8:
                object_loc_node = dest1

            directions = path[i]
            print directions
            # send each direction one by one

            for j in range(0, len(directions)):
                cmd = directions[j]

                # combine forwards and reverse which occur repeatedly
                if (cmd == 'forward' or cmd == 'reverse') and (directions[j - 1] != cmd or j < 1):
                    count = 0
                    for k in range(j, len(directions)):
                        if directions[k] == cmd:
                            count += 1
                        else:
                            break
                    send_command(cmd, count, bot_face)
                elif cmd == 'left-forward':
                    send_command('left', 1, bot_face)
                    if bot_face == 'N':  # after turn, change face
                        bot_face = 'W'
                    elif bot_face == 'W':
                        bot_face = 'S'
                    elif bot_face == 'S':
                        bot_face = 'E'
                    else:
                        bot_face = 'N'

                    send_command('forward', 1, bot_face)

                elif cmd == 'right-forward':
                    send_command('right', 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'E'
                    elif bot_face == 'W':
                        bot_face = 'N'
                    elif bot_face == 'S':
                        bot_face = 'W'
                    else:
                        bot_face = 'S'
                    send_command('forward', 1, bot_face)

                elif cmd == 'left':
                    send_command(cmd, 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'W'
                    elif bot_face == 'W':
                        bot_face = 'S'
                    elif bot_face == 'S':
                        bot_face = 'E'
                    else:
                        bot_face = 'N'

                elif cmd == 'right':
                    send_command(cmd, 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'E'
                    elif bot_face == 'W':
                        bot_face = 'N'
                    elif bot_face == 'S':
                        bot_face = 'W'
                    else:
                        bot_face = 'S'

                elif cmd == 'turn-180':
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    bot_cords = assign_node_pt(bot_center)  # bot cell coordinates

                    command = turn_decider_180(bot_face, bot_cords)
                    send_command(command, 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'S'
                    elif bot_face == 'W':
                        bot_face = 'E'
                    elif bot_face == 'S':
                        bot_face = 'N'
                    else:
                        bot_face = 'W'


                elif cmd == 'pickup' or cmd == 'pickup-store' or cmd == 'pickup-store2':
                    send_command(cmd, 1, bot_face)

                    for obj in object_list:  # object picked up, remove from list so that the node is not masked further
                        if obj[0] == object_loc_node:
                            print "Object removed: " + str(object_loc_node)
                            object_list.remove(obj)

                elif cmd == 'drop' or cmd == 'drop-store' or cmd == 'drop-store2':
                    send_command(cmd, 1, bot_face)

    elif len(path) == 8:
        # Contains 2 directions from index 3 to 4 #
        obj = path[1]
        dest = path[6]

        for i in range(3, 5):
            if i == 3:
                object_loc_node = obj
            elif i == 4:
                object_loc_node = dest

            directions = path[i]
            print directions
            for j in range(0, len(directions)):
                cmd = directions[j]

                # combine forwards and reverse which occur repeatedly
                if (cmd == 'forward' or cmd == 'reverse') and (directions[j - 1] != cmd or j < 1):
                    count = 0
                    for k in range(j, len(directions)):
                        if directions[k] == cmd:
                            count += 1
                        else:
                            break
                    send_command(cmd, count, bot_face)
                elif cmd == 'left-forward':
                    send_command('left', 1, bot_face)
                    if bot_face == 'N':  # after turn, change face
                        bot_face = 'W'
                    elif bot_face == 'W':
                        bot_face = 'S'
                    elif bot_face == 'S':
                        bot_face = 'E'
                    else:
                        bot_face = 'N'

                    send_command('forward', 1, bot_face)

                elif cmd == 'right-forward':
                    send_command('right', 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'E'
                    elif bot_face == 'W':
                        bot_face = 'N'
                    elif bot_face == 'S':
                        bot_face = 'W'
                    else:
                        bot_face = 'S'
                    send_command('forward', 1, bot_face)

                elif cmd == 'left':
                    send_command(cmd, 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'W'
                    elif bot_face == 'W':
                        bot_face = 'S'
                    elif bot_face == 'S':
                        bot_face = 'E'
                    else:
                        bot_face = 'N'

                elif cmd == 'right':
                    send_command(cmd, 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'E'
                    elif bot_face == 'W':
                        bot_face = 'N'
                    elif bot_face == 'S':
                        bot_face = 'W'
                    else:
                        bot_face = 'S'

                elif cmd == 'turn-180':
                    left_center, bot_center, right_center, bot_angle = get_pos_orientation(cap)
                    bot_cords = assign_node_pt(bot_center)  # bot cell coordinates

                    command = turn_decider_180(bot_face, bot_cords)
                    send_command(command, 1, bot_face)
                    if bot_face == 'N':
                        bot_face = 'S'
                    elif bot_face == 'W':
                        bot_face = 'E'
                    elif bot_face == 'S':
                        bot_face = 'N'
                    else:
                        bot_face = 'W'

                elif cmd == 'pickup' or cmd == 'pickup-store' or cmd == 'pickup-store2':
                    send_command(cmd, 1, bot_face)
                    for obj in object_list:  # object picked up, remove from list so that the node is not masked further
                        if obj[0] == object_loc_node:
                            print "Object removed: " + str(object_loc_node)
                            object_list.remove(obj)
                elif cmd == 'drop' or cmd == 'drop-store' or cmd == 'drop-store2':
                    send_command(cmd, 1, bot_face)


# Task done! Hurray!
sound_buzzer()

ser.close()
cap.release()
cv2.destroyAllWindows()

