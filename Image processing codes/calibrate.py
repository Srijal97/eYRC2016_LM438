import cv2
import numpy as np
import math

'''
* Team Id: LM#438
* Authors: Srijal Poojari
* Filename: calibrate.py
* Theme: eYRC-Launch a Module, 2016
* Description: This program provides sliders to calibrate colour and threshold values
*              for values used in the main program. The calibrated values are stored in
*              the text file 'values.txt' and are loaded up by the main program.
'''


camera_port = 1
values_file = open('values.txt', 'r+')

values = {}

for line in values_file:  # Read existing values
    content = line.split('.', 2)
    if len(content) == 2:
        values[content[0]] = int(content[1])

values_file.seek(0, 0)

allowed_block_size = [3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39]
allowed_blur = [0, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19]

thresh_mode = 0
blur_val = 0
lower_thresh = 0
upper_thresh = 255
block_size = 0
C_val = 0

lower_thresh_bgrnd = 0
blur_val_bgrnd = 0
sat_min = 0
sat_max = 255

g_hue_min = 0
g_sat_min = 0
g_val_min = 0
g_hue_max = 255
g_sat_max = 255
g_val_max = 255

b_hue_min = 0
b_sat_min = 0
b_val_min = 0
b_hue_max = 255
b_sat_max = 255
b_val_max = 255

p_hue_min = 0
p_sat_min = 0
p_val_min = 0
p_hue_max = 255
p_sat_max = 255
p_val_max = 255


def nothing():
    pass

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
def get_boundary_img(src_img, thresh_mode, blur_val, lower_thresh, upper_thresh, block_size, C_val):
    img_gray = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)

    img_gray = cv2.medianBlur(img_gray, blur_val)

    if thresh_mode == 0:
        ret1, img_thresh = cv2.threshold(img_gray, lower_thresh, upper_thresh, cv2.THRESH_BINARY)
    else:
        img_thresh = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
                                           block_size, C_val)


    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ''' hierarchy = [[[NextContour, PrevContour, FirstChild, Parent]]] '''

    # For all contours with perimeter > 1000, take the lowest. This would be the inner boundary of arena.
    perimeter_list = []
    for i in range(0, len(contours)):
        perimeter = cv2.arcLength(contours[i], True)
        if perimeter > 1000:
            perimeter_list.append((perimeter, i))

    if len(perimeter_list) == 0:
        return -1, src_img, []

    perimeter_list.sort()
    boundary_cnt = perimeter_list[0][1]

    # Draw bounding rectangle for this inner boundary.
    rect = cv2.minAreaRect(contours[boundary_cnt])
    box = list(cv2.cv.BoxPoints(rect))
    box_pts = []

    # convert float cords to int and tuple to list
    for tuples in box:
        box_pts.append( list( (int(tuples[0]), int(tuples[1])) ) )

    box_pts = get_cw_points(box_pts)  # convert in clockwise order

    # Get 4 corner points for boundary contour and arrange in clockwise order
    epsilon = 0.01 * cv2.arcLength(contours[boundary_cnt], True)
    approx = cv2.approxPolyDP(contours[boundary_cnt], epsilon, True)

    approx_pts_list = []
    final_pts_list = []
    for pt in approx:
        approx_pts_list.append(list(pt[0])) # store points as a list, not tuple.

    '''
     Now we have 2 points' lists: approxPolyDP points and boundingRect points
     approxPolyDP may sometimes return more than 4 points but bounding rectangle
     always has 4 points.
     So we take 4 approxPolyDP points which are geometrically closest to the bounding
     rectangle points
    '''
    for pt1 in box_pts:  # For every point in bounding rect, find closest approx poly dp point
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
    # print "final matched points: " + str(final_pts_list)

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

# Create a black image, a window
cap = cv2.VideoCapture(camera_port)

# ----------------------------- Get boundary Image --------------------------------------- #
cv2.namedWindow('settings', flags=cv2.WINDOW_AUTOSIZE)

cv2.createTrackbar('Sel_Thresh', 'settings', 0, 1, nothing)
cv2.createTrackbar('median_blur', 'settings', 0, 19, nothing)
cv2.createTrackbar('lower_thresh', 'settings', 0, 255, nothing)
cv2.createTrackbar('upper_thresh', 'settings', 0, 255, nothing)
cv2.createTrackbar('adaptive_block_size', 'settings', 3, 39, nothing)
cv2.createTrackbar('adaptive_C', 'settings', 0, 30, nothing)

while not cv2.waitKey(1) & 0xFF == ord('q'):
    ret_val, src_img = cap.read()
    img_gray = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)

    switch_pos = cv2.getTrackbarPos('Sel_Thresh', 'settings')

    blur_val = cv2.getTrackbarPos('median_blur', 'settings')

    if switch_pos == 0:
        # Normal thresh #
        thresh_mode = 0
        lower_thresh = cv2.getTrackbarPos('lower_thresh', 'settings')
        upper_thresh = cv2.getTrackbarPos('upper_thresh', 'settings')

        if blur_val not in allowed_blur:
            print "blur value not allowed!\n"
            continue

        img_gray = cv2.medianBlur(img_gray, blur_val)
        ret1, img_thresh = cv2.threshold(img_gray, lower_thresh, upper_thresh, cv2.THRESH_BINARY)
    else:
        # Adaptive thresh #
        thresh_mode = 1
        block_size = cv2.getTrackbarPos('adaptive_block_size', 'settings')
        C_val = cv2.getTrackbarPos('adaptive_C', 'settings')

        if blur_val not in allowed_blur:
            print "blur value not allowed!\n"
            continue

        if block_size not in allowed_block_size:
            print "block size not allowed!\n"
            continue

        img_gray = cv2.medianBlur(img_gray, blur_val)
        img_thresh = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                           cv2.THRESH_BINARY, block_size, C_val)

    cv2.imshow('thresh', img_thresh)
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(src_img, contours, -1, (255, 255, 0), 2)
    cv2.imshow('source', src_img)

print "Save settings?\ny:yes, n:no"
while True:
    if cv2.waitKey(1) & 0xFF == ord('n'):  # Save old values
        values_file.write('reposition.' + '0' + '\n')
        values_file.write('0.0\n')
        values_file.write('10.0\n')
        values_file.write('20.0\n')
        values_file.write('30.0\n')
        values_file.write('40.0\n')
        values_file.write('50.0\n')
        values_file.write('*\n')
        values_file.write('thresh_mode.' + str(values['thresh_mode']) + '\n')
        values_file.write('blur_val.' + str(values['blur_val']) + '\n')
        values_file.write('lower_thresh.' + str(values['lower_thresh']) + '\n')
        values_file.write('upper_thresh.' + str(values['upper_thresh']) + '\n')
        values_file.write('block_size.' + str(values['block_size']) + '\n')
        values_file.write('C_val.' + str(values['C_val']) + '\n')
        values_file.write('*\n')
        print "Not Saved!"
        break
    if cv2.waitKey(1) & 0xFF == ord('y'):  # Update values
        values_file.write('reposition.' + '0' + '\n')
        values_file.write('0.0\n')
        values_file.write('10.0\n')
        values_file.write('20.0\n')
        values_file.write('30.0\n')
        values_file.write('40.0\n')
        values_file.write('50.0\n')
        values_file.write('*\n')
        values_file.write('thresh_mode.' + str(thresh_mode) + '\n')
        values_file.write('blur_val.' + str(blur_val) + '\n')
        values_file.write('lower_thresh.' + str(lower_thresh) + '\n')
        values_file.write('upper_thresh.' + str(upper_thresh) + '\n')
        values_file.write('block_size.' + str(block_size) + '\n')
        values_file.write('C_val.' + str(C_val) + '\n')
        values_file.write('*\n')

        values['thresh_mode'] = thresh_mode
        values['blur_val'] = blur_val
        values['lower_thresh'] = lower_thresh
        values['upper_thresh'] = upper_thresh
        values['block_size'] = block_size
        values['C_val'] = C_val
        print "Saved!"
        break

cv2.destroyAllWindows()

# ------------------------------- Remove background -------------------------------------- #
cv2.namedWindow('settings', flags=cv2.WINDOW_AUTOSIZE)

cv2.createTrackbar('lower_thresh', 'settings', 0, 255, nothing)
cv2.createTrackbar('median_blur', 'settings', 0, 19, nothing)
cv2.createTrackbar('sat_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('sat_max', 'settings', 0, 255, nothing)

while not cv2.waitKey(1) & 0xFF == ord('q'):
    ret, src_img = cap.read()
    ret, src_img, pts = get_boundary_img(src_img, values['thresh_mode'], values['blur_val'],
                                         values['lower_thresh'], values['upper_thresh'],
                                         values['block_size'], values['C_val'])

    lower_thresh_bgrnd = cv2.getTrackbarPos('lower_thresh', 'settings')
    blur_val_bgrnd = cv2.getTrackbarPos('median_blur', 'settings')
    sat_min = cv2.getTrackbarPos('sat_min', 'settings')
    sat_max = cv2.getTrackbarPos('sat_max', 'settings')

    if blur_val_bgrnd not in allowed_blur:
        print "blur value not allowed! \n"
        continue

    img_gray = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)

    ret, img_thresh = cv2.threshold(img_gray, lower_thresh_bgrnd, 255, cv2.THRESH_BINARY_INV)

    img_thresh = cv2.medianBlur(img_thresh, blur_val_bgrnd)
    res = cv2.bitwise_and(src_img, src_img, mask=img_thresh)
    hsv = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, sat_min, 0])
    upper_white = np.array([255, sat_max, 255])

    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    # cv2.imshow('mask_white', mask_white)
    final_mask = cv2.bitwise_and(res, res, mask=mask_white)
    cv2.imshow('final_mask', final_mask)


print "Save settings?\ny:yes, n:no"
while True:
    if cv2.waitKey(1) & 0xFF == ord('n'):  # Save old values
        values_file.write('lower_thresh_bgrnd.' + str(values['lower_thresh_bgrnd']) + '\n')
        values_file.write('blur_val_bgrnd.' + str(values['blur_val_bgrnd']) + '\n')
        values_file.write('sat_min.' + str(values['sat_min']) + '\n')
        values_file.write('sat_max.' + str(values['sat_max']) + '\n')
        values_file.write('*\n')
        print "Not Saved!"
        break
    if cv2.waitKey(1) & 0xFF == ord('y'):  # Update values
        values_file.write('lower_thresh_bgrnd.' + str(lower_thresh_bgrnd) + '\n')
        values_file.write('blur_val_bgrnd.' + str(blur_val_bgrnd) + '\n')
        values_file.write('sat_min.' + str(sat_min) + '\n')
        values_file.write('sat_max.' + str(sat_max) + '\n')
        values_file.write('*\n')

        values['lower_thresh_bgrnd'] = lower_thresh_bgrnd
        values['blur_val_bgrnd'] = blur_val_bgrnd
        values['sat_min'] = sat_min
        values['sat_max'] = sat_max
        print "Saved!"
        break

cv2.destroyAllWindows()

# ------------------------------- Blue and Green masking -------------------------------------- #
cv2.namedWindow('settings', flags=cv2.WINDOW_NORMAL)

cv2.createTrackbar('g_hue_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('g_sat_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('g_val_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('g_hue_max', 'settings', 0, 255, nothing)
cv2.createTrackbar('g_sat_max', 'settings', 0, 255, nothing)
cv2.createTrackbar('g_val_max', 'settings', 0, 255, nothing)

cv2.createTrackbar('b_hue_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('b_sat_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('b_val_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('b_hue_max', 'settings', 0, 255, nothing)
cv2.createTrackbar('b_sat_max', 'settings', 0, 255, nothing)
cv2.createTrackbar('b_val_max', 'settings', 0, 255, nothing)

while not cv2.waitKey(1) & 0xFF == ord('q'):
    ret, src_img = cap.read()
    ret, src_img, pts = get_boundary_img(src_img, values['thresh_mode'], values['blur_val'],
                                         values['lower_thresh'], values['upper_thresh'],
                                         values['block_size'], values['C_val'])

    img_gray = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
    ret, img_thresh = cv2.threshold(img_gray, values['lower_thresh_bgrnd'], 255, cv2.THRESH_BINARY_INV)
    img_thresh = cv2.medianBlur(img_thresh, values['blur_val_bgrnd'])

    res = cv2.bitwise_and(src_img, src_img, mask=img_thresh)

    hsv = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, values['sat_min'], 0])
    upper_white = np.array([255, values['sat_max'], 255])

    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    no_bgrnd = cv2.bitwise_and(res, res, mask=mask_white)
    cv2.imshow('final_mask', no_bgrnd)

    g_hue_min = cv2.getTrackbarPos('g_hue_min', 'settings')
    g_sat_min = cv2.getTrackbarPos('g_sat_min', 'settings')
    g_val_min = cv2.getTrackbarPos('g_val_min', 'settings')
    g_hue_max = cv2.getTrackbarPos('g_hue_max', 'settings')
    g_sat_max = cv2.getTrackbarPos('g_sat_max', 'settings')
    g_val_max = cv2.getTrackbarPos('g_val_max', 'settings')

    b_hue_min = cv2.getTrackbarPos('b_hue_min', 'settings')
    b_sat_min = cv2.getTrackbarPos('b_sat_min', 'settings')
    b_val_min = cv2.getTrackbarPos('b_val_min', 'settings')
    b_hue_max = cv2.getTrackbarPos('b_hue_max', 'settings')
    b_sat_max = cv2.getTrackbarPos('b_sat_max', 'settings')
    b_val_max = cv2.getTrackbarPos('b_val_max', 'settings')

    hsv = cv2.cvtColor(no_bgrnd, cv2.COLOR_BGR2HSV)

    lower_green = np.array([g_hue_min, g_sat_min, g_val_min])
    upper_green = np.array([g_hue_max, g_sat_max, g_val_max])

    lower_blue = np.array([b_hue_min, b_sat_min, b_val_min])
    upper_blue = np.array([b_hue_max, b_sat_max, b_val_max])

    # we detect green and blue colors, remaining is red
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    cv2.imshow('mask_green', mask_green)
    cv2.imshow('mask_blue', mask_blue)


print "Save settings?\ny:yes, n:no"
while True:
    if cv2.waitKey(1) & 0xFF == ord('n'):  # Save old values
        values_file.write('g_hue_min.' + str(values['g_hue_min']) + '\n')
        values_file.write('g_sat_min.' + str(values['g_sat_min']) + '\n')
        values_file.write('g_val_min.' + str(values['g_val_min']) + '\n')
        values_file.write('g_hue_max.' + str(values['g_hue_max']) + '\n')
        values_file.write('g_sat_max.' + str(values['g_sat_max']) + '\n')
        values_file.write('g_val_max.' + str(values['g_val_max']) + '\n')

        values_file.write('b_hue_min.' + str(values['b_hue_min']) + '\n')
        values_file.write('b_sat_min.' + str(values['b_sat_min']) + '\n')
        values_file.write('b_val_min.' + str(values['b_val_min']) + '\n')
        values_file.write('b_hue_max.' + str(values['b_hue_max']) + '\n')
        values_file.write('b_sat_max.' + str(values['b_sat_max']) + '\n')
        values_file.write('b_val_max.' + str(values['b_val_max']) + '\n')

        values_file.write('*\n')
        print "Not Saved!"
        break
    if cv2.waitKey(1) & 0xFF == ord('y'):  # Update values
        values_file.write('g_hue_min.' + str(g_hue_min) + '\n')
        values_file.write('g_sat_min.' + str(g_sat_min) + '\n')
        values_file.write('g_val_min.' + str(g_val_min) + '\n')
        values_file.write('g_hue_max.' + str(g_hue_max) + '\n')
        values_file.write('g_sat_max.' + str(g_sat_max) + '\n')
        values_file.write('g_val_max.' + str(g_val_max) + '\n')

        values_file.write('b_hue_min.' + str(b_hue_min) + '\n')
        values_file.write('b_sat_min.' + str(b_sat_min) + '\n')
        values_file.write('b_val_min.' + str(b_val_min) + '\n')
        values_file.write('b_hue_max.' + str(b_hue_max) + '\n')
        values_file.write('b_sat_max.' + str(b_sat_max) + '\n')
        values_file.write('b_val_max.' + str(b_val_max) + '\n')

        values['g_hue_min'] = g_hue_min
        values['g_sat_min'] = g_sat_min
        values['g_val_min'] = g_val_min
        values['g_hue_max'] = g_hue_max
        values['g_sat_max'] = g_sat_max
        values['g_val_max'] = g_val_max

        values['b_hue_min'] = b_hue_min
        values['b_sat_min'] = b_sat_min
        values['b_val_min'] = b_val_min
        values['b_hue_max'] = b_hue_max
        values['b_sat_max'] = b_sat_max
        values['b_val_max'] = b_val_max

        values_file.write('*\n')
        print "Saved!"
        break

cv2.destroyAllWindows()

# ------------------------------- Pink masking -------------------------------------- #
cv2.namedWindow('settings', flags=cv2.WINDOW_NORMAL)

cv2.createTrackbar('p_hue_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('p_sat_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('p_val_min', 'settings', 0, 255, nothing)
cv2.createTrackbar('p_hue_max', 'settings', 0, 255, nothing)
cv2.createTrackbar('p_sat_max', 'settings', 0, 255, nothing)
cv2.createTrackbar('p_val_max', 'settings', 0, 255, nothing)

while not cv2.waitKey(1) & 0xFF == ord('q'):
    ret, src_img = cap.read()
    ret, src_img, pts = get_boundary_img(src_img, values['thresh_mode'], values['blur_val'],
                                         values['lower_thresh'], values['upper_thresh'],
                                         values['block_size'], values['C_val'])

    img_gray = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
    ret, img_thresh = cv2.threshold(img_gray, values['lower_thresh_bgrnd'], 255, cv2.THRESH_BINARY_INV)
    img_thresh = cv2.medianBlur(img_thresh, values['blur_val_bgrnd'])

    res = cv2.bitwise_and(src_img, src_img, mask=img_thresh)

    hsv = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, values['sat_min'], 0])
    upper_white = np.array([255, values['sat_max'], 255])

    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    no_bgrnd = cv2.bitwise_and(res, res, mask=mask_white)
    cv2.imshow('final_mask', no_bgrnd)

    p_hue_min = cv2.getTrackbarPos('p_hue_min', 'settings')
    p_sat_min = cv2.getTrackbarPos('p_sat_min', 'settings')
    p_val_min = cv2.getTrackbarPos('p_val_min', 'settings')
    p_hue_max = cv2.getTrackbarPos('p_hue_max', 'settings')
    p_sat_max = cv2.getTrackbarPos('p_sat_max', 'settings')
    p_val_max = cv2.getTrackbarPos('p_val_max', 'settings')

    hsv = cv2.cvtColor(no_bgrnd, cv2.COLOR_BGR2HSV)

    lower_pink = np.array([p_hue_min, p_sat_min, p_val_min])
    upper_pink = np.array([p_hue_max, p_sat_max, p_val_max])

    mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)

    cv2.imshow('mask_pink', mask_pink)


print "Save settings?\ny:yes, n:no"
while True:
    if cv2.waitKey(1) & 0xFF == ord('n'):  # Save old values
        values_file.write('p_hue_min.' + str(values['p_hue_min']) + '\n')
        values_file.write('p_sat_min.' + str(values['p_sat_min']) + '\n')
        values_file.write('p_val_min.' + str(values['p_val_min']) + '\n')
        values_file.write('p_hue_max.' + str(values['p_hue_max']) + '\n')
        values_file.write('p_sat_max.' + str(values['p_sat_max']) + '\n')
        values_file.write('p_val_max.' + str(values['p_val_max']) + '\n')

        values_file.write('*\n')
        print "Not Saved!"
        break
    if cv2.waitKey(1) & 0xFF == ord('y'):  # Update values
        values_file.write('p_hue_min.' + str(p_hue_min) + '\n')
        values_file.write('p_sat_min.' + str(p_sat_min) + '\n')
        values_file.write('p_val_min.' + str(p_val_min) + '\n')
        values_file.write('p_hue_max.' + str(p_hue_max) + '\n')
        values_file.write('p_sat_max.' + str(p_sat_max) + '\n')
        values_file.write('p_val_max.' + str(p_val_max) + '\n')

        values['p_hue_min'] = p_hue_min
        values['p_sat_min'] = p_sat_min
        values['p_val_min'] = p_val_min
        values['p_hue_max'] = p_hue_max
        values['p_sat_max'] = p_sat_max
        values['p_val_max'] = p_val_max

        values_file.write('*\n')
        print "Saved!"
        break

cv2.destroyAllWindows()

values_file.close()
