
import __future__
import cv2
import matplotlib
import numpy as np

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
plt.rcParams.update({'figure.max_open_warning': 0})
import time
import math

from scipy.stats import norm
import scipy.interpolate as interpolate

'''
#Below code work for straight lines only
def show_image(title, img):
    cv2.imwrite("/constraint_model/images/{}.png".format(title), img)

SIZE = 500
img = cv2.imread("/constraint_model/images/new_image_m.png")
img = cv2.resize(img, (SIZE, SIZE))

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# detect edges
edges = cv2.Canny(gray, 150, 300)



show_image("edges", edges)

# create mask
mask = np.zeros(img.shape[:2], dtype = "uint8") # 0 - 255 = 8 bits

# white pentagon
#pts = np.array([[0, SIZE * 3 / 4], [SIZE / 2, SIZE / 2], [SIZE, SIZE * 3 / 4], [SIZE, SIZE], [0, SIZE]], np.int32)

# black triangle
#pts2 = np.array([[SIZE / 2, 0], [SIZE / 4, SIZE], [SIZE * 3 / 4, SIZE]], np.int32)

#cv2.fillPoly(mask, [pts], 255)

#cv2.fillPoly(mask, [pts2], 0)

#show_image("mask", mask)

# get lines
# (x1, y1, x2, y2)
lines = cv2.HoughLinesP(
    edges,
    rho=1.0,
    theta=np.pi/180,
    threshold=20,
    minLineLength=30,
    maxLineGap=30        
)

# draw lines
line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
line_color = [0, 255, 0]
line_thickness = 2
dot_color = [0, 255, 0]
dot_size = 3

for line in lines:
    for x1, y1, x2, y2 in line:
        print(line)
        cv2.line(line_img, (x1, y1), (x2, y2), line_color, line_thickness)
        cv2.circle(line_img, (x1, y1), dot_size, dot_color, -1)
        cv2.circle(line_img, (x2, y2), dot_size, dot_color, -1)
        
line_img = cv2.bitwise_and(line_img, line_img)

overlay = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

show_image("Overlay", overlay)

# Based on this: https://stackoverflow.com/questions/63727525/how-to-connect-broken-lines-that-cannot-be-connected-by-erosion-and-dilation, 
# find the m and b and line equation, then find other line points are touching 

def calculate_slope(line_object):
    x_point1 = line_object[0]
    y_point1 = line_object[1]
    x_point2 = line_object[2]
    y_point2 = line_object[3]

    m = abs((y_point2 - y_point1) / (x_point2 - x_point1))
    m = float("{:.2f}".format(m))
    return m

    
for index in range(len(lines)):
    x1, y1, x2, y2 = lines[index][0]
    
    # Finding the slope m = (y2-y1)/x2-x1)
    m = (y2-y1)/(x2-x1)
    b = y2-m*x2
    
    # Now we have the equation y = mx+b, check the quation can give similar points to neighbouring lines
    for n_index in range(index+1, len(lines)):
        n_x1, n_y1, n_x2, n_y2 = lines[n_index][0]
        y = m*n_x1+b
        if y==n_y1:
            #print("y:{} and y1:{}".format(y,n_y1))
            cv2.line(overlay,
                pt1=(n_x1, n_y1),
                pt2=(x2, y2),
                color=(0, 255, 0),
                thickness=3)
            
    show_image("Overlay1", overlay)
        
'''    
'''
img = cv2.imread("/constraint_model/images/new_image_m.png")
print(img.shape)
(winW, winH) = (40, 40)
step_size = 40
nonzero = img.nonzero()
nonzeroy = np.array(nonzero[0])
nonzerox = np.array(nonzero[1])

def sliding_window(image, stepSize, windowSize):
    # slide a window across the image
	for y in range(0, image.shape[0], stepSize):
		for x in range(0, image.shape[1], stepSize):
			# yield the current window
			yield (x, y, image[y:y + windowSize[1], x:x + windowSize[0]])


lane_points_x = {}
lane_points_y = {}

lane_points_ids = {}


for (x, y, window) in sliding_window(img, stepSize=step_size, windowSize=(winW, winH)):
    # if the window does not meet our desired window size, ignore it
    if window.shape[0] != winH or window.shape[1] != winW:
        continue
    # THIS IS WHERE YOU WOULD PROCESS YOUR WINDOW, SUCH AS APPLYING A
    # MACHINE LEARNING CLASSIFIER TO CLASSIFY THE CONTENTS OF THE
    # WINDOW
    # since we do not have a classifier, we'll just draw the window
    
    
    top_left_x = x
    top_left_y = y
    bottom_left_x = x
    bottom_left_y = y+winH
    
    top_right_x = x + winW
    top_right_y = y
    bottom_right_x = x + winW
    bottom_right_y = y + winH
    
    good_inds = ((nonzeroy >= top_left_y) & (nonzeroy < bottom_right_y) & 
        (nonzerox >= top_left_x) &  (nonzerox < bottom_right_x)).nonzero()[0]

    
    if y in lane_points_ids: 
        lane_points_ids[y].append(good_inds)
    else:
        lane_points_ids[y] = []
        lane_points_ids[y].append(good_inds)

    
    #clone = img.copy()
    #cv2.rectangle(clone, (x, y), (x + winW, y + winH), (0, 255, 0), 2)
    #cv2.imshow("Window", clone)
    #cv2.waitKey(1)
    #time.sleep(1)
    

for key in lane_points_ids.keys():
    lane_points_ids[key] = np.concatenate(lane_points_ids[key])
    lanex = nonzerox[lane_points_ids[key]]
    laney = nonzeroy[lane_points_ids[key]]
    nonzero_combined = []
    for index in range(len(laney.tolist())):
        nonzero_combined.append([laney[index],lanex[index]])
    nonzero_combined = sorted(nonzero_combined, key=lambda x: x[0])
    
    laney = []
    lanex = []
    for index in range(len(nonzero_combined)):
        proceed = True
        if nonzero_combined[index][0] in laney:
            if nonzero_combined[index][1] in lanex:
                proceed = False
        if proceed:
            laney.append(nonzero_combined[index][0])
            lanex.append(nonzero_combined[index][1])

    
    x = np.array(lanex)
    y = np.array(laney)
    lane_fit = np.polyfit(y, x, 3)
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
    #lane_fitx = lane_fit[0]*ploty**2 + lane_fit[1]*ploty + lane_fit[2]
    lane_fitx =lane_fit[0]*ploty**3+ lane_fit[1]*ploty**2+lane_fit[2]*ploty + lane_fit[3]

    img[y, x] = [255, 0, 0]
    plt.imshow(img)
    plt.plot(lane_fitx, ploty, color='yellow')
plt.xlim(0, img.shape[1])
plt.ylim(img.shape[0], 0)
plt.savefig('images/fit.png')
    

'''


# Chekcing the item in any of the group
def check_group(group, item):
    status = False
    for index in range(len(group)):
        if index in group and item in group[index]:
            status = True
    return status

def sliding_window(img = None, selected_pair = None):
    
     # Create an output image to draw on and visualize the result
    out_img = np.dstack((img, img, img))
    
    # left side is top lane and right side is down lane
    left_starting_point = selected_pair[0]
    right_starting_point = selected_pair[1]


    # HYPERPARAMETERS
    # Choose the number of sliding windows
    nwindows = 9

    # Set minimum number of pixels found to recenter window
    minpix = 50

    # set the height of windows - based on nwindows above and image shape. You need to see from the perspective of y axis as x axis. then width will be height and height will be width
    # and we are moving from left to right parallel to x axis. Here top lane considered as left lane and down lane considered as right lane.
    window_height = np.int(img.shape[1]//nwindows)

    # Set the width of the windows +/- margin
    margin = 10


    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Current positions to be updated later for each window in nwindows
    lefty_current = left_starting_point
    righty_current = right_starting_point

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []


    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_x_high = (window+1)*window_height #66
        win_x_low = window*window_height #33
        
        ### TO-DO: Find the four below boundaries of the window ###
        win_yleft_low = lefty_current - margin # 331
        win_yleft_high = lefty_current + margin # 351
        win_yright_low = righty_current - margin
        win_yright_high = righty_current + margin
        
        left_first_point = (win_x_low, win_yleft_low)
        left_fourth_point = (win_x_high,win_yleft_high)
        
        right_first_point = (win_x_low, win_yright_low)
        right_fourth_point = (win_x_high,win_yright_high)
        
        ### Identify the nonzero pixels in x and y within the window ###
        good_left_inds = ((nonzerox >= win_x_low) & (nonzerox < win_x_high) & 
        (nonzeroy >= win_yleft_low) &  (nonzeroy < win_yleft_high)).nonzero()[0]
        good_right_inds = ((nonzerox >= win_x_low) & (nonzerox < win_x_high) & 
        (nonzeroy >= win_yright_low) &  (nonzeroy < win_yright_high)).nonzero()[0]
        
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        ### If you found > minpix pixels, recenter next window ###
        ### (`right` or `leftx_current`) on their mean position ###
            
        if len(good_left_inds) > minpix:
            lefty_current = np.int(np.mean(nonzeroy[good_left_inds]))
        if len(good_right_inds) > minpix:        
            righty_current = np.int(np.mean(nonzeroy[good_right_inds]))
            
        print(lefty_current)
        print(righty_current)
        print("___________")
        
        #clone = img.copy()
        #cv2.rectangle(clone, left_first_point, left_fourth_point, (0, 255, 0), 2)
        #cv2.rectangle(clone, right_first_point, right_fourth_point, (0, 255, 0), 2)
        #cv2.imshow("Window", clone)
        #cv2.waitKey(1)
        #time.sleep(1)

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        # Avoids an error if the above is not implemented fully
        pass

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    

    return leftx, lefty, rightx, righty, out_img


img = cv2.imread("/constraint_model/images/new_image.png")
sum_array = np.sum(img[:,:img.shape[1]], axis=1)


# 1. Find peaks of the array
# 2. Mean of each distribution is the starting points for both one and two lane's sliding window one
# 3. Get all the lines in the sliding window
# 2. Middle point of two lines
# 3 . Apply logic of advanced lane finding to fit the ploynomial

print(sum_array.shape)
flatten_array_y = [each[0] for each in sum_array]
hist, bin_edges = np.histogram(flatten_array_y, density=True)
plt.hist(hist)
plt.savefig('images/hist_m.png')

#FInding the peaks in the y data
from scipy.signal import find_peaks
peaks, _ = find_peaks(flatten_array_y, height=1000)
print("All peaks over 1000: {}".format(peaks))


# Grouping items that are closer
group = {}
for index in range(len(peaks)):
    item = peaks[index]
    status = check_group(group, item)
    if status == False:
        if index not in group:
            group[index] = []
            group[index].append(item)
        for sub_index in range(len(peaks)):
            sub_item = peaks[sub_index]
            if index == sub_index:
                continue

            diff = abs(item-sub_item)
            if diff < 20:
                group[index].append(sub_item)

selected_pair = []

# Finding the highest peak in each group
for key in group.keys():
    items_peak = [flatten_array_y[item] for item in group[key]]
    items = [item for item in group[key]]
    highest = 0
    highest_index = None
    for index in range(len(items_peak)):
        if items_peak[index] > highest:
            highest = items_peak[index]
            highest_index = index
    selected_pair.append(items[highest_index])
    
print("Selected pair of peaks: {}".format(selected_pair))

leftx, lefty, rightx, righty, out_img = sliding_window(img=img, selected_pair=selected_pair)

# Fit a second order polynomial to each using `np.polyfit` ###
left_fit = np.polyfit(leftx, lefty, 2)
right_fit = np.polyfit(rightx, righty, 2)

# Generate x and y values for plotting
plotx = np.linspace(0, img.shape[1]-1, img.shape[1] )
try:
    left_fity = left_fit[0]*plotx**2 + left_fit[1]*plotx + left_fit[2]
    right_fity = right_fit[0]*plotx**2 + right_fit[1]*plotx + right_fit[2]
except TypeError:
    # Avoids an error if `left` and `right_fit` are still none or incorrect
    print('The function failed to fit a line!')
    left_fitx = 1*plotx**2 + 1*plotx
    right_fitx = 1*plotx**2 + 1*plotx
    

print
    
## Visualization ##
# Colors in the left and right lane regions
img[lefty, leftx] = [255, 0, 0]
img[righty, rightx] = [0, 0, 255]

# Plots the left and right polynomials on the lane lines
plt.plot(plotx,left_fity, color='red')
plt.plot(plotx,right_fity, color='red')
plt.savefig("images/polyfit.png")
