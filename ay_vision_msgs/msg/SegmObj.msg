std_msgs/Header header
uint32 width
uint32 height

#Number of points in each contour
int32[] num_points

#Sequence of contours.
#Each contour is a sequence of points (x,y).
#Number of contour = len(num_points)
#Number of contour i = num_points(i)
#len(contours) = sum(num_points)*2
int32[] contours
