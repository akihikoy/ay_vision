uint8 LINE=0        #param= [x1,y1,x2,y2]
uint8 CIRCLE=1      #param= [x,y,rad]
uint8 RECTANGLE=2   #param= [x,y,w,h]
uint8 POLYGON=3     #param= [x0,y0,x1,y1,...]
uint8 FILLED_POLY=4 #param= [x0,y0,x1,y1,...]
uint8 REVERSED_POLY=5 #param= [x0,y0,x1,y1,...]

int32 type

std_msgs/ColorRGBA color  #Alpha (a) is not used.
float64[] param
float64 line_width
