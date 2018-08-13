# ROS Message to wrap (most of) a Kivy mouse event
# The Kivy event fields are as follows (this is a tap event, so a lot of the delta and time feilds are boring)
# <MouseMotionEvent 
#	button="left"                      - Mouse button, probably always left for touches
# 	device="mouse"                     - Device that created this touch
#	double_tap_time="0" 
#	dpos="(0.0, 0.0)"                  - Change in position since previous event
#	dsx="0.0"                          - Change in x, y, and z since previous event, 0-1 range 
#	dsy="-0.000952380952381" 
#	dsz="0.0" 
#	dx="0.0"                           - Change in x, y, and z since previous event, window range
#	dy="0.0" 
#	dz="0.0" 
#	grab_current="None"                - Used to determine which widget the touch is dispatched to
#	grab_exclusive_class="None" 
#	grab_list="[]" 
#	grab_state="False" 
#	id="mouse2"                        - ID of the touch source, not unique
#	is_double_tap="False"              - True if Kivy detected this as a doubltap
#	is_mouse_scrolling="False"         - True for mousewheel scrolling 
#	is_touch="True"                    - True for touch events
#	is_triple_tap="False"              - True if Kivy detected this as a tripletap 
#	opos="(577.0, 952.0)"              - "Origin" x and y, in window range (pixels)
#	osx="0.343452380952"               - "Origin" x, y, and z, as proportion of screen (range 0-1)
#	osy="0.908571428571" 
#	osz="0.0" 
#	ox="577.0"                         - "Origin" x, y, and z in window range (pixels)
#	oy="952.0" 
#	oz="0.0" 
#	pos="(577.0, 952.0)"               - Position in window range
#	ppos="(577.0, 952.0)"              - Previous position, in window range
#	profile="['pos', 'button']"        - Profile of the event, 'pos' means this is a touch
#	psx="0.343452380952" 			   - Previous position x, y, and z as proportion of screen
#	psy="0.907619047619"               
#	psz="0.0" 
#	push_attrs="('x', 'y', 'z', 'dx', 'dy', 'dz', 'ox', 'oy', 'oz', 'px', 'py', 'pz', 'pos')" - Attributes to get pushed by Kivy's push()
#	push_attrs_stack="[]" 
#	px="577.0"                         - Previous x, y, and z, in window range
#	py="952.0" 
#	pz="0.0" 
#	shape="None"                       - Shape of the touch, defaults to "None"
#	spos="(0.34345238095238095, 0.9066666666666666)"      - Position, in the 0-1 coordinate system, probably of the dispatching widget
#	sx="0.343452380952"                - X, y, and z position in the 0-1 coordinate system of the dispatching widget?
#	sy="0.906666666667" 
#	sz="0.0" 
#	time_end="1533144946.81"           - Time of the end of the event
#	time_start="1533144946.81"         - Time of the start of the event
#	time_update="1533144946.81"        - Time the event was last updated
#	triple_tap_time="0"                - Time between first touch and current event
#	ud="{}"                            - Dictionary for user data
#	uid="3"                            - Unique ID of the touch point
#	x="577.0"                          - X, y, and z of the event, in window range
#	y="952.0" 
#	z="0.0"
# >

#Needless to say, we don't care about a lot of that
geometry_msgs/Point  point
bool   isDoubletap
bool   isTripletap
bool   ended
time   start
time   end
time   update
uint32 uid
