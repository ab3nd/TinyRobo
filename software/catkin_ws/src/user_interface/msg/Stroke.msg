# ROS Message to wrap (most of) a sequence of Kivy-based touch events that have been destuttered
# Most of the message content is based on the first and last events of the event list 

user_interface/Kivy_Event[] events
geometry_msgs/Point centroid
bool   isDoubletap
bool   isTripletap
time   start
time   end
uint32 uid
uint32 width
uint32 height
