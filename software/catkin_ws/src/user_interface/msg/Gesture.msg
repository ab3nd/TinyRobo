# Messages for non-touch UI events (buttons, and gestures so far)

#E.g. the button that was pressed to generate it
string eventName 

bool isButton

time stamp

#Robots affected by this gesture
uint32[] robots

#Stroke(s) used to create the event (could be none for button presses)
user_interface/Stroke[] strokes