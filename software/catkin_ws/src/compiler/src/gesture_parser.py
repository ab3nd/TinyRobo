#!/usr/bin/python

from lark import Lark

gesture_grammar='''
	select : tap_select | box_select | lasso_select | select_group
	tap_select : "tap_robot"+ 
	box_select : "box_select"
	lasso_select : "lasso_select"
	select_group : "tap_robot" "select_group" | "select_group" "tap_robot" //order doesn't matter

	patrol : "patrol"
	formation : "formation"
	move : "move_object"
	remove : "remove_object"
	
	disperse : "disperse_gesture"

	drag_path : "drag_robot"

	path : "path" | "tap_waypoint"+ //may need to add option to treat box/lasso as path

	patrol_cmd : select? patrol path
	formation_cmd : select? formation path
	move_obj_cmd : select? move path 
	disperse_cmd : select? disperse
	path_cmd : drag_path | select? path

	start : (patrol_cmd | formation_cmd | move_obj_cmd | disperse_cmd | path_cmd) end

	end : "end"

	%import common.WS
	%ignore WS
'''

l = Lark(gesture_grammar)

print l.parse("tap_robot select_group disperse_gesture end").pretty()