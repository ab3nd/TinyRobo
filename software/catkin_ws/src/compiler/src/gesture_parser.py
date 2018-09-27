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

	tap_waypoint : "tap_waypoint"
	path : "path" | tap_waypoint+ //may need to add option to treat box/lasso as path

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

live_attempts=[
"lasso_select path tap_waypoint path tap_waypoint end",
"tap_waypoint path path tap_waypoint end",
"tap_waypoint path path tap_waypoint tap_waypoint end",
"path tap_select path tap_waypoint tap_waypoint end",
"tap_waypoint path tap_select end",
"tap_waypoint path tap_select end",
"path tap_waypoint lasso_select path path end",
"tap_waypoint path end",
"box_select path tap_waypoint tap_waypoint end",
"path tap_waypoint tap_waypoint box_select path end",
"path path tap_waypoint end",
"lasso_select tap_waypoint path path end",
"box_select path tap_waypoint end",
"box_select path path end",
"path path path path tap_select path path tap_waypoint path tap_waypoint end",
"path path path tap_select tap_select path path tap_select path end",
"tap_waypoint path end",
"box_select lasso_select end",
"box_select path path end",
"lasso_select path end",
"lasso_select Patrol path tap_waypoint path tap_waypoint end",
"lasso_select Formation path end",
"lasso_select Move Object path end",
"tap_waypoint Remove Robot path tap_waypoint end",
"remove_robot path tap_select tap_waypoint end",
"path remove_robot tap_waypoint end",
"tap_select select_group tap_waypoint end",
"path tap_select select_group select_group tap_select path path tap_waypoint path tap_waypoint end",
"select_group tap_select end",
"tap_waypoint tap_select select_group tap_waypoint end",
"tap_waypoint end"]

for line in live_attempts:
	print l.parse(line).pretty()