#!/usr/bin/python

#Given a list of pairs of bluetooth MAC and epuck id, generate a launch file to launch all the 
#epuck driver nodes

epuck_list = [("10:00:E8:AD:77:ED", "2097"),
			  ("10:00:E8:AD:77:C2", "2137"),
			  ("10:00:E8:AD:79:B3", "2151"),
			  ("10:00:E8:AD:77:AE", "2117"),
			  ("10:00:E8:AD:5B:B5", "2028"),
			  ("10:00:E8:AD:77:F5", "2180"),
			  ("10:00:E8:AD:5B:D7", "2046"),
			  ("10:00:E8:AD:77:6E", "2099"),
			  ("10:00:E8:AD:5B:AA", "2039"),
			  ("10:00:E8:AD:76:FD", "2110"),
			  ("10:00:E8:AD:77:B4", "2087")]

robot_config = '''    <param name="robot_description" textfile="$(find epuck_driver)/urdf/epuck_urdf.xml"/>'''

def genStanza(robot_number, robot_id, robot_addr, x, y, theta):
	robot_stanza = '''
    <group ns="epuck_robot_{0}">
        <include file="$(find epuck_driver_cpp)/launch/epuck_controller.launch">
            <arg name="epuck_id" value="{1}"/>
            <arg name="epuck_address" value="{2}"/>
            <arg name="epuck_name" value="epuck_robot_{0}"/>
            <arg name="cam_en" value="false"/>
            <arg name="acc_en" value="false"/>
            <arg name="prox_en" value="true"/>
            <arg name="mot_pos_en" value="true"/>
            <arg name="floor_en" value="false"/>
            <arg name="mot_speed_en" value="false"/>
            <arg name="mic_en" value="false"/>
            <arg name="xpos" value="{3}"/>
            <arg name="ypos" value="{4}"/>
            <arg name="theta" value="{5}"/>
            <arg name="is_single_robot" value="0"/>
        </include>
        <!--<node pkg="tf" type="static_transform_publisher" name="epuck_robot_{0}_tf" args="0 0 0 0 0 0 /base_link /epuck_robot_{0}/base_link 30"/>-->
    </group>'''.format(robot_number, robot_id, robot_addr, x, y, theta)
	return robot_stanza

with open ("launch_all_epucks.launch", "w") as main_launchfile:
	#Print header for main file
	main_launchfile.write("<launch>\n")
	main_launchfile.write(robot_config)
	main_launchfile.write("\n")

	#Print all the robot stanzas
	#Attempt to arrange the robots into a square grid in rviz. 
	#Note that if they're not actually in a square, your idea of the robots' positions will be wrong forever
	import math
	#If the robots are in a square, how many are along one edge?
	edge_len = math.ceil(math.sqrt(len(epuck_list)))
	#Accumulators for position
	col = 0
	row = 0
	#How far apart the robots are, in meters
	space = 0.1

	for index, pair in enumerate(epuck_list):
		#Place x position, increment for next robot
		xpos = col * space
		ypos = row * space
		col += 1
		#Check for wrap to next row
		if col > edge_len:
			col = 0
			row += 1

		#Write it all to a launchfile
		subfilename = "epuck_{0}.launch".format(pair[1])
		with open(subfilename, "w") as robot_launchfile:
			robot_launchfile.write("<launch>\n")
			robot_launchfile.write(genStanza(index, pair[1], pair[0], xpos, ypos, 0.0))
			robot_launchfile.write("\n</launch>\n")
			#Include it in the main launchfile
			main_launchfile.write('''    <include file="{0}" />\n'''.format(subfilename))
	#Print the rviz config and close the file
	main_launchfile.write('''    <node pkg="rviz" type="rviz" output="screen" name="rviz" args="-d $(find epuck_driver)/config/multi_epuck_driver_rviz.rviz"/>''')
	main_launchfile.write("\n</launch>")