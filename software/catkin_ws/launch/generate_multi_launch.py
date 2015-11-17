#!/usr/bin/python

#Generate an e-puck launch file

pucks = {"2117":"10:00:E8:AD:77:AE"
         "2099":"10:00:E8:AD:77:6E",
         "2046":"10:00:E8:AD:5B:D7",
         "2137":"10:00:E8:AD:77:C2",
         "2097":"10:00:E8:AD:77:ED",
         "2110":"10:00:E8:AD:76:FD",
         "2039":"10:00:E8:AD:5B:AA",
         "2028":"10:00:E8:AD:5B:B5",
         "2180":"10:00:E8:AD:77:F5",
         "2087":"10:00:E8:AD:77:B4",
         "2151":"10:00:E8:AD:79:B3"
         }
         

print "<launch>"
#Addresses of all robots
for robot in pucks.keys():
    argName = "robot_{0}_addr"
    print "    <arg name=\"{0}\" value=\"{1}\"/>".format(argName, pucks[robot])
    pucks[robot] = [pucks[robot], argName]

print "    <param name=\"robot_description\" textfile=\"$(find epuck_driver)/urdf/epuck_urdf.xml\"/>"

#positions, used for RVIZ, kind of bogus
xPos = 0
yPos = 0
for robot in pucks.keys():
    robotName = "epuck_robot_{0}".format(robot)
    print "    <group ns=\"{0}\">".format(robotName)
    print '''        <include file=\"$(find epuck_driver)/launch/epuck_controller.launch\">
            <arg name=\"epuck_address\" value=\"{0}\"/>
            <arg name=\"epuck_name\" value=\"{1}\"/>
            <arg name=\"cam_en\" value=\"false\"/>
            <arg name=\"acc_en\" value=\"false\"/>
            <arg name=\"prox_en\" value=\"true\"/>
            <arg name=\"mot_pos_en\" value=\"true\"/>
            <arg name=\"light_en\" value=\"false\"/>
            <arg name=\"floor_en\" value=\"false\"/>
            <arg name=\"sel_en\" value=\"false\"/> <!--this command is available only in ascii mode-->
            <arg name=\"mot_speed_en\" value=\"false\"/>
            <arg name=\"mic_en\" value=\"false\"/>
            <arg name=\"xpos\" value=\"{2}\"/>
            <arg name=\"ypos\" value=\"{3}\"/>
            <arg name=\"theta\" value=\"0.0\"/>
            <arg name=\"is_single_robot\" value=\"0\"/>
        </include>'''.format(pucks[robot][0], robotName, xPos, yPos)
    print "    </group>"
    
    #Grid layout for robots in RVIZ
    xPos += 0.1
    if xPos >= 0.5:
        yPos += 0.1
        if yPos >= 0.5:
            yPos = 0
        xPos = 0
        	
print "<node pkg=\"rviz\" type=\"rviz\" output=\"screen\" name=\"rviz\" args=\"-d $(find epuck_driver)/config/multi_epuck_driver_rviz.rviz\"/>"
print "</launch>"         
