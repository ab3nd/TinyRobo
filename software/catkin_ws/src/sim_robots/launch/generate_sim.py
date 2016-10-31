#!/usr/bin/python
# Generate launch files for a bunch of robots

#Number of robots to launch
robotCount = 3

#Name of the launch file to generate
fName = "./sim_{0}_robots.launch".format(robotCount)

#TODO eventually, each robot will have to have a translator based on how the motors are connected to the drivetrain

with open(fName, "w") as launchFile:
    launchFile.write("<launch>")
    robotNames = []
    for ii in range(robotCount):
        botName = "bot_{0}".format(ii)
        robotNames.append(botName)
        launchFile.write(
'''\n  <node name="{1}" pkg="sim_robots" type="sim_robot" output="screen">
    <!-- Which drive translator should this robot listen to -->
      <param name="translator" type="str" value="/trans_bot_{0}"/>
  </node>
  <node name="drive_bot_{0}" pkg="drive_logic" type="random_driver" output="screen"/>
  <node name="trans_bot_{0}" pkg="motor_translation" type = "abstract_trans" output="screen">
    <!-- The driver that this translation node listens to -->
    <param name="driver_name" type="str" value="/drive_bot_{0}/drive_cmd"/>
  </node>\n'''.format(ii, botName))
    
    #Write a node to launch the simulated world and pass it a list of all the robots to subscribe to
    robotList = ",".join(robotNames)
    robotList = "[" + robotList + "]"
    launchFile.write(
'''\n  <node name="the_world" pkg="sim_robots" type="sim_world" output="screen">
    <rosparam param="robot_list">{0}</rosparam>
  </node>
'''.format(robotList))
    launchFile.write("</launch>")
    