#!/usr/bin/python
# Generate launch files for a bunch of robots

#Number of robots to launch
robotCount = 100

#Name of the launch file to generate
fName = "./sim_{0}_robots.launch".format(robotCount)

with open(fName, "w") as launchFile:
    launchFile.write("<launch>")
    launchFile.write('''   <node name="sim_world" pkg="sim_robots" type="sim_world" output="screen"/>''')
    for ii in range(robotCount):
        launchFile.write(
'''\n  <node name="bot_{0}" pkg="sim_robots" type="sim_robot" output="screen">
    <!-- Which drive translator should this robot listen to -->
      <param name="translator" type="str" value="/trans_bot_{0}"/>
  </node>
  <node name="drive_bot_{0}" pkg="drive_logic" type="random_driver" output="screen"/>
  <node name="trans_bot_{0}" pkg="motor_translation" type = "abstract_trans" output="screen">
    <!-- The driver that this translation node listens to -->
    <param name="driver_name" type="str" value="/drive_bot_{0}/drive_cmd"/>
  </node>\n'''.format(ii))
    launchFile.write("</launch>")
    