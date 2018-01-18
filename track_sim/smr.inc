
# Desc: Device definitions for "Small Mobile Robot" from DTU
# Author: Lars Valdemar Mogensen and Anders Billesø Beck
# Date: 3 Oct 2008
# CVS: $Id: smr.inc,v 1.30 2008-01-15 01:25:42 rtv Exp $

# The AUT Sharp IR distance sensor
define AUT_irSensor ranger
(
  scount 5 # the number of IR sensors

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [ 0.135  0.135  90 ]
  spose[1] [ 0.14   0.09    0 ]
  spose[2] [ 0.14   0.00    0 ]
  spose[3] [ 0.14  -0.09    0 ]
  spose[4] [ 0.135 -0.135 -90 ]
    
  # define the field of view of each IR sensor [range_min range_max view_angle]
  sview [0.1 0.8 15]

  # define the size of each transducer [xsize ysize] in meters
  ssize [0.02 0.02]

  # define the color of the sensors
  color "black"
)

#Using a blobfinder to emulate the linesensor module
define AUT_linesensor blobfinder 
( 
  colors_count 1 
  colors [ "blue" ]
  #image[ 8 6 ]
  #range 1.25       #Range at 5cm
  #fov   0.75  #180 degrees FOV (Total 10cm range)
  alwayson 1
)

define AUT_fiducial fiducial
(
  # fiducial properties
  range_min 0.0
  range_max 8.0
  range_max_id 5.0
  fov 180

  # model properties
  size [ 0.1 0.1 0.1 ]
  color "cyan"
)

# The AUT line sensor
define AUT_lineSensor linesensor
(
  scount 8 # the number of  sensors

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [ 0.14  -0.105  0 ]
  spose[1] [ 0.14  -0.075  0 ]
  spose[2] [ 0.14  -0.045  0 ]
  spose[3] [ 0.14  -0.015  0 ]
  spose[4] [ 0.14   0.015  0 ]
  spose[5] [ 0.14   0.045  0 ]
  spose[6] [ 0.14   0.075  0 ]
  spose[7] [ 0.14   0.105  0 ]  

  # define the size of each transducer [xsize ysize] in meters
  ssize [0.02 0.02]

  # define the color of the sensors
  color "black"
)

define smr_base position 
(
  color "grey"			# Default color.
  drive "diff"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 1           	# Can hit things.
  laser_return 1                 # reflects laser beams
  ranger_return 1             	# reflects sonar beams
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"
  localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  # localization "odom"         	# Change to "gps" to have impossibly perfect, global odometry
  #  odom_error [ 0.05 0.05 0.1 ]   	# Odometry error or slip in X, Y and Theta
                                   	# (Uniform random distribution)   
)

define smr smr_base
(
  # actual size
  size [0.37 0.28 0.38] # sizes from MobileRobots' web site

  # the pioneer's center of rotation is offset from its center of area
  origin [0.095 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 8.0 

  #Test a blobfinder as linesensor
  #AUT_linesensor ( pose [0.0 0 0 0] )

  # use the URG laser scanner
  urg04lx( pose [0.255 0 -0.37 0] )

  # use the AUT IR distance sensor
  AUT_irSensor( pose [0.08 0 -0.20 0])
  
  # use the AUT  line sensor
  AUT_lineSensor( pose [0.08 0 -0.30 0])
  
  AUT_fiducial( pose [0.5 0.0 -0.15 0])
  
  # differential steering model
  drive "diff"
  
  # Top part body
  block 
  (
	 points 4
	 point[0] [  0.14  0.14 ]
	 point[1] [ -0.14  0.14 ]
	 point[2] [ -0.14 -0.14 ]
	 point[3] [  0.14 -0.14 ]
	 z [0.07 0.29]
	 color "grey"
  )

  # Bottom part body
  block 
  (
	 points 4
	 point[0] [  0.13  0.10 ]
	 point[1] [ -0.14  0.10 ]
	 point[2] [ -0.14 -0.10 ]
	 point[3] [  0.13 -0.10 ]
	 z [0.04 0.07]
	 color "grey"
  )

  # URG laser scanner housing
  block 
  (
    points 4
    point[0] [  0.20  0.03 ]
    point[1] [  0.14  0.03 ]
    point[2] [  0.14 -0.03 ]
    point[3] [  0.20 -0.03 ]
    z [0.095 0.10]
    color "grey"
  )

  # Camera stand
  block 
  (
    points 4
    point[0] [ -0.14 -0.02 ]
    point[1] [ -0.17 -0.02 ]
    point[2] [ -0.17 -0.05 ]
    point[3] [ -0.14 -0.05 ]
    z [0.06 0.38]
    color "grey"
  )

  # Camera house
  block 
  (
    points 4
    point[0] [ -0.14  0.015 ]
    point[1] [ -0.17  0.015 ]
    point[2] [ -0.17 -0.015 ]
    point[3] [ -0.14 -0.015 ]
    z [0.41 0.44]
    color "red"
  )

  # Camera lense
  block 
  (  
    points 4
    point[0] [ -0.11  0.015 ]
    point[1] [ -0.14  0.015 ]
    point[2] [ -0.14 -0.015 ]
    point[3] [ -0.11 -0.015 ]
    z [0.41 0.44]
    color "black"
  )
  
  # Left back wheel
  block
  (
    points 4
    point[0] [ -0.065 0.14 ]
    point[1] [ -0.125 0.14 ]
    point[2] [ -0.125 0.11 ]
    point[3] [ -0.065 0.11 ]
    z [0 0.065]
    color "black"
  )
  
  # Right back wheel
  block 
  (
    points 4
    point[0] [ -0.065 -0.11 ]
    point[1] [ -0.125 -0.11 ]
    point[2] [ -0.125 -0.14 ]
    point[3] [ -0.065 -0.14 ]
    z [0 0.065]
    color "black"
   )

  # Left front wheel
  block
  (
    points 4
    point[0] [  0.13  0.13 ]
    point[1] [  0.08  0.13 ]
    point[2] [  0.08  0.11 ]
    point[3] [  0.13  0.11 ]
    z [0 0.065]
    color "black"
  )

  # Right front wheel
  block
  (
    points 4
    point[0] [  0.13  -0.11 ]
    point[1] [  0.08  -0.11 ]
    point[2] [  0.08  -0.13 ]
    point[3] [  0.13  -0.13 ]
    z [0 0.065]
    color "black"
  )

)
