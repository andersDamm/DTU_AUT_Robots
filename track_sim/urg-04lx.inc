
define urg04lx laser
(
  # laser-specific properties

  # factory settings for Hokuyo URG-04LX	
  range_max 4.00
  fov 180.0
  samples 500

  resolution 1
  #100ms between each sample
  interval 100

  # generic model properties
  color "grey"
  size [ 0.050 0.050 0.071 ] # dimensions from Hokuyo URG-04LX data sheet	
)

define URGlaser urg04lx
(

#
  # bottom
  block 
  (
    points 4
    point[0] [  0.025  0.025 ]
    point[1] [ -0.025  0.025 ]
    point[2] [ -0.025 -0.025 ]
    point[3] [  0.025 -0.025 ]
    z [0 0.041 ]
    color "grey"
  )

   # laser house
  block 
  (
    points 8
    point[0] [  0.02    0.00   ]
    point[1] [  0.0141  0.0141 ]
    point[2] [  0.0     0.02   ]
    point[3] [ -0.0141  0.0141 ]
    point[4] [ -0.02    0.0    ]
    point[5] [ -0.0141 -0.0141 ]
    point[6] [  0.0    -0.02   ]
    point[7] [  0.0141 -0.0141 ]
    z [0.041 0.071 ]
    color "black"
  )
)



