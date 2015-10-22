# name or id assigned by the knowledge base
string name

# tf frame name. pos, dir and ref_dir are expressed
# in this frame
string frame_id

# description of the feature
# string description

uint8 LINE=0    #
uint8 PLANE=1   #
uint8 POINT=2   #
uint8 VERSOR=3  #

# ...
uint8 type
geometry_msgs/Vector3 position   # center position of the feature
geometry_msgs/Vector3 direction  # direction of the feature.

