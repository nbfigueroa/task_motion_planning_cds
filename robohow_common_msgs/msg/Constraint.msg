# Definition of a constraint

string name      # 'human readable' name for the constraint

uint8 ANGLE=0		 # computes the angle between the two features
uint8 DISTANCE=1	 # computes the distance between the two features
uint8 POSITION=2	 # compute the difference of position between the two features
uint8 OTHER=3	 # compute the difference of position between the two features

uint8 function    # name of the constraint function ('perpendicular', etc.)

Feature tool_feature
Feature world_feature

ConstraintCommand command # initial parameterization of the constraint.
