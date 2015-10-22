# Representation of a motion phase, described by a set of motion models

# Identifier of this motion phase
string id

# Object to be manipulated
string object

# Threshold for reaching the attractor
float64 threshold

# attractor pose matrix
float64[] attractor

# List of motion models active during this phase
MotionModel[] described_by_motion_model
