# Allows to modify the state of a constraint online
# string identifier for the controller used
string controller_id

# hash identifier for this motion
int64 movement_id #TODO: remove?

# specify position ranges for the current constraint set
# which was specified using ConstraintConfig.msg
# NOTE: arrays in this and that message all need to have
# same size
float64[] pos_lo	# lower bound
float64[] pos_hi	# upper bound
string    selec   # selection of the dof handled (only used for the position constraint)
                  # TODO: detailed explanation

float64[] gain    # exponential gain (1/3 parameters)

