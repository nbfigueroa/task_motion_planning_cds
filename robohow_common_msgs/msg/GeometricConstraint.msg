# ID assigned to the constraint in the knowledge base
string name

# expression to constrain
GeometricExpression expression

# box constraint boundaries
float64 lower
float64 higher

# trajectory generator used for interpolation
#TODO(Georg&Enea): add this!

# behavior
#TODO(Georg&Enea): add this?

# gain of the proportional controller for this constraint
float64 p_gain

# weight of this constraint
#TODO(Georg&Enea): is this necessary when we have p_gain and 'task hierachies'?
float64 weight
