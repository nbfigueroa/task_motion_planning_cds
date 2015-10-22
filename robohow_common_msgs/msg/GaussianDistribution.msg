# Representation of a general Gaussian distribution

# Identifier of this Gaussian
string id

# A single value representing the prior of the Gaussian
float64 prior

# A n x 1 vector representing the mean value of the Gaussian
float64[] mean

# A row-major representation of a n x n covariance matrix
# NOTE: This is the same 'n' as for the mean vector
float64[] cov
