# Representation of a GMM-based motion model

# Identifier of this motion model
string id

# Type of the motion model, e.g. CDSModel or GMRModel
string type

# List of GMMs that encode this motion
GaussianMixtureModel[] described_by_GMM
