# Representation of a Gaussian Mixture Model (GMM)

# Identifier of this GMM
string id

# Type of the GMM, for example CouplingGMM
string type

# Strings describing the types (i.e. physical quantities) for input and
# output. Examples are 'Force' or 'Position' (without quotes)
string input_type
string output_type

# String encoding of the semantics of the input and output slots. Examples
# are 'x-y', '*-*', or 'Other' (without quotes)
string input_dim
string output_dim

# Vector of GaussianDistributions describing the distribution of this
# mixture model
GaussianDistribution[] gaussian_dist
