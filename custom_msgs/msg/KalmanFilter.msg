Header header
int32 state_num                   # number of states
float32[] x_predict             # prediction of a state
float32[] x                     # states (updates, measurements included) 
float32[] z                     # measurements
float32[] covariance            # filter covariance
