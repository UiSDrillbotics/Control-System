import scipy.io as sio
import numpy as np
import pylab

class KalmanFilter(object):

    def __init__(self, Q, R):
        self.Q = Q                                      # Auto-covariance for process noise
        self.R = R                                      # Auto-covariance for measurement noise
        self.xhat = 0.0                                 # A posteriori condition estimate
        self.Phat = 1.0                                 # A posteriori estimation deviation

    def input_latest_WOB_data(self, measurement):
# Predict
        xbar = self.xhat                                # A priori condition estimate
        Pbar = self.Phat + self.Q                       # A priori estimation deviation
# Update
        K = Pbar / (Pbar + self.R)                      # Kalman gain
        self.xhat = xbar + K * (measurement - xbar)     # A posteriori condition estimate
        self.Phat = (1 - K) * Pbar                      # A posteriori estimation deviation

    def get_latest_estimated_measurement(self):
        return self.xhat

if __name__ == "__main__":
# Get data from .MAT (matlab file) or data matrix
    data_read = sio.loadmat(r'D:\Google Drive\1. MASTERS THESIS 2018\1. MATLAB\SIMULINK\Torsional parameter estimation\090518test1_600Hz.MAT', struct_as_record=False)
# Relevant data from matlab file
    WOB_data = data_read['Channel_11_Data']
# Sampling time of the data set
    SamplingTime = data_read['Channel_1_Data']
# Length of data set.
    iteration_count = len(SamplingTime)
# xrange for Python 3.6 and range for 2.7 (both 32 bit)
    xrange=range

# Standard deviation of measurement data set
    measurement_std = np.std([WOB_data * 2.0 - 1.0 for j in xrange(iteration_count)])

    Q = 0.05e-3
    R = measurement_std ** 2
    kalman_filter = KalmanFilter(Q, R)
    xhat_est = []
   
    for iteration in xrange(0, iteration_count):
        kalman_filter.input_latest_WOB_data(WOB_data[iteration])
        xhat_est.append(kalman_filter.get_latest_estimated_measurement())

# Plot measurement versus a posteriori estimate
    pylab.figure()
    pylab.plot(SamplingTime,WOB_data, color='r', label='Load cell measurement')
    pylab.plot(SamplingTime,xhat_est, 'b-', label='xhat, a posteriori estimate')
    pylab.autoscale(enable=True, axis='x', tight=True)
    pylab.legend()
    pylab.xlabel('Time(s)')
    pylab.ylabel('Load Cell (gram)')
    pylab.title('Estimated load cell measurement')
    pylab.show()