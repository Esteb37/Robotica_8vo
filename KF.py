import numpy as np

class KF:

    Q = np.array([[0.1, 0], [0, 0.1]])
    C = np.array([[1, 0]])

    # Inicialización
    def __init__(self, init_x, init_v):
        self.mu = np.zeros(2)
        self.mu[0] = init_x
        self.mu[1] = init_v
        self.sigma = np.eye(2)

    # Funcion de prediccion
    def predict(self, dt, variance):
      A = np.array([[1, dt], [0, 1]])
      B = np.array([0.5*dt*dt, dt])

      self.mu = A.dot(self.mu) + B * variance
      self.sigma = A.dot(self.sigma).dot(A.T) + self.Q


    # Funcion de correccion
    def correct(self, meas_x, meas_variance):
      print(self.C.dot(self.sigma).dot(self.C.T) + meas_variance)
      K = self.sigma.dot(self.C.T).dot(np.linalg.inv(self.C.dot(self.sigma).dot(self.C.T) + meas_variance))
      self.mu = self.mu + K.dot(meas_x - self.C.dot(self.mu))
      self.sigma = self.sigma - K.dot(self.C).dot(self.sigma)

    @property
    def mean(self):
        return self.mu

    @property
    def cov(self):
        return self.sigma