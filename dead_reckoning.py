import numpy as np


pose = np.array([[0.0],
                 [0.0],
                 [0.0]])

theta = 0.0

covariance = np.zeros((3,3))

Qk = np.array([[ 0.5, 0.01, 0.01],
               [0.01,  0.5, 0.01],
               [0.01, 0.01,  0.2]])
Vk = 1.0 # m/s
Wk = 1.0 # rad/s
dt = 0.1 # s


for i in range(3):

	print(pose,"\n\n", covariance)

	if i > 0:
		print("\n", Hk)

	print("\n-------\n")


	pose += np.array([[dt*Vk*np.cos(theta)],
					  [dt*Vk*np.sin(theta)],
					  [Wk*dt]])

	Hk = np.array([ [1, 0, -dt*Vk*np.sin(theta)],
					[0, 1, dt*Vk*np.cos(theta)],
					[0, 0, 1]])

	covariance = Hk.dot(covariance).dot(Hk.T) + Qk

	theta = pose[2,0]
