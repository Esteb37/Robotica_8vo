import numpy as np
from KF import KF
import matplotlib.pyplot as plt

# Variables del sistema S = [x; v]  z = x + r
real_x = 0
meas_x = 0
ref_x = 0
error_x = 0
KX = 0.05

meas_variance = 0.05
accel_variance = 0
mov_error = 0
MOV_VARIANCE = 15

# Parámetros de simulacion
DT = 0.1
STEPS = 1000
MEAS_STEP = 100

# Vectores de resultados
x_real = []
x_meas = []
x_ref = []
mean_kf = []
cov_kf = []

# Filtro de Kalman
kf = KF(0, 0.5)

# Simulación
for step in range(STEPS):
	ref_x = ref_x + 0.5 * DT

	# Preddicción
	mov_error +=  + np.random.random() * (MOV_VARIANCE / STEPS)
	real_x = kf.mean[0] + mov_error

	# Error
	error_x = ref_x - real_x

	# Control
	accel_variance = KX * error_x

	kf.predict(DT, accel_variance)

	if (step != 0) and (step % MEAS_STEP == 0):
		meas_x = real_x + np.random.randn() * meas_variance

		# Corrección
		kf.correct(meas_x, meas_variance)
		mov_error = 0

	# Guardar resultados
	x_real.append(real_x)
	x_meas.append(meas_x)
	x_ref.append(ref_x)
	mean_kf.append(kf.mu)
	cov_kf.append(kf.sigma)

# Graficar resultados de posición
plt.subplot(2, 1, 1)
plt.plot(x_real, 'b')
plt.plot(x_ref, 'g')
plt.plot(x_meas, 'y')
plt.plot([mean[0] for mean in mean_kf], 'r')

# Graficar resultados de velocidad
plt.subplot(2, 1, 2)
plt.plot([mean[1] for mean in mean_kf], 'r')

plt.show()