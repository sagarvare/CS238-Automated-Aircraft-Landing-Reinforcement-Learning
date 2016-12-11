import numpy as np 
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from const import Const


max_t = 50
vopt_filename = "V_OPT_2D-Y_PROFILE.pdf"

v_opt_by_time = np.zeros((max_t,Const.BINS_Y))

color_choices = ['black', 'blue', 'cyan', 'green', 'orange', 'red']
time_choices = [1, 3, 5, 10, 20 , 50]
v_opt_choices = []

y_min = 2 * (0.5 * Const.BIN_SIZE_Y + Const.Y_MIN) / (Const.Y_MAX_RUNWAY - Const.Y_MIN_RUNWAY)
y_max = 2 * ((Const.BINS_Y - 0.5) * Const.BIN_SIZE_Y + Const.Y_MIN) / (Const.Y_MAX_RUNWAY - Const.Y_MIN_RUNWAY)
y_range = np.linspace(y_min, y_max, Const.BINS_Y)

plt.ioff()
plt.figure()

for i, t in enumerate(time_choices):

	filename_vopt = "Dynamic_programming_vopt_t=" + str(t) + ".txt"

	v_opt = np.loadtxt(filename_vopt)
	v_opt = v_opt.reshape([Const.BINS_Y, Const.BINS_VY, Const.BINS_VW])

	v_opt_avgs = np.sum(v_opt,axis = (1,2))/(Const.BINS_VY * Const.BINS_VW)

	plt.plot(y_range, v_opt_avgs, color = color_choices[i])


plt.legend(['t = 1', 't = 3', 't = 5', 't = 10', 't = 20', 't = 50'], loc='lower center')

plt.xlim((y_min,y_max))
plt.xlabel(r'$2y / w$')
plt.ylim((-1500,1000))
plt.ylabel("Averaged Optimal Utility " + r'$\overline{U^{*}}(t,y)$')
plt.title("Utility profile at t seconds from landing")
plt.savefig(vopt_filename)
plt.clf()

  