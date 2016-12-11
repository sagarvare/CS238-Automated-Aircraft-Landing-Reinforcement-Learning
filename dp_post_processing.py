import numpy as np 
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from const import Const

if __name__ == '__main__':
    
    # parameters from dynamic_programming
    max_t = 50
    
    # Initialize file name of figure file
    # Initialize array to store the files after reading them
    vopt_plot_filename = "V_OPT_2D_PLOT.pdf"
    v_opt_by_time = np.zeros((max_t,Const.BINS_Y))
    
    # Read all the files
    for t in range(1, max_t + 1):
        # Read the file
        filename_vopt = "Dynamic_programming_vopt_t=" + str(t) + ".txt"
        v_opt = np.loadtxt(filename_vopt)
        v_opt = v_opt.reshape([Const.BINS_Y, Const.BINS_VY, Const.BINS_VW])
        
        # Compute average
        v_opt_avgs = np.sum(v_opt,axis = (1,2))/(Const.BINS_VY * Const.BINS_VW)
        
        # Store in array
        v_opt_by_time[t-1] = v_opt_avgs
    
    #############################################################################
    #Plot the results for v_opt
    
    # Make X,Y arrays holding axis values for pcolormesh
    X = np.zeros((max_t, Const.BINS_Y))
    Y = np.zeros((max_t, Const.BINS_Y))
    for i in range(max_t):
        for j in range(Const.BINS_Y):
            X[i, j] = i + 1 # time starts from 1
            y_val = ((j + 0.5) * Const.BIN_SIZE_Y) + Const.Y_MIN
            Y[i, j] = 2 * y_val / (Const.Y_MAX_RUNWAY - Const.Y_MIN_RUNWAY)
    
    plt.ioff()
    plt.figure()
    plt.pcolormesh(X, Y, v_opt_by_time)
    plt.colorbar()
    
    # Ensure plot is centered on y-axis
    plt.xlim((max_t,1))
    plt.xlabel("Time " + r'$t$' + " (seconds)")
    y_min = 2 * (0.5 * Const.BIN_SIZE_Y + Const.Y_MIN) / (Const.Y_MAX_RUNWAY - Const.Y_MIN_RUNWAY)
    y_max = 2 * ((Const.BINS_Y - 0.5) * Const.BIN_SIZE_Y + Const.Y_MIN) / (Const.Y_MAX_RUNWAY - Const.Y_MIN_RUNWAY)
    
    print y_min
    print y_max
    plt.ylim((y_min, y_max))
    plt.ylabel(r'$2y / w$')
    plt.title("Averaged Optimal Utility Map " + r'$\overline{U^{*}}(t,y)$')
    
    plt.savefig(vopt_plot_filename)