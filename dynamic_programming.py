from const import Const
from simulator import AirplaneSimulator
import math, random, collections
import numpy as np
import time
from multiprocessing import Pool
from functools import partial

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


"""
Dynamic programming based policy search
"""
    
def rollout_evaluation_1step(action_list, nIter, next_state_vopt,t,y,vy, vw):
    """
    Assume that start_state is always among the valid discrete states
    nIter is the number of iterations we run for each action
    Also input next_state_vopt numpy array
    Take the action
    Return estimate of (reward + v_opt)
    Return pi_opt
    """
    # Initialize Qopt_average as numpy array
    Q_opt_average = np.zeros(len(action_list) , dtype = 'float') + 1e-6
    start_state = [t,y,vy,vw]
    # Loop over all actions
    for action in action_list:
        #print "correct here"

        # Initialize sum of Qopt
        Q_opt_sum = 0.0
        
        # Run the simulation nIter number of times
        for iteration in range(nIter):
            
            # Initialize the simulator from the discrete start_state
            sim = AirplaneSimulator(dim = 1, init_discrete_state = start_state)
            
            # Randomize the state
            sim.randomize_state_motion_y()
            
            # Take the action and get Qopt(s,a)
            next_state, reward = sim.controller_motion_y(action)
            
            # From next_state create loop-up key
            key = (next_state[1], next_state[2], next_state[3])
            Q_opt_sum += reward + next_state_vopt[key]
        #print "correct here"
    
        # Get average Qopt
        Q_opt_average[action] = Q_opt_sum / nIter
        
    # Extract v_opt and pi_opt
    pi_opt, v_opt = max(enumerate(Q_opt_average), key = lambda tups : tups[1])
    
    # Return pi_opt annd v_opt
    return (pi_opt, v_opt)
    
def compute_optimum_value_policy(t, next_state_vopt, nIter):
    """
    This method takes as input the current t
    Also input next_state_vopt numpy array
    The method computes current_state_vopt and current_state_pi
    """
    # Initialize the numpy arrays to return
    current_state_vopt = np.zeros([Const.BINS_Y, Const.BINS_VY, Const.BINS_VW], dtype = 'float') + 1e-6
    current_state_piopt = np.zeros([Const.BINS_Y, Const.BINS_VY, Const.BINS_VW], dtype = 'int')
    
    # Get list of actions
    action_list = [a1 for a1 in range(Const.BINS_DELTA_VY)]
    
    # Note that the number of states for y, vy, vw are giicev by:
    # y : Const.BINS_Y, vy : Const.BINS_VY, vw : Const.BINS_VW
    start_time = time.time()
    for y in range(Const.BINS_Y):
        print "y = ", y
        print "prev y took : ", time.time() - start_time
        for vy in range(Const.BINS_VY):
            startTime2 = time.time()
            ##Parallelizing this shit
            list_of_vw = range(Const.BINS_VW)
            pool = Pool(processes = 40)
            ##(action_list, nIter, next_state_vopt,y,vy, vw)
            partial_rollout = partial(rollout_evaluation_1step,action_list,nIter,next_state_vopt,t,y,vy)

            temp = pool.map(partial_rollout,list_of_vw)

            pool.terminate()
            for i,vw in enumerate(range(Const.BINS_VW)):
                current_state_piopt[(y, vy, vw)] = temp[i][0]
                current_state_vopt[(y, vy, vw)] = temp[i][1]

            #print "vy", vy, "took about:", time.time() - startTime2    
    # Return current_state_piopt, current_state_vopt
    return current_state_piopt, current_state_vopt
    
if __name__ == '__main__':
    
    warmstart_flag = False
    t_flag = 2
    # Generate next_state_vopt for t = 0
    # Note that the number of states for y, vy, vw are giicev by:
    # y : Const.BINS_Y, vy : Const.BINS_VY, vw : Const.BINS_VW
    next_state_vopt = np.zeros([Const.BINS_Y, Const.BINS_VY, Const.BINS_VW], dtype = 'float')
<<<<<<< HEAD
    #file_name_vopt = "Dynamic_programming_vopt_t=" +str(2) + ".txt"
  
    #next_state_vopt = np.loadtxt(file_name_vopt)
    #next_state_vopt = next_state_vopt.reshape([Const.BINS_Y, Const.BINS_VY, Const.BINS_VW])
    
    nIter = 5
    max_t = 50
    for t in xrange(1, max_t + 1):   
=======
    
    if warmstart_flag:
        file_name_vopt = "Dynamic_programming_vopt_t=" + str(t_flag) + ".txt"
        next_state_vopt = np.loadtxt(file_name_vopt)
        next_state_vopt = next_state_vopt.reshape([Const.BINS_Y, Const.BINS_VY, Const.BINS_VW])
    else:
        t_flag = 1

    nIter = 5
    max_t = 10
    for t in xrange(t_flag, max_t + 1):   
>>>>>>> 0afce93f4710d6d5d063ff889a72757c3b653e4a

        ##file name to write stuff too
        file_name_vopt = "Dynamic_programming_vopt_t=" + str(t) + ".txt"
        file_name_piopt = "Dynamic_programming_piopt_t=" + str(t) + ".txt"

        ##Load a file
        # next_state_vopt = np.load(file_name_vopt)
        # next_state_vopt = next_state_vopt.reshape([Const.BINS_Y, Const.BINS_VY, Const.BINS_VW])

        
        current_state_piopt, current_state_vopt = compute_optimum_value_policy(t, next_state_vopt, nIter)
        pi_opt_shape = current_state_piopt.shape 
        vopt_shape = current_state_vopt.shape

        np.savetxt(file_name_vopt, current_state_vopt.reshape([vopt_shape[0], vopt_shape[1] * vopt_shape[2]] ) )
        np.savetxt(file_name_piopt, current_state_piopt.reshape([pi_opt_shape[0], pi_opt_shape[1] * vopt_shape[2]] ) )

        Vopt = np.sum(current_state_vopt,axis = (1,2))/(Const.BINS_VY * Const.BINS_VW + 0.0)
        ##Plot the results for this 
        plt.ioff()
        plt.figure()
        plt.plot(range(Const.BINS_Y) ,Vopt, color = 'cyan')
        string = "VOPT_Y_profile_t=" + str(t) + ".jpg"
        plt.savefig(string)
        plt.close()






        next_state_vopt = current_state_vopt

    
    
        
            
            
            

