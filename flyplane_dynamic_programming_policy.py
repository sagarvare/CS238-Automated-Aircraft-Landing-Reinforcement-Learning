from const import Const
from simulator import AirplaneSimulator
import math, random
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


if __name__ == '__main__':
    
    # Set the value of max_t
    max_t = 50.0
    
    # Initialize file name of figure file
    trajectory_plot_filename = "FLIGHT_TRAJECTORIES_2.pdf"
    plt.figure()
    
    # Set the number of episodes here
    episodes = 100
    
    # Loop over episodes
    for epi in range(episodes):
        
        # Print episode number
        print "Episode number : ", epi
        
        # Generate a random initial discrete state with t = max_t
        t = max_t
        # y = random.choice([i for i in range(Const.BINS_Y)])
        # vy = random.choice([i for i in range(Const.BINS_VY)])
        # vw = random.choice([i for i in range(Const.BINS_VW)])
        y = random.choice([i for i in range(12,38)])
        vy = random.choice([i for i in range(12,38)])
        vw = 0
        
        # Initialize the simulator from discrete state
        # Initialize variables t_list and y_list
        sim = AirplaneSimulator(dim = 1, init_discrete_state = [t, y, vy, vw])
        t_list = []
        y_list = []
        
        # Randomize the initial state
        sim.randomize_state_motion_y()
        print "Start state = ", [t, y, vy, vw]
        
        # Follow the episode
        while sim.is_end_state(sim.state) == False:
            
            # Store the trajectory into variables t_list and y_list
            t_list.append(sim.state[0])
            y_list.append(2 * sim.state[1] / (Const.Y_MAX_RUNWAY - Const.Y_MIN_RUNWAY))
            
            # Load the pi_opt file and reshape it
            filename_piopt = "Dynamic_programming_piopt_t=" +str(int(t)) + ".txt"
            pi_opt = np.loadtxt(filename_piopt)
            pi_opt = pi_opt.reshape([Const.BINS_Y, Const.BINS_VY, Const.BINS_VW])
            
            # Get the policy
            state = (sim.discrete_state[1], sim.discrete_state[3], sim.discrete_state[5])
            action = pi_opt[(state)]
            
            # Execute the action
            next_state, reward = sim.controller_motion_y(action)
            
            # Update t
            t = next_state[0]
            
        # Plot the episode in current figure
        line = plt.plot(t_list, y_list)
        plt.setp(line, linewidth = 0.5, linestyle = '--')
        
    # Adjust figure axes, labels etc
    plt.xlim((max_t, 1))
    plt.xlabel("Time " + r'$t$' " (seconds)")
    
    y_min = 2 * Const.Y_MIN / (Const.Y_MAX_RUNWAY - Const.Y_MIN_RUNWAY)
    y_max = 2 * Const.Y_MAX / (Const.Y_MAX_RUNWAY - Const.Y_MIN_RUNWAY)
    
    plt.ylim((y_min, y_max))
    plt.ylabel(r'$2y / w$')
    plt.title("Flight trajectories")
    
    # Save the figure
    plt.savefig(trajectory_plot_filename)