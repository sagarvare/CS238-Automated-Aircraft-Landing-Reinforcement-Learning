from const import Const
import math, random

class AirplaneSimulator:
    '''
    This is the airplane simulator class
    '''
    def __init__(self, dim = 2, init_discrete_state = []):
        '''
        This is the constructor function.
        The bounds, bin size and number of bins parameters are initialized for state and action.
        The initial continuous state, discrete state and penalty calculator are initialized.
        The file to write the log is also opened
        '''
        
        self.bin_sizes_states = self.get_state_bin_sizes()
        self.bin_sizes_actions = self.get_action_bin_sizes()
        
        self.min_bounds_states, self.max_bounds_states = self.get_state_bounds()
        self.min_bounds_actions, self.max_bounds_actions = self.get_action_bounds()
        
        self.total_bins_states = self.get_state_total_bins()
        self.total_bins_actions = self.get_action_total_bins()
        
        if dim == 2:
            if init_discrete_state == []:
                self.state = self.create_initial_state()
                self.discrete_state = self.get_discrete_state(self.state)
                self.end_state_flag = self.is_end_state(self.state)
            else:
                self.discrete_state = init_discrete_state
                self.state = self.get_continuous_state(self.discrete_state)
                self.end_state_flag = self.is_end_state(self.state)
                
        elif dim == 1:
            self.state = self.create_initial_state_motion_y(init_discrete_state)
            self.discrete_state = self.get_discrete_state(self.state)
            self.end_state_flag = self.is_end_state(self.state)
    
    def create_initial_state(self):
        '''
        Method to initialize the starting state
        '''        
        # Generate wind directions randomly
        if random.uniform(0, 1) < 0.5:
            f = -1
        else:
            f = 1
        return [Const.START_T , Const.START_Y, Const.START_Z, \
                Const.START_VY, Const.START_VZ, f * Const.START_VW]
    
    def get_state_bin_sizes(self):
        '''
        Method to get the bin sizes for the state variables
        ''' 
        return [Const.BIN_SIZE_T, Const.BIN_SIZE_Y, Const.BIN_SIZE_Z, \
                Const.BIN_SIZE_VY, Const.BIN_SIZE_VZ, Const.BIN_SIZE_VW]
        
    def get_action_bin_sizes(self):
        '''
        Method to get the bin sizes for the actions
        ''' 
        return [Const.BIN_SIZE_DELTA_VY, Const.BIN_SIZE_DELTA_VZ]
        
    def get_state_bounds(self):
        '''
        Method to get the min/max bounds for the state variables
        ''' 
        state_min_bound = [Const.T_MIN, Const.Y_MIN, Const.Z_MIN, \
                           Const.VY_MIN, Const.VZ_MIN, Const.VW_MIN]
        state_max_bound = [Const.T_MAX, Const.Y_MAX, Const.Z_MAX, \
                           Const.VY_MAX, Const.VZ_MAX, Const.VW_MAX]
        return state_min_bound, state_max_bound
        
    def get_action_bounds(self):
        '''
        Method to get the min/max bounds for the actions
        ''' 
        action_min_bound = [Const.DELTA_VY_MIN, Const.DELTA_VZ_MIN]
        action_max_bound = [Const.DELTA_VY_MAX, Const.DELTA_VZ_MAX]
        return action_min_bound, action_max_bound
        
    def get_state_total_bins(self):
        '''
        Method to get the total number of bins for the state variables
        ''' 
        return [Const.BINS_T, Const.BINS_Y, Const.BINS_Z, \
                Const.BINS_VY, Const.BINS_VZ, Const.BINS_VW]
        
    def get_action_total_bins(self):
        '''
        Method to get the total number of bins for the actions
        ''' 
        return [Const.BINS_DELTA_VY, Const.BINS_DELTA_VZ]
        
    def is_end_state(self, state):
        '''
        Method to detect if a state is an end state
        State is an end state if any of the following is True:
            - t <= 0
            - plane is outside radar
            - plane has crashed
        Return True if state is an end state, else return False
        '''
        # Name elements in state variables for readability
        t = state[0]
        
        # Check if t <= T_MIN (< should never happen)
        if t <= Const.T_MIN:
            return True
        # Otherwise check for plane outside radar & plane crash
        else:
            plane_outside_radar, plane_crash, plane_land, plane_missed_landing\
                = self.plane_state_analysis(state)
            return plane_outside_radar or plane_crash
            
    def plane_state_analysis(self, state):
        '''
        Method that takes a state and detects the following:
        
        A. Plane outside radar or not.
        Plane is outside radar if the following has happened:
            - lateral position is outside [Y_MIN, Y_MAX]
            - vertical position is > Z_MAX
            - plane_outside_radar = True if above holds, False otherwise
            
        B. Plane has crashed or not.
        Plane has crashed if the following has happened:
            - If t > T_MIN : plane crashed if z <= Z_MIN
            - If t = T_MIN : plane crashed if z <= Z_LAND_TOL, v_z <= 0.0 and one of following is true:
                - v_y > VY_LAND_TOL_MAX or v_y < VY_LAND_TOL_MIN
                - v_z < VZ_LAND_TOL_MIN
                - y > Y_MAX_RUNWAY or y < Y_MIN_RUNWAY
            - plane_crash = True if crash happened, False otherwise
        
        C. Plane has landed or not safely.
        Plane has landed safely if t = T_MIN and the following has happened:
            - z <= Z_LAND_TOL
            - plane has not crashed
            - v_z <= 0
        
        D. Plane has missed landing or not.
        Planne has missed landing only if the following are true:
            - t = T_MIN
            - plane did not crash
            - plane did not land
        Method returns : plane_outside_radar, plane_crash, plane_land, plane_missed_landing
        '''
        t, y, z, v_y, v_z, v_w = state
        
        plane_land = False
        plane_missed_landing = False
        # Case : Plane outside radar
        # Check if y < Y_MIN or y > Y_MAX or z > Z_MAX
        if y < Const.Y_MIN or y > Const.Y_MAX or z > Const.Z_MAX:
            plane_outside_radar = True
        else:
            plane_outside_radar = False
        
        # Case : Plane crash & plane land
        # Check for crash if t > T_MIN
        if t > Const.T_MIN:
            # Check if z <= Z_MIN
            if z <= Const.Z_MIN:
                plane_crash = True
            else:
                plane_crash = False
                
        # Check for crash and land if t <= T_MIN (< should never happen)
        else:
            # Check if z <= Z_LAND_TOL and v_z <= 0.0, only then crash or land can happen
            if z <= Const.Z_LAND_TOL and v_z <= 0.0:
                plane_crash = False
                plane_land = True

                # Check if y outside runway
                # Check if v_z < VZ_LAND_TOL_MIN
                # Check if v_y < VY_LAND_TOL_MIN or v_y > VY_LAND_TOL_MAX
                if y > Const.Y_MAX_RUNWAY or y < Const.Y_MIN_RUNWAY \
                    or v_z < Const.VZ_LAND_TOL_MIN \
                    or v_y > Const.VY_LAND_TOL_MAX or v_y < Const.VY_LAND_TOL_MIN:
                    plane_crash = True
                    plane_land = False
                    
            else:
                plane_crash = False
                plane_land = False
                plane_missed_landing = True
        
        # Return the state analysis results
        return plane_outside_radar, plane_crash, plane_land, plane_missed_landing
        
    def get_discrete_state(self, state):
        ''' 
        Method to return discrete state corresponding to state
        '''
        # Evaluate discrete state
        discrete_state = [int(math.floor((state_var - self.min_bounds_states[i]) \
                         / self.bin_sizes_states[i])) for i, state_var in enumerate(state)]
        
        # Check if any state index < 0, if yes, set to 0
        for i, state_var in enumerate(discrete_state):
            if state_var < 0: discrete_state[i] = 0
            
        # Check if any state index >= total bins for the state, set to total bins - 1
        for i, state_var in enumerate(discrete_state):
            max_bin = self.total_bins_states[i]
            if state_var >= max_bin: discrete_state[i] = max_bin - 1
        
        return discrete_state
        
    def get_continuous_state(self, discrete_state):
        ''' 
        Method to return continuous state corresponding to a discrete state
        Assume that discrete_state is a valid discrete state
        '''
        state = []
        state.append(float(discrete_state[0]))
        for i in range(1, len(discrete_state)):
            state.append((discrete_state[i] + 0.5) * self.bin_sizes_states[i] \
                         + self.min_bounds_states[i])        
        return state
        
    def get_discrete_action(self, action):
        ''' 
        Method to return discrete action corresponding to action
        '''
        # Evaluate discrete action
        discrete_action = [int(math.floor((actions - self.min_bounds_actions[i]) \
                         / self.bin_sizes_actions[i])) for i, actions in enumerate(action)]
        
        # Check if any action index < 0, if yes, set to 0
        for i, actions in enumerate(discrete_action):
            if actions < 0: discrete_action[i] = 0
        
        # Check if any action index >= total bins for the action, set to total bins - 1
        for i, actions in enumerate(discrete_action):
            max_bin = self.total_bins_actions[i]
            if actions >= max_bin: discrete_action[i] = max_bin - 1
        
        return discrete_action
        
    def get_continuous_action(self, discrete_action):
        ''' 
        Method to return continuous action corresponding to a discrete action
        Assume that discrete_action is a valid discrete action
        '''
        action = []
        for i in range(len(discrete_action)):
            action.append((discrete_action[i] + 0.5) * self.bin_sizes_actions[i] \
                         + self.min_bounds_actions[i])        
        return action
        
    def get_action_list(self):
        ''' 
        Method to return get discrete action list from the current state of simulator.
        If simulator is in an end state, return None
        If simulator is not in an end state, return all possible actions
        '''
        if self.is_end_state(self.state) == True:
            return None
        else:
            discrete_action_list = [(a1, a2) for a1 in range(self.total_bins_actions[0])\
                                    for a2 in range(self.total_bins_actions[1])]
            return discrete_action_list
            
    def update_state(self, action):
        '''
        Method to update the current simulator state according to a given action
        '''
        # Only carry out update if self.state is not an end state
        if self.is_end_state(self.state) == False:
            
            # Name elements in state variables for readability
            t, y, z, v_y, v_z, v_w = self.state
            
            # Name elements in action variable for readability
            delta_vy, delta_vz = action[0], action[1]
            
            # Update state variables
            next_t = t - Const.BIN_SIZE_T
            next_y = y + v_y * Const.BIN_SIZE_T / 3600.0
            
            next_z = z + v_z * Const.BIN_SIZE_T / 3600.0
            wind_effect = 0.01 * (v_w**2) * self.sign_real(v_w)
            next_v_y = v_y + (delta_vy + wind_effect) * Const.BIN_SIZE_T
            
            next_v_z = v_z + delta_vz * Const.BIN_SIZE_T
            next_v_w_mean = v_w 
            next_v_w = random.normalvariate(next_v_w_mean, Const.VW_SIGMA)
            
            # Bound state variables for v_w to avoid very high values
            self.snap_to_bounds(next_v_w, Const.VW_MIN, Const.VW_MAX)
            
            # Update state
            self.state = [next_t, next_y, next_z, next_v_y, next_v_z, next_v_w]
            self.discrete_state = self.get_discrete_state(self.state)
            self.end_state_flag = self.is_end_state(self.state)
            
            # Record to log file
            # self.record_state()
            
    def randomize_state(self):
        '''
        Method to randomize the continuous state, without altering the discrete state
        Do not randomize t
        For rest of state variables, generate state randomly from a uniform distribution
        '''
        # First get the continuous state mean that corresponds to the discrete state
        continuous_state_mean = self.get_continuous_state(self.discrete_state)
        
        # Get the lower and upper bounds for each variable
        lower_bound = [mean - self.bin_sizes_states[i] / 2.0 \
                       for i, mean in enumerate(continuous_state_mean)]
        upper_bound = [mean + self.bin_sizes_states[i] / 2.0 \
                       for i, mean in enumerate(continuous_state_mean)]
        
        # Generate random number uniformly
        new_state = [random.uniform(lower_bound[i], upper_bound[i]) \
                     for i in range(len(continuous_state_mean))]
        
        # Reset the time
        new_state[0] = continuous_state_mean[0]
        
        # Update self.state
        self.state = new_state
        
    # def record_state(self):
    #     '''
    #     Method to keeps a log of the states (continuous) visited in simulation
    #     '''
    #     with open('states_visited.txt', 'a+') as f:
    #         output_data = [str(value) for value in self.state]
    #         f.write('\t'.join(output_data) + '\n')
    
    def snap_to_bounds(self, value, bound_min, bound_max):
        '''
        Method to check value, bound_min, bound_max to see if value is in the 
        range [bound_min bound_max] and clips value to within that range if needed.
        '''
        if value > bound_max:
            value = bound_max
            return bound_max
        elif value < bound_min:
            value = bound_min
            return bound_min

    def sign_real(self, number):
        '''
        Method that takes a real number and returns its sign:
        return 1,  if number > 0; return -1, if number < 0; return 0, if number = 0
        '''
        if number == 0.0: return 0
        elif number > 0.0: return 1
        else: return -1        
        
    def get_reward(self, state, next_state):
        '''
        Method that takes as input continuous states : state(s), next_state(s')
        Returns the reward for (s,s')
        '''
        # Name elements in state variables for readability
        t, y, z, v_y, v_z, v_w = state
        next_t, next_y, next_z, next_v_y, next_v_z, next_v_w = next_state
        
        # Check if initial state is crash / outside radar / missed landing
        # If yes, return None
        plane_outside_radar, plane_crash, plane_land, plane_missed_landing \
            = self.plane_state_analysis(state)
            
        if plane_outside_radar == True or plane_crash == True \
            or plane_missed_landing == True or plane_land == True:
            print "Control never reaches here - plane already outside radar / crashed / landed / missed landing."
            return None
        
        def penalty_dv(vy1, vy2, vz1, vz2):
            '''
            Method to calculate penalty due to change in velocity
            '''
            # Calculate magnitude of change in velocity
            dv = math.sqrt((vy1 - vy2)**2.0 + (vz1 - vz2)**2.0)
            return Const.PENALTY_DV * math.pow(dv, 2)
        
        p_dv = penalty_dv(v_y, next_v_y, v_z, next_v_z)
        
        # Check if next state is crash / outside radar / missed landing
        # If yes, return high penalty
        plane_outside_radar, plane_crash, plane_land, plane_missed_landing \
            = self.plane_state_analysis(next_state)
        if plane_outside_radar == True:
            p_outside_radar = Const.PENALTY_OUTSIDE_RADAR
        else:
            p_outside_radar = 0.0

        if plane_crash == True:
            p_crash = Const.PENALTY_CRASH
        else:
            p_crash = 0.0

        if plane_missed_landing == True:
            p_missed_landing = Const.PENALTY_MISSED_LANDING
        else:
            p_missed_landing = 0.0

        if plane_outside_radar or plane_crash or plane_missed_landing:
            return p_dv + p_outside_radar + p_crash + p_missed_landing
        
        # Check if next_t != T_MIN - Not ended
        if next_t > Const.T_MIN:
            return p_dv
        # Otherwise need to check for landing
        else:
            # If landing missed
            if plane_land == False:
                print "Control should not reach here...plane should have safely landed."
                return None
            # If landing has happened
            else:
                return p_dv + Const.PENALTY_RUNWAY * math.pow(next_y, 2) + Const.LANDING_REWARD
        
    def controller(self, discrete_action):
        '''
        Method that takes as input discrete action : discrete_action
        Outside entities should update the class objects through this method.
        The following steps are performed:
            - Convert discrete_action into continuous action
            - Update the state, discrete state
            - Compute the reward
            - Return new discrete state, reward
        '''
        # Convert discrete action to continuous action, and snap to bounds
        action = self.get_continuous_action(discrete_action)
        for i, actions in enumerate(action):
            self.snap_to_bounds(actions, self.min_bounds_actions[i], self.max_bounds_actions[i])
            
        # Record current state & update the state
        current_state = self.state
        self.update_state(action)

        # Compute reward
        reward = self.get_reward(current_state, self.state)
        
        # Return new discrete state and controller
        return self.discrete_state, reward
        
    ###########################################################################
    """
    For this section we assume that the vertical velocity is fixed. We always choose
    an action in the Z direction that implies delta_vz = 0.0
    The simplificatiion with this assumption is that we only need to control the
    motion in the Y direction
    """
    def create_initial_state_motion_y(self, init_discrete_state):
        '''
        Method to initialize the starting state
        init_discrete_state is a list of discrete states [t, y, vy, vw]
        '''        
        # Generate wind directions randomly
        if random.uniform(0, 1) < 0.5:
            f = -1
        else:
            f = 1
        
        if init_discrete_state == []:
            return [Const.START_T , Const.START_Y, (Const.Z_MIN + Const.BIN_SIZE_Z / 2.0), \
                    Const.START_VY, 0.0, f * Const.START_VW]
        else:
            discrete_state = [init_discrete_state[0], init_discrete_state[1], 0, \
                              init_discrete_state[2], 0, init_discrete_state[3]]
            state = self.get_continuous_state(discrete_state)
            
            return [state[0] , state[1], (Const.Z_MIN + Const.BIN_SIZE_Z / 2.0), \
                    state[3], 0.0, state[5]]
                
    def get_action_list_motion_y(self):
        ''' 
        Method to return discrete action list for only y motion.
        If simulator is in an end state, return None
        If simulator is not in an end state, return all possible actions
        '''
        if self.is_end_state(self.state) == True:
            return None
        else:
            discrete_action_list_motion_y = [a1 for a1 in range(self.total_bins_actions[0])]
            return discrete_action_list_motion_y
            
    def randomize_state_motion_y(self):
        '''
        Method to randomize the continuous state, without altering the discrete state
        Do not randomize t, z, vz
        For rest of state variables, generate state randomly from a uniform distribution
        '''
        # First get the continuous state mean that corresponds to the discrete state
        continuous_state_mean = self.get_continuous_state(self.discrete_state)
        
        # Get the lower and upper bounds for each variable
        lower_bound = [mean - self.bin_sizes_states[i] / 2.0 \
                       for i, mean in enumerate(continuous_state_mean)]
        upper_bound = [mean + self.bin_sizes_states[i] / 2.0 \
                       for i, mean in enumerate(continuous_state_mean)]
        
        # Generate random number uniformly
        new_state = [random.uniform(lower_bound[i], upper_bound[i]) \
                     for i in range(len(continuous_state_mean))]
        
        # Reset the time, z, vz
        new_state[0] = continuous_state_mean[0]
        new_state[2] = continuous_state_mean[2]
        new_state[4] = continuous_state_mean[4]
        
        # Update self.state
        self.state = new_state
        
    def controller_motion_y(self, discrete_action_motion_y):
        '''
        Method that takes as input discrete action : discrete_action_motion_y
        Outside entities should update the class objects through this method.
        The following steps are performed:
            - Convert discrete_action_motion_y into continuous action
            - Update the state, discrete state
            - Compute the reward
            - Return new discrete state, reward
        '''
        # Create discrete_action from discrete_action_motion_y
        # Z action is 0.0
        discrete_action = [discrete_action_motion_y, int((Const.BINS_DELTA_VZ - 1) / 2.0)]
        
        # Convert discrete action to continuous action, and snap to bounds
        action = self.get_continuous_action(discrete_action)
        for i, actions in enumerate(action):
            self.snap_to_bounds(actions, self.min_bounds_actions[i], self.max_bounds_actions[i])
            
        # Record current state, update the state and get next state
        current_state = self.state
        self.update_state(action)

        # Compute reward
        reward = self.get_reward(current_state, self.state)
        
        # Return new discrete state and controller
        discrete_state = [self.discrete_state[0], self.discrete_state[1], \
                          self.discrete_state[3], self.discrete_state[5]]
        return discrete_state, reward
        