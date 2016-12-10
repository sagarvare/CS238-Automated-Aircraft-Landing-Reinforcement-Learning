
from const import Const
import numpy as np
import copy, math, time, collections, random
import simulator, pickle

def epsilon_greedy(w,state,possible_actions,num_iter,eps):
    """
    Defines an exploratory strategy

    @params:
    state: current state
    possible_actions : the list of all possible actions that can be done at that point
    num_iter: the number of iterations done, to reduce the exploration accordinly

    Returns an action based upon the exploratory strategy
    """
    # Explore with some probability
    if eps > np.random.uniform(0,1):
      idx = np.random.randint(len(possible_actions))
      return possible_actions[idx]
    # Otherwise, compute best action
    else: 
      best_action = None
      max_q = None 
      random.shuffle(possible_actions)
      for action in possible_actions:
        feature_inds = feature_extractor(state, action)
        q = compute_Q(w, feature_inds)
        # print "for action", action, "q value is", q
        if q > max_q:
          # print "found a better q val: ", q
          # print "with action ", action
          max_q = q
          best_action = action
      # print "Best action: ", best_action
      return best_action

def feature_extractor(state,action):
    """
    Takes in (state,action) 
    Returns a list of keys 
    """
    features = []
    t, y, v_y, v_w = state

    ##The amount of coarseness for each of the features
    bin_y = 2
    bin_vy = 4
    bin_action = 2
    bin_theta = 5


    # Another interesting feature 
    real_x = (float(Const.X_MAX) / Const.T_MAX) * t
    real_y = y * Const.BIN_SIZE_Y + Const.Y_MIN
    theta = int(np.arctan(real_y / (real_x+1e-6)) * (180.0 / np.pi) / bin_theta)
    
    real_vy = (v_y*Const.BIN_SIZE_VY) + Const.VY_MIN
    ##Funky feature
    real_diff = (real_y/(t/3600.0)) - real_vy
    funk_feat = (np.arctan(real_diff)*(180.0/np.pi))
    #print "theta: ", theta

    ##the modified states
    y = int(float(y)/bin_y)
    action = float(action)/bin_action
    v_y = int(float(v_y)/bin_vy) 

    ##rewriting state
    state = [t, y, v_y,v_w] 
    
    ##Funky feature

    features.append(("v_y",v_y,action))
    features.append(("t",t,action))
    features.append(("y",y,action))
    features.append(("Wind",v_w,action))
    features.append(("funk_feat",funk_feat,action))
    features.append(("theta",theta,action))
    
    #features.append((""))
    # for idx,val in enumerate(state):
      # features.append((idx,val,action))

    # features.append(("theta",theta,action))
    return features

def compute_Q(w, features):
  q_sa = 0.0
  for key in features:
    q_sa += w[key]
  return q_sa

def q_learning(w, gam, iter, s_0, num_iter,eps):
  alpha = .01/(num_iter**(0.33))
  # eps = 1/(num_iter**(0.33))
  s = s_0 
  # print "state is:", s
  while not sim.end_state_flag:
    # print "State is: ", s
    possible_actions = sim.get_action_list_motion_y()
    
    # check if we've reached an end state, if so, terminate
    if possible_actions == None:
      # print "breaking because we reached an end state"
      break

    # get relevant variables
    a = epsilon_greedy(w,s, possible_actions, num_iter,eps)
    # print "\taction taken: ", a
    next_s, r = sim.controller_motion_y(a)

    # Compute Q(s,a)
    feature_inds = feature_extractor(s, a)
    q_sa = compute_Q(w, feature_inds)

    # Compute max_a Q(s', a)
    max_next_q = None
    possible_next_actions = sim.get_action_list_motion_y()
    # check if we've reached an end state, if so, terminate
    if possible_next_actions == None:
      # print "breaking because we reached an end state"
      break
    # random.shuffle(possible_actions)
    next_possible_actions = sim.get_action_list_motion_y()

    if next_possible_actions == None:
      # print "breaking because we reached an end state"
      break

    for next_action in next_possible_actions:
      next_feature_inds = feature_extractor(next_s, next_action)
      next_q = compute_Q(w, next_feature_inds)
      if next_q > max_next_q:
        # print "next_q update: ", next_q
        # print "\tcorresponding action:", next_action
        max_next_q = next_q

    # Update weights vector for relevant features
    delta = r + gam * max_next_q - q_sa
    # print "feature_inds length: ", len(feature_inds)
    for f in feature_inds:
      # print "feature being updated: ", f
      # print "\t with val", alpha * delta
      w[f] += alpha * delta
    s = next_s
    # print "\tnew state is:", s



if __name__ == '__main__':

  # parameters
  maxIters = 15002
  discount = 1
  minTime = 1000
  warmStart_FLAG = False
  eps = 0.10

  t_start = 10
  file_name = "NEW_weights_found_time_t=" + str(t_start) + ".txt"

  y_start = int(Const.BINS_Y/2.5)
  const1 = 1.7 ## Number between 0 and 2
  vy_start = int((Const.BINS_VY/2.0)*const1)
  vw_start = Const.BINS_VW
  # print "starting wind:", vw_start
  state = [t_start, y_start, vy_start, vw_start]

  landing_counter = 0
  crash_counter = 0
  outsideRadar_counter = 0
  if warmStart_FLAG:
    # print "Reading weights from the file"
    with open(file_name, 'rb') as handle:
      w = pickle.loads(handle.read())
    # print "Done reading the weights_found"
  else: 
    # start with empty weights vector
    # print "Initializing a new dictionary"
    w = collections.defaultdict(float)
  # run Q learning!!
  for num_iter in xrange(1,maxIters):
    state[-1] = random.choice([12,38])
    # print "Iteration #", num_iter
    # Start new simulation
    sim = simulator.AirplaneSimulator(dim = 1, init_discrete_state = state)

    startTime = time.time()
    q_learning(w, discount, num_iter, state, num_iter,eps)

    # Check status of plane
    final_state_analysis = sim.plane_state_analysis(sim.state)
    if sim.state[0] < minTime:
        minTime = sim.state[0]
    if final_state_analysis[2] == True:
        # print "Landed safely"
      landing_counter += 1
    if final_state_analysis[1] == True:
      crash_counter += 1
    if final_state_analysis[0] == True:
      # print "OUTSIDE RADAR"
      outsideRadar_counter += 1
    # if final_state_analysis[0] and final_state_analysis[1]:
      # print "Outside RADAR AND CRASHED"
    # elif final_state_analysis[0] == True:
        # print "Outside radar"
    # elif final_state_analysis[1] == True:
        # print "Crash"
    # elif final_state_analysis[3] == True:
        # print "missed the landing!!"
    # else:
        # print "WTF!!!!!!!!!!!!!!!!"
    # print sim.state

    # Record weights 
    if num_iter % 50 == 0: ##Write after every 50 iterations!
      print "Iteration #", num_iter
      print "Landed safely ", landing_counter, "times"
      print "Crashed ", crash_counter, "times"
      print "outside radar ", outsideRadar_counter, "times"

      landing_counter = 0
      crash_counter = 0
      outsideRadar_counter = 0
      # print "Writing the weights to file!"
      with open(file_name, 'wb') as handle:
        pickle.dump(w, handle)  
      warmStart_FLAG = False
      w_norm = 0
      for key in w:
        w_norm += (w[key]**2.0)
      print "Norm of w: ", w_norm

    # Report time
    # print "It took about",(time.time() - startTime)
    
  print "Done with training part of Q-learning"
  print "Running the TEST PART of the Q-learning"
for _ in xrange(2):
  maxIters = 502
  eps = 0.10
  sum_land = 0
  landing_counter = 0
  crash_counter = 0
  outsideRadar_counter = 0
  for num_iter in xrange(1,maxIters):
    state[-1] = random.choice([12,38])
    # print "Iteration #", num_iter
    # Start new simulation
    sim = simulator.AirplaneSimulator(dim = 1, init_discrete_state = state)
    startTime = time.time()
    q_learning(w, discount, num_iter, state, num_iter,eps)
    # Check status of plane
    final_state_analysis = sim.plane_state_analysis(sim.state)
    if sim.state[0] < minTime:
        minTime = sim.state[0]
    if final_state_analysis[2] == True:
        # print "Landed safely"
      landing_counter += 1
    if final_state_analysis[1] == True:
      crash_counter += 1
    if final_state_analysis[0] == True:
      # print "OUTSIDE RADAR"
      outsideRadar_counter += 1
    # Record weights 
    if num_iter%50 == 0:
      print "Iteration #", num_iter
      print "Landed safely ", landing_counter, "times"
      print "Crashed ", crash_counter, "times"
      print "outside radar ", outsideRadar_counter, "times"
      sum_land += landing_counter
      landing_counter = 0
      crash_counter = 0
      outsideRadar_counter = 0
  print "Final Average is:", (sum_land)/(maxIters + 0.0)
      
