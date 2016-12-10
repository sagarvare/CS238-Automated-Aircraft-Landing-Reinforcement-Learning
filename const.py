# This class stores all the global parameters for the simulation

class Const(object):
    
    # Position parameters
    Z_MAX = 1.0                                                                # in km
    Z_MIN = 0.0                                                                # in km
    BINS_Z = 100                                                               # number of bins in Z direction
    BIN_SIZE_Z = (Z_MAX - Z_MIN) / BINS_Z                                      # in km
    
    X_MAX = 5.0                                                                # in km
    
    Y_MAX = 0.125                                                              # in km
    Y_MIN = -0.125                                                             # in km
    BINS_Y = 51                                                               # number of bins in Y direction
    BIN_SIZE_Y = (Y_MAX - Y_MIN) / BINS_Y                                      # in km
    
    # Time parameters
    T_MAX = 275.0                                                              # in seconds
    T_MIN = 0.0                                                                # in seconds
    BINS_T = 276                                                               # number of bins in T
    BIN_SIZE_T = (T_MAX - T_MIN + 1) / BINS_T                                  # in seconds
    
    # Flight path and velocity parameters
    ALPHA = 2.86                                                               # in degrees
    
    VY_MIN = -50.0                                                             # in km/hr
    VY_MAX = 50.0                                                              # in km/hr
    BINS_VY = 51                                                              # number of bins in VY
    BIN_SIZE_VY = (VY_MAX - VY_MIN) / BINS_VY                                  # in km/hr
    
    VZ_MIN = -20.0                                                             # in km/hr
    VZ_MAX = 20.0                                                              # in km/hr
    BINS_VZ = 41                                                               # number of bins in VZ
    BIN_SIZE_VZ = (VZ_MAX - VZ_MIN) / BINS_VZ                                  # in km/hr
    
    
    # Mass of plane
    M = 1.0                                                                    # in kg
    
    # Runway parameters and landing parameters
    Y_MAX_RUNWAY = 0.04                                                        # in km
    Y_MIN_RUNWAY = -0.04                                                       # in km
    Z_LAND_TOL = 0.02                                                          # in km
    VY_LAND_TOL_MAX = 20.0                                                     # in km/hr
    VY_LAND_TOL_MIN = -20.0                                                    # in km/hr
    VZ_LAND_TOL_MIN = -15.0                                                    # in km/hr
    
    # Wind parameters
    VW_SIGMA = 1.0                                                             # in km/hr
    VW = 10.0                                                                  # in km/hr
    VW_MAX = 25.0                                                              # in km/hr
    VW_MIN = -25.0                                                             # in km/hr
    BINS_VW = 25                                                               # number of bins in VW
    BIN_SIZE_VW = (VW_MAX - VW_MIN) / BINS_VW                                  # in km/hr
    
    
    # Action parameters
    DELTA_VZ_MAX = 35.0                                                        # in km/hr/s
    DELTA_VZ_MIN = -35.0                                                       # in km/hr/s
    BINS_DELTA_VZ = 201                                                        # number of bins in DELTA_VZ
    BIN_SIZE_DELTA_VZ = (DELTA_VZ_MAX - DELTA_VZ_MIN) / BINS_DELTA_VZ          # in km/hr/s
    
    
    DELTA_VY_MAX = 35.0                                                        # in km/hr/s
    DELTA_VY_MIN = -35.0                                                       # in km/hr/s
    BINS_DELTA_VY = 101                                                        # number of bins in DELTA_VY
    BIN_SIZE_DELTA_VY = (DELTA_VY_MAX - DELTA_VY_MIN) / BINS_DELTA_VY          # in km/hr/s
    
    # Set the simulation starting parameters here
    START_T = T_MAX                                                            # in sec
    START_Y = 0.0                                                              # in km
    START_Z = Z_MAX                                                            # in km
    START_VY = 0.0                                                             # in km/hr
    START_VZ = -13.0                                                           # in km/hr
    START_VW = VW                                                              # in km/hr
    
    # Set the penalty parameters here
    LANDING_REWARD = 1e3
    PENALTY_CRASH = -1e3
    PENALTY_OUTSIDE_RADAR = -1e3
    PENALTY_MISSED_LANDING = -1e3
    PENALTY_DV = -1
    PENALTY_RUNWAY = -1e3
