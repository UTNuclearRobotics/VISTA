import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseArray, Pose
from scipy.spatial.transform import Rotation




def generate_helix(
    r_min: float,
    radius_multiplier: float,
    num_shells: int,
    helix_height: float,
    clearance: float,
    psi_max_deg: float,
    delta_theta_deg: float,
    mount_angle_deg: float,
) -> PoseArray:
    """
    Generate candidate viewpoint poses on a helical manifold around an ROI center.

    For each cylindrical shell at radius r_i, a helix is traced from the top
    (clearance + helix_height) downward using a linear pitch z = (h_max/2pi)*theta.
    Poses represent the desired sonar_optical frame in the ROI-local frame,
    with heading tangent to the helix and the FLS mount angle applied.

    Args:
        r_min:             Minimum standoff radius in metres (= Dubins turn radius)
        radius_multiplier: Max radius = radius_multiplier * r_min
        num_shells:        Number of concentric cylindrical shells
        helix_height:      Vertical extent of helix above clearance in metres
        clearance:         Minimum height above seabed in metres
        psi_max_deg:       Maximum vehicle climb angle in degrees
        delta_theta_deg:   Angular step between samples in degrees
        mount_angle_deg:   FLS downward mount angle from horizontal in degrees

    Returns:
        PoseArray with header.frame_id='roi_frame', one Pose per sample point
    """
    
    # necessary deg 2 rad conversions
    delta_theta = np.radians(delta_theta_deg)
    mount_angle = np.radians(mount_angle_deg)
    psi_max = np.radians(psi_max_deg)
    
    
    
    
    r_max = r_min * radius_multiplier
    radii = np.linspace(r_min, r_max, num=num_shells, endpoint=True, dtype=float)

    pose_array = PoseArray()
    pose_array.header.frame_id = 'roi_frame'
    
    # note that the theta list (linspace) depends on the pitch, which depends on r
    # as the radius increases, the total theta decreases. 
    # for constant theta increment, this decreasing the num samples on each helix.
    # essentially:
    # radius -> changes pitch -> changes total theta -> changes num samples on each helix for const del_theta
    
    for r in radii:
        # compute maximum allowable pitch h (height climb for one revolution spiral)
        h_max = 2*np.pi * r * np.tan(psi_max)
        # compute total theta over all spirals.
        theta_total = helix_height * (2*np.pi)/(h_max)
        num_samples = int(np.ceil(theta_total/delta_theta))
        
        #compute theta linspace
        
        theta_vec = np.linspace(0, theta_total, num = num_samples, endpoint= True, dtype = float)
        
        # now create starting pose, which we refer to as having R_0 and t_0
        # to show parameterization, we will index theta_vec
        
        t_0= np.array([r*np.cos(theta_vec[0]), r*np.sin(theta_vec[0]), helix_height + clearance -(h_max/(2*np.pi) * theta_vec[0])])
        R_0= Rotation.from_euler('z',theta_vec[0])
        
        
        
        
        
        
        
        
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    




