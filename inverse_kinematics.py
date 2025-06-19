import numpy as np

def calculate_angles(x, y, z, l1 = 9.5, l2=13.5, l3=14.7, l4=10):
    theta_1 = np.arctan2(y, x)
    a = np.sqrt(x**2 + y**2)
    b = a - l4
    d = z - l1
    c = np.sqrt(b**2 + d**2)
    
    alpha_1 = np.arctan2(d, b)
    alpha_2 = np.arccos((l2**2 + c**2 - l3**2) / (2 * l2 * c))

    theta_2 = alpha_1 + alpha_2
    theta_3 = np.arccos((l2**2 + l3**2 - c**2) / (2 * l2 * l3))
    
    theta_1_deg = np.degrees(theta_1)
    theta_2_deg = np.degrees(theta_2)
    theta_3_deg = np.degrees(theta_3)
    
    theta_1_servo = 90 - 2 * theta_1_deg
    theta_2_servo = theta_2_deg
    theta_3_servo = theta_2_deg + theta_3_deg
    
    is_out_of_reach = theta_1_servo < 0 or theta_1_servo > 180 or theta_2_servo < 55 or theta_2_servo > 165 or theta_3_servo < 125 or theta_3_servo > 180

    return theta_1_servo, theta_2_servo, theta_3_servo, is_out_of_reach


