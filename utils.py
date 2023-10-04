import numpy as np

def calc(h):
    theta1 = np.arccos((h**2 - 44)/(20*h))
    theta2 = np.arccos((h**2 + 44)/(24*h))
    theta3 = theta1 + theta2

    L1 = 10
    L2 = 3
    L3 = 10
    L4 = 4

    A = (L3**2 - L1**2 - L2**2 - L4**2)/(-2*L2*L4)
    B = L1/L4
    C = L1/L2

    a = A - B*np.cos(theta3) + np.cos(theta3) - C
    b = 2*np.sin(theta3)
    c = A - B*np.cos(theta3) - np.cos(theta3) + C

    theta4 = 2*np.arctan((-b + np.sqrt(b**2 - 4*a*c))/(2*a))
    
    theta1=theta1*180/np.pi
    theta4=theta4*180/np.pi

    return (theta1, theta4)

