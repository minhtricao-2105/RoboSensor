import roboticstoolbox as rtb
import numpy as np

def _create_DH():

    """
    Create robot's standard DH model
    """
    # deg = np.pi / 180
    mm = 1e-3

    # kinematic parameters
    a = np.r_[0, -329, -311.5, 0, 0, 0] * mm
    d = np.r_[146, 0, 0, 129.7, 106, 113.2] * mm
    alpha = [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0]
    qlim = [[-2*np.pi, 2*np.pi] for _ in range(7)]

    # offset to have the dh from toolbox match with the actual pos
    offset = [0, -np.pi/2, 0, -np.pi/2, 0, 0]
    links = []
    for j in range(6):
        link = rtb.RevoluteDH(
            d=d[j], a=a[j], alpha=alpha[j], offset=offset[j], qlim=qlim[j])
        links.append(link)

    return links

# Main routine
if __name__ == "__main__":
    links = _create_DH()
    robot = rtb.DHRobot(links, name="MyRobot")
    q = [0, 0, 0, 0, 0, 0]
    
    figure = robot.plot(q)
    figure.hold()