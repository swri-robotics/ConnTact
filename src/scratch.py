

import numpy as np
import tf.transformations as tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

print("pbutt")

def qToEu(a):
    return np.degrees(tf.euler_from_quaternion(a))

def euToQ(a):
    return tf.quaternion_from_euler(*np.radians(a))

q = euToQ([45,0,0])

vect = [0,.25,0]

lead_maximums = [.10,20] #m, degrees

commandList = [vect,list(q)]

def qGetMagnitude(a):
    return np.degrees(np.arccos(a[3])*2)

def interpCommandByMagnitude(vec):
    # Get magnitude of move and rotation
    trans_mag   = np.linalg.norm(vec[0])
    rot_mag     = qGetMagnitude(vec[1])
    new_trans = vec[0]
    new_rot = vec[1]
    changed = False
    # Clip magnitude
    if trans_mag > lead_maximums[0]:
        new_trans = (lead_maximums[0] / trans_mag) * np.array(vec[0])
        trans_mag = np.linalg.norm(new_trans)
        changed = True

    if rot_mag > lead_maximums[1]:
        new_rot = quat_lerp(np.array([0., 0., 0., 1.]), vec[1], (lead_maximums[1] / rot_mag))
        rot_mag = qGetMagnitude(new_rot)
        changed = True

    if changed:
        return [new_trans,new_rot],[trans_mag,rot_mag], True
    return vec, [trans_mag,rot_mag], True

def applyRot(quat, points):
    """
    In: Quaternion and list of 3d points
    Return: list of array(3) of rotated points.
    """
    outList = [rotate(quat, [x,y,z]) for (x,y,z) in points.T]

    return np.array(outList).T

def rotate(quat, list3):
    vector4 = np.array([*list3,0])
    return list(tf.quaternion_matrix(quat) @ vector4)[:3]

def printTrace(fromV, toV):
    """
    Return a list of points going from the tip of From to To unit vectors
    """
    interps = np.linspace(0,1,10)
    outPoints = [rotate(quat_lerp(fromV, toV, i), [1,0,0]) for i in interps]
    return np.array(outPoints).T

def getColorTuple(factor):
    """
    Get a point on the color wheel with the three colors out of phase and rotating.
    """
    tau = np.pi * 2
    phase = tau * factor
    values = [np.sin(phase + tau*0/3) * .5 + .5,
                np.sin(phase + tau*1/3) * .5 + .5,
                np.sin(phase + tau*2/3) * .5 + .5]
    return (tuple(values), tuple(np.array(values)*.65))

def quat_lerp(q1, q2, factor):
    # return tf.quaternion_slerp(q1, q2, factor)
    euOne = np.array(qToEu(q1))
    euTwo = np.array(qToEu(q2))
    resultEu = factor * euTwo + (1-factor) * euOne
    return euToQ(resultEu)

quat_battery = [euToQ(i) for i in [[0,0,90],
                                    # [0,0,-90],
                                    # [0,90,0],
                                    # [0,-90,0],
                                    # [90,0,0],
                                    # [-90,0,0],
                                    [135,135,180],
                                    # [90,0,90],
                                    # [0,90,90]
                                   ]]
quat_battery *= 6

quat_iter = quat_battery.__iter__()

# nextInd = -1
# def getQuatBattery():
#     nextInd += 1
#     if nextInd > len(quat_battery):
#         nextInd = -1
#     return quat_battery[nextInd]

def checkTest(num):
    trials = num
    successes = 0
    fig = plt.figure(figsize=(4,4))
    ax=fig.add_subplot(111, projection="3d" )
    t = np.linspace(1,0,50)
    # xs = [min(abs(time-.125)*(8/7), 7/8) for time in t]
    # xs.reverse()

    xs = [1-abs(7/8 - time)*(8/7) for time in t]

    ys = np.zeros(50)
    zs = [max(time-7/8, 0) for time in t]
    # xs = t
    # ys = t
    # zs = t
    points = np.array([xs, ys, zs])
    ax.plot(*points, color="red")
    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-1.1, 1.1)
    ax.set_zlim(-1.1, 1.1)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    while num:
        num -= 1
        testVec = tf.random_vector(3)
        # testQuat = quat_iter.__next__()
        testQuat = tf.random_quaternion()
        testVec = [np.array(testVec), np.array(testQuat)]
        startMagnitudes = [np.linalg.norm(testVec[0]),
                           qGetMagnitude(testVec[1])]
        newVec, mags, changed = interpCommandByMagnitude(testVec)
        if(changed):
            print("---\n Output for command \nIn:  {}:\nOut: {} \nstart magnitudes: {} \nfinal magnitudes: {}\n".format(testVec, newVec, startMagnitudes, mags))
            color, darkerColor = getColorTuple(num/trials)
            ax.plot(*applyRot(testQuat, points), color=color)
            ax.plot(*applyRot(newVec[1], points), color=darkerColor)
            ax.plot(*printTrace(newVec[1], testQuat), color=color)

        else:
            print("---\n For command\n {}:\nNo change. Start magnitudes \n{}\n ".format(testVec, startMagnitudes))

        if np.linalg.norm(newVec[0]) <= lead_maximums[0] and qGetMagnitude(newVec[1]) <= lead_maximums[1]:
            successes += 1
    print("Successes/trials: {}/{}".format(successes, trials))
    plt.show()

checkTest(20)

print("pdone")