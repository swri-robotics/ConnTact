#! /usr/bin/env python3


import numpy as np
import tf.transformations as tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import (Point,Pose,Wrench,Quaternion)#, Pose, PoseStamped, Quaternion, Transform,
                               #TransformStamped, Vector3, Wrench,
                               #WrenchStamped)
import assembly_utils as utils

runProjectionTest = False
runInterpolationTest = True

def getColorTuple(factor):
    """
    Get a point on the color wheel with the three colors out of phase and rotating.
    """
    tau = np.pi * 2
    phase = tau * factor
    values = [np.sin(phase + tau*0/3) * .5 + .5,
              np.sin(phase + tau*1/3) * .5 + .5,
              np.sin(phase + tau*2/3) * .5 + .5]
    return (tuple(values), tuple(np.array(values)*.85))


def qToEu(a):
    return np.degrees(tf.euler_from_quaternion(a))

def euToQ(a):
    if len(a) != 3:
        print("Bad input!")
    return tf.quaternion_from_euler(*np.radians(a))

if runProjectionTest:

    # Begin projection system unit tests ______________________________
    def projection_test(ax):
        def randrange(n, vmin, vmax):
            return (vmax - vmin) * np.random.rand(n) + vmin
        plot_size = 2.1
        scatter_size = 2
        ax.set_xlim(-plot_size, plot_size)
        ax.set_ylim(-plot_size, plot_size)
        ax.set_zlim(-plot_size, plot_size)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        max_translation_start = 1
        #Plot the line at the origin
        x,y,z = np.array([orig, vect+orig]).T

        # Paint on the vector axis
        ax.plot(x,y,z, color="gray")

        xs = randrange(n, -scatter_size, scatter_size)
        ys = randrange(n, -scatter_size, scatter_size)
        zs = randrange(n, -scatter_size, scatter_size)
        # ax.scatter(xs, ys, zs, marker='x')
        paths = []
        index=0
        for row in np.array([xs, ys, zs]).T:
            #Row is the start point; find where it gets projected
            color, darkerColor = getColorTuple(index / n)
            result = mover.current_move([*row])[0]
            path = np.array([row, result]).T
            ax.plot(*path, color=color, dashes=(4, 3, 1, 3))
            # ax.scatter(*row, color=darkerColor, marker='x')
            ax.scatter(*result, color=darkerColor, marker='.')
            ax.scatter(xs[index], ys[index], zs[index], color=darkerColor, marker='x')
            index += 1

        # Add the origin point on top
        ax.scatter(*orig, color="black", marker="o")

    fig = plt.figure(figsize=(4, 4))

    orig = np.array([1, 1, 1])
    # orig = np.array([-0, 0, 0])
    vect = np.array([-1, -1, -1])
    # vect = np.array([0, 0, -11])
    n = 20 #number of points

    ax = fig.add_subplot(221, projection="3d")
    ax.set_title('"Line"')
    mover = utils.MovePolicy(utils.MoveModeLine(), origin=orig, vector=vect)
    projection_test(ax)
    ax2 = fig.add_subplot(222, projection="3d")
    ax2.set_title('"Plane"')
    mover = utils.MovePolicy(utils.MoveModePlane(), origin=orig, vector=vect)
    projection_test(ax2)
    ax3 = fig.add_subplot(223, projection="3d")
    ax3.set_title('"Set"')
    mover = utils.MovePolicy(utils.MoveModeSet(), origin=orig, vector=vect)
    projection_test(ax3)
    ax4 = fig.add_subplot(224, projection="3d")
    ax4.set_title('"Free"')
    mover = utils.MovePolicy(utils.MoveModeFree(), origin=orig, vector=vect)
    projection_test(ax4)

    plt.show()
    # end projection system unit tests ______________________________

    print("Done")


    quit()

if runInterpolationTest:
    current_pose = Pose()
    current_pose.position = Point(2,2,2)
    current_pose.orientation = Quaternion(0,0,0,1)
    com = [[3,3,None], [0,None,None]]

    q = euToQ([45,0,0])
    vect = [0,.25,0]
    lead_maximums = [.20,20] #m, degrees
    commandList = [vect,list(q)]


    def qGetMagnitude(a):
        return np.degrees(np.arccos(a[3])*2)

    i = 10
    def euGetMagnitude(euler):
        return np.linalg.norm(euler)
    while i>0:
        i-=1
        qstart = tf.random_quaternion()
        qend = tf.random_quaternion()
        eudiff = qToEu(qend) - qToEu(qstart)
        # print("Start, end, Diff are \n{},\n{},\n{},".format(qToEu(qstart),qToEu(qend),qToEu(qdiff)))
        eustart = qToEu(qstart)
        euend = qToEu(qend)
        print("Validating euler angle magnituder:{:.3f} is the mag of {}".format(euGetMagnitude(eustart), eustart))
        print("Validating: Start plus Diff should equal end:\n{} vs \n{}".format(
            np.around(eustart + eudiff, 3),
            np.around(euend, 3)
        ))


    # def interpCommandByMagnitude(curr_vec,target_vec, lead_maximum=lead_maximums):
    #     """
    #     Shorten a command's 'lead' to a given pos/rot cap to artificially restrict motion speed on a PD controller.
    #     We take in the current position and the initial target position, and return a modified target position
    #     which can't be further than the bounds specified.
    #     :param curr_vec: (list) = [[x,y,z position],[rotation in either Euler or Quaternion]]
    #     """
    #     # Keep track of whether a command change was required. If the target lead is small, there's no need.
    #     changed = False
    #
    #     # Record whether a quaternion was passed in instead of an Euler rotation:
    #     input_quaternion = len(curr_vec[1]) == 4
    #
    #     if input_quaternion: #We're getting a quaternion for orientation; change it to Euler for the process
    #         curr_vec[1] = qToEu(curr_vec[1])
    #         target_vec[1] = qToEu(target_vec[1])
    #
    #     # Get magnitude of move and rotation by taking the difference from current to target:
    #     diff_vec = [target_vec[0]-curr_vec[0],
    #                 target_vec[1]-curr_vec[1]]
    #     trans_mag   = np.linalg.norm(diff_vec[0])
    #     rot_mag     = euGetMagnitude(diff_vec[1])
    #     new_command_vec = [*target_vec]
    #
    #     # Clip magnitudes
    #     if trans_mag > lead_maximum[0]:
    #         new_command_vec[0] = (lead_maximum[0] / trans_mag) * np.array(diff_vec[0]) + curr_vec[0]
    #         trans_mag = np.linalg.norm(new_command_vec[0])
    #         changed = True
    #     if rot_mag > lead_maximum[1]:
    #         new_command_vec[1] = quat_lerp(curr_vec[1], target_vec[1], (lead_maximum[1] / rot_mag))
    #         rot_mag = euGetMagnitude(new_command_vec[1])
    #         changed = True
    #
    #     if not changed:
    #         return target_vec, [trans_mag, rot_mag], False
    #
    #     # convert command back to quaternion if we started with one:
    #     if input_quaternion:
    #         new_command_vec[1] = euToQ(new_command_vec[1])
    #     # Add the commanded lead back to
    #     if changed:
    #         return new_command_vec,[trans_mag,rot_mag], True
    #

    def applyRot(vec, points):
        """
        In: Quaternion and list of 3d points
        Return: list of array(3) of rotated points.
        """
        # quat = euToQ(list(vec[1]))
        quat = vec[1]
        outList = [rotate(quat, np.array([x,y,z])) + vec[0] for (x,y,z) in points.T]
        # outList = [rotate(quat, np.array([x,y,z]) + vec[0]) for (x,y,z) in points.T]

        return np.array(outList).T

    def rotate(quat, list3):
        """
        Rotates the vector list3 by the rotation quat
        """
        vector4 = np.array([*list3,0])
        return list(tf.quaternion_matrix(quat) @ vector4)[:3]

    def printTrace(fromV, toV, intervals = 10):
        """
        Return a list of points going from the tip of From to To unit vectors
        """
        interps = np.linspace(0,1,intervals)
        outPoints = [rotate(quat_lerp(fromV[1], toV[1], i), [1,0,0])+fromV[0] for i in interps]
        return np.array(outPoints).T


    def printTrace2(fromV, toV, intervals = None):

        # Record whether a quaternion was passed in instead of an Euler rotation:
        input_quaternion = len(fromV[1]) == 4
        assert len(fromV[1]) == len(toV[1]), "Different orientation representations sent to interp!"
        if input_quaternion:  # We're getting a quaternion for orientation; change it to Euler for the process
            fromV[1] = qToEu(fromV[1])
            toV[1] = qToEu(toV[1])

        # Get magnitude of move and rotation by taking the difference from current to target:
        diff_vec = [toV[0] - fromV[0],
                    toV[1] - fromV[1]]
        trans_mag = np.linalg.norm(diff_vec[0])
        rot_mag = euGetMagnitude(diff_vec[1])

        if intervals is None:
            # intervals = (trans_mag + rot_mag/180)*5
            intervals = 15
        interps = np.linspace(0,1,intervals)

        newVects = [utils.interpCommandByMagnitude(fromV, toV, np.array([trans_mag,rot_mag])*i) for i in interps]
        # Find the points at the end of the arrows for all these poses.
        outVects = [rotate(utils.euToQ(vec[1]), [1,0,0]) + vec[0] for vec in newVects]
        return np.array(outVects).T



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
        # euOne = (qToEu(q1))
        # euTwo = (qToEu(q2))
        # resultEu = factor * qToEu(q2) + (1-factor) * qToEu(q1)
        # return euToQ(factor * qToEu(q2) + (1-factor) * qToEu(q1))
        return (factor * q2 + (1-factor) * q1)

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

    # fig = plt.figure(figsize=(4,4))
    max_translation_start = 0
    max_translation_end = 2
    def checkTest(num, ax = None):
        trials = num
        successes = 0
        if ax is None:
            ax=fig.add_subplot(111, projection="3d" )
        t = np.linspace(1,0,50)
        # xs = [min(abs(time-.125)*(8/7), 7/8) for time in t]
        # xs.reverse()

        xs = [1-abs(7/8 - time)*(8/7) for time in t]
        ys = np.zeros(50)
        zs = [max(time-7/8, 0) for time in t]
        points = np.array([xs, ys, zs])

        plot_size = 2.1
        # ax.set_xlim(-plot_size, plot_size)
        # ax.set_ylim(-plot_size, plot_size)
        # ax.set_zlim(-plot_size, plot_size)
        # ax.set_xlim(-1.1, 1.1)
        # ax.set_ylim(-1.1, 1.1)
        # ax.set_zlim(-1.1, 1.1)
        # ax.set_xlabel("x")
        # ax.set_ylabel("y")
        # ax.set_zlabel("z")

        test_start_vec = [(np.array(tf.random_vector(3)*2 - np.array([1, 1, 1]))
                           *max_translation_start),
                          np.array(tf.random_quaternion())]
        while num:
            num -= 1
            # testQuat = quat_iter.__next__()
            testQuat = tf.random_quaternion()
            # while qGetMagnitude(testQuat) < 90:
            #     testQuat = tf.random_quaternion()
            test_target_vec = [test_start_vec[0]+np.array(tf.random_vector(3)*2 - np.array([1, 1, 1]))*max_translation_end,
                               np.array( tf.quaternion_multiply(testQuat, test_start_vec[1] ) )]
            newCommandVec = utils.interpCommandByMagnitude([*test_start_vec],[*test_target_vec],lead_maximums)

            color, darkerColor = getColorTuple(num/(trials*2))
            # Plot target vec
            ax.plot(*applyRot(test_target_vec, points), color=darkerColor)
            # color, darkerColor = getColorTuple(num+1/(trials*2))
            # ax.scatter(*printTrace(test_start_vec, test_target_vec), color=color)
            # Plot a trace from start to finish arrow tip position
            ax.scatter(*printTrace2([*test_start_vec], test_target_vec), color=color)
            # Plot the line between the two points
            xs, ys, zs =np.array([[*test_start_vec][0], test_target_vec[0]]).T
            ax.plot(xs, ys, zs , color=color, dashes=(4, 3, 1, 3))
            ax.plot(*applyRot(newCommandVec, points), color=darkerColor)

            if np.linalg.norm(newCommandVec[0]) <= lead_maximums[0] and qGetMagnitude(newCommandVec[1]) <= lead_maximums[1]:
                successes += 1
                print("Trim succeeded!")
            else:
                print("Clipping failed! Move magnitude: {} Turn magnitude: {}".format(np.linalg.norm(newCommandVec[0]), qGetMagnitude(newCommandVec[1])))
        ax.plot(*applyRot([*test_start_vec], points), color="black")

        print("Successes/trials: {}/{}".format(successes, trials))



    fig = plt.figure(figsize=(4, 4))

    orig = np.array([1, 1, 1])
    vect = np.array([-1, -1, -1])

    # checkTest(3)
    # plt.show()
    # fig.clear()

    ax = fig.add_subplot(221, projection="3d")
    # ax.set_title('"Line"')
    checkTest(1, ax)
    # projection_test(ax)
    ax2 = fig.add_subplot(222, projection="3d")
    # ax2.set_title('"Plane"')
    mover = utils.MovePolicy(utils.MoveModePlane(), origin=orig, vector=vect)
    checkTest(1, ax2)
    ax3 = fig.add_subplot(223, projection="3d")
    # ax3.set_title('"Set"')
    checkTest(1, ax3)
    ax4 = fig.add_subplot(224, projection="3d")
    # ax4.set_title('"Free"')
    checkTest(1, ax4)
    fig.set_tight_layout(True)


    plt.show()


    print("pdone")
    quit()