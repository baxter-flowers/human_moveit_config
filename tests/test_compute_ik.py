#!/usr/bin/env python
import unittest
import rospy
import time
import numpy as np
from human_moveit_config.human_model import HumanModel


class TestTransformation(unittest.TestCase):
    def test_compute_ik(self):
        rospy.init_node('test_compute_ik')
        human = HumanModel()

        # move back to initial pose
        init_state = human.get_current_state()
        human.send_state(init_state)

        start = time.time()
        nb_iter = 0

        # sides = ['right', 'left']
        # links = ['torso', 'head_tip']
        # for j in range(2):
        #         links += [sides[j]+'_upper_arm', sides[j]+'_forearm', sides[j]+'_hand_tip']

        links = ['right_hand_tip']

        while nb_iter < 100 and not rospy.is_shutdown():
            # get random pose
            state = human.get_random_state()

            fk = human.forward_kinematic(state, links=links)

            # calculate the inverse kinematic
            ik = human.inverse_kinematic(fk)
            # calculate fk(ik)
            fk_compared = human.forward_kinematic(ik, links=links)

            print 'result'
            for key, value in fk.iteritems():
                print key + ': ' + str(value) + ' ~ ' + str(fk_compared[key])
                print '---'
            print '-----------'

            rospy.sleep(10)
            print 'run number '+str(nb_iter)+' passed'
            nb_iter += 1
            print "time: "+str(time.time() - start)

if __name__ == '__main__':
    unittest.main()
