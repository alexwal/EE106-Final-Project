def rotate_zumy(direction):
    directions = {"F" : 0., "L" : 90., "R" : 270., "B" : 180.} # (CCW, 0 is N) the direction that the the zumy will face
    zumy_vel = rospy.Publisher('%s/cmd_vel' % zumy, Twist, queue_size=2)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        # YOUR CODE HERE
        #  The code should compute the twist given 
        #  the translation and rotation between arZ and ar1
        #  Then send it publish it to the zumy
        # omega, theta = eqf.quaternion_to_exp(rot)
        omega = [0, 0, 1] # rotate about z
        translation = [0, 0, 0]
        theta = directions[direction]
        omega = omega * theta

        rbt = eqf.create_rbt(omega, theta, translation)
        v = eqf.find_v(omega, theta, trans)
        linear = Vector3(v[0]/5, v[1]/5, v[2]/5)
        angular = Vector3(omega[0], omega[1], omega[2])
        xi = Twist(linear, angular)

        zumy_vel.publish(xi)
        rate.sleep()
