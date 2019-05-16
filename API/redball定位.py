def Searchredball():
    motion.wakeUp()
    rnumber = 1
    camera.setActiveCamera(1)
    motion.setMoveArmsEnabled(False,False)
    for i in range(0,3):
        motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
        motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
        motion.setMoveArmsEnabled(False,False)
        targlist = [45*math.pi/180.0 , 0.0 , -45*math.pi/180.0]

        effectornamelist = ["HeadYaw"]
        timelist = [0.5]
        for j in range(0,3):
            motion.angleInterpolation(effectornamelist, targlist[j], timelist, True)
            sign = Distinguish()
            headangle = motion.getAngles("HeadYaw", True)
            motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
            motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
            if sign[1] == 1 :
                if headangle[0] > 0.0:
                    if sign[0] < 0.0:
                        sign[0] += headangle[0]
                if headangle[0] < 0.0:
                    if sign[0] > 0.0:
                        sign[0] += headangle[0]
                thetah = sign[0]
                break


        if sign[1] == 0:
            motion.moveTo(0.3, 0.0, 0.0,
                        [["MaxStepX", maxstepx], ["MaxStepY", maxstepy], ["MaxStepTheta", maxsteptheta],
                        ["MaxStepFrequency", maxstepfrequency], ["StepHeight", stepheight],
                        ["TorsoWx", torsowx], ["TorsoWy", torsowy]])
            motion.waitUntilMoveIsFinished()

        elif sign[1] == 1:
            motion.angleInterpolation(effectornamelist, 0.0, timelist, True)
            break

    h = 0.478
    isenabled = False
    x = 0.0
    y = 0.0
    memvalue = "redBallDetected"
    period = 1000
    redball.subscribe("Redball", period, 0.0)
    theta = thetah
    motion.setMoveArmsEnabled(False, False)
    # 首先机器人转到正对红球的方向
    motion.moveTo(x, y, theta,
                  [["MaxStepX", maxstepx], ["MaxStepY", maxstepy], ["MaxStepTheta", maxsteptheta],
                   ["MaxStepFrequency", maxstepfrequency], ["StepHeight", stepheight],
                   ["TorsoWx", torsowx], ["TorsoWy", torsowy]])  # Turn

    time.sleep(0.5)

    val = memory.getData(memvalue)
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (39.7 * math.pi / 180.0)
    x = h / (math.tan(thetav)) - 0.35
    theta = 0.0
    motion.setMoveArmsEnabled(False, False)
