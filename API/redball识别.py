def Distinguish():

    memvalue = "redBallDetected"
    period = 1000
    redball.subscribe("Redball", period, 0.0)
    Flag = 0
    count = 0

    time.sleep(3.5)
    for i in range(0, 3):
        time.sleep(1.5)
        val = memory.getData(memvalue)
        if (val and isinstance(val, list) and len(val) >= 2):
            print "i saw the red ball !"
            tts.say("我看到了！")
            val = memory.getData(memvalue)
            ballinfo = val[1]
            thetah = ballinfo[0]
            thetav = ballinfo[1] + (39.7 * math.pi / 180.0)
            Flag = 1
            break
        else:
            count += 1
            print ("try count: ", count)
            print "where is the red ball !"
            tts.say("球在哪里呢？")
            thetah = 0.0
            Flag = 0

    sign = [thetah,Flag]

    return sign
