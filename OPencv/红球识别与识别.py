#-*- coding=GBK -*-

import sys
import os
from naoqi import ALProxy
import cv2
import time
import motion
import math
import almath
import numpy as np
sys.path.append("./")
import vision_definitions as vd
cv_version = cv2.__version__.split(".")[0]
if cv_version == "2": # for OpenCV 2
	import cv2.cv as cv



#------------------ConfigureNao.py--------------------#

class ConfigureNao(object):

	def __init__(self, IP, PORT=9559):
		self.IP = IP
		self.PORT = PORT
		try:
			self.cameraProxy = ALProxy("ALVideoDevice", self.IP, self.PORT)
			self.motionProxy = ALProxy("ALMotion", self.IP, self.PORT)
			self.postureProxy = ALProxy("ALRobotPosture", self.IP, self.PORT)
			self.tts = ALProxy("ALTextToSpeech",self.IP, self.PORT)
			self.memoryProxy = ALProxy("ALMemory", self.IP, self.PORT)
			self.landMarkProxy = ALProxy("ALLandMarkDetection", self.IP, self.PORT)
		except Exception, e:
			print("Error when configuring the NAO!")
			print(str(e))
			exit(1)

#------------------------------visualTask---------------------------#

class VisualBasis(ConfigureNao):

	def __init__(self, IP, PORT=9559, cameraId=vd.kBottomCamera, resolution=vd.kVGA):
		super(VisualBasis, self).__init__(IP, PORT)
		self.cameraId = cameraId
		self.cameraName = "CameraBottom" if self.cameraId==vd.kBottomCamera else "CameraTop"
		self.resolution = resolution
		self.colorSpace = vd.kBGRColorSpace
		self.fps = 20
		self.frameHeight = 0
		self.frameWidth = 0
		self.frameChannels = 0
		self.frameArray = None
		self.cameraPitchRange = 47.64/180*np.pi
		self.cameraYawRange = 60.97/180*np.pi
		self.cameraProxy.setActiveCamera(self.cameraId)


	def updateFrame(self, client="python_client"):

		if self.cameraProxy.getActiveCamera() != self.cameraId:
			self.cameraProxy.setActiveCamera(self.cameraId)
			time.sleep(1)

		videoClient = self.cameraProxy.subscribe(client, self.resolution, self.colorSpace, self.fps)

		frame = self.cameraProxy.getImageRemote(videoClient)

		self.cameraProxy.unsubscribe(videoClient)

		try:
			self.frameWidth = frame[0]
			self.frameHeight = frame[1]
			self.frameChannels = frame[2]
			self.frameArray = np.frombuffer(frame[6], dtype=np.uint8).reshape([frame[1],frame[0],frame[2]])

		except IndexError:
			print("get image failed!")


	def getFrameArray(self):
		if self.frameArray is None:
			return np.array([])
		return self.frameArray

	def showFrame(self):
		if self.frameArray is None:
			print("please get an image from Nao with the method updateFrame()")
		else:
			cv2.imshow("current frame", self.frameArray)


	def printFrameData(self):
		print("frame height = ", self.frameHeight)
		print("frame width = ", self.frameWidth)
		print("frame channels = ", self.frameChannels)
		print("frame shape = ", self.frameArray.shape)

	def saveFrame(self, framePath):
		cv2.imwrite(framePath, self.frameArray)
		print("current frame image has been saved in", framePath)


	def setParam(self, paramName=None, paramValue = None):
		raise NotImplementedError


	def setAllParamsToDefault(self):
		raise NotImplementedError


#--------------------------红球检测类--------------------------#

class BallDetect(VisualBasis):

	def __init__(self, IP, PORT=9559, cameraId=vd.kBottomCamera, resolution=vd.kVGA,
				 writeFrame=False):
		super(BallDetect, self).__init__(IP, PORT, cameraId, resolution)
		self.ballData = {"centerX":0, "centerY":0, "radius":0}
		self.ballPosition = {"disX":0, "disY":0, "angle":0}
		self.ballRadius = 0.025
		self.writeFrame = writeFrame


	'''-------------------------图像预处理-------------------------'''

	def __getChannelAndBlur(self, color):

		try:
			channelB = self.frameArray[:,:,0]
			channelG = self.frameArray[:,:,1]
			channelR = self.frameArray[:,:,2]
		except:
			print("no image detected!")

		Hm = 6
		if color == "red":
			channelB = channelB*0.1*Hm
			channelG = channelG*0.1*Hm
			channelR = channelR - channelB - channelG
			channelR = 3*channelR
			channelR = cv2.GaussianBlur(channelR, (9,9), 1.5)
			channelR[channelR<0] = 0
			channelR[channelR>255] = 255
			return np.uint8(np.round(channelR))
		elif color == "blue":
			channelR = channelR*0.1*Hm
			channelG = channelG*0.1*Hm
			channelB = channelB - channelG - channelR
			channelB = 3*channelB
			channelB = cv2.GaussianBlur(channelB, (9,9), 1.5)
			channelB[channelB<0] = 0
			channelB[channelB>255] = 255
			return np.uint8(np.round(channelB))
		elif color == "green":
			channelB = channelB*0.1*Hm
			channelR = channelR*0.1*Hm
			channelG = channelG - channelB - channelR
			channelG = 3*channelG
			channelG = cv2.GaussianBlur(channelG, (9,9), 1.5)
			channelG[channelG<0] = 0
			channelG[channelG>255] = 255
			return np.uint8(np.round(channelG))
		else:
			print("can not recognize the color!")
			print("supported color:red, green and blue.")
			return None


	def __binImageHSV(self, minHSV1, maxHSV1, minHSV2, maxHSV2):

		try:
			frameArray = self.frameArray.copy()
			imgHSV = cv2.cvtColor(frameArray, cv2.COLOR_BGR2HSV)
		except:
			print("no image detected!")
		else:
			frameBin1 = cv2.inRange(imgHSV, minHSV1, maxHSV1)
			frameBin2 = cv2.inRange(imgHSV, minHSV2, maxHSV2)
			frameBin = np.maximum(frameBin1, frameBin2)
			frameBin = cv2.GaussianBlur(frameBin, (9,9), 1.5)
			return frameBin


	'''------------------------红球识别--------------------------'''

	def __findCircles(self, img, minDist, minRadius, maxRadius):

		cv_version = cv2.__version__.split(".")[0]
		if cv_version == "3": # for OpenCV >= 3.0.0
			gradient_name = cv2.HOUGH_GRADIENT
		else:
			gradient_name = cv.CV_HOUGH_GRADIENT
		circles = cv2.HoughCircles(np.uint8(img), gradient_name, 1,
								   minDist, param1=150, param2=15,
								   minRadius=minRadius, maxRadius=maxRadius)
		if circles is None:
			return np.uint16([])
		else:
			return np.uint16(np.around(circles[0, ]))


	'''--------------------------红球筛选------------------------'''

	def __selectCircle(self, circles):
		if circles.shape[0] == 0:
			return circles
		if circles.shape[0] == 1:
			centerX = circles[0][0]
			centerY = circles[0][1]
			radius = circles[0][2]
			initX = centerX - 2*radius
			initY = centerY - 2*radius
			if (initX<0 or initY<0 or (initX+4*radius)>self.frameWidth or
			   (initY+4*radius)>self.frameHeight or radius<1):
				return circles
		channelB = self.frameArray[:,:,0]
		channelG = self.frameArray[:,:,1]
		channelR = self.frameArray[:,:,2]
		rRatioMin = 1.0; circleSelected = np.uint16([])
		for circle in circles:
			centerX = circle[0]
			centerY = circle[1]
			radius = circle[2]
			initX = centerX - 2*radius
			initY = centerY - 2*radius
			if initX<0 or initY<0 or (initX+4*radius)>self.frameWidth or \
			   (initY+4*radius)>self.frameHeight or radius<1:
				continue
			rectBallArea = self.frameArray[initY:initY+4*radius+1, initX:initX+4*radius+1,:]
			bFlat = np.float16(rectBallArea[:,:,0].flatten())
			gFlat = np.float16(rectBallArea[:,:,1].flatten())
			rFlat = np.float16(rectBallArea[:,:,2].flatten())
			rScore1 = np.uint8(rFlat>1.0*gFlat)
			rScore2 = np.uint8(rFlat>1.0*bFlat)
			rScore = float(np.sum(rScore1*rScore2))
			gScore = float(np.sum(np.uint8(gFlat>1.0*rFlat)))
			rRatio = rScore/len(rFlat)
			gRatio = gScore/len(gFlat)
			if rRatio>=0.12 and gRatio>=0.1 and abs(rRatio-0.19)<abs(rRatioMin-0.19):
				circleSelected = circle
				rRatioMin = rRatio
		return circleSelected


	def __updateBallPositionFitting(self, standState):
		bottomCameraDirection = {"standInit":49.2, "standUp":39.7}
		ballRadius = self.ballRadius
		try:
			cameraDirection = bottomCameraDirection[standState]
		except KeyError:
			print("Error! unknown standState, please check the value of stand state!")
		else:
			if self.ballData["radius"] == 0:
				self.ballPosition= {"disX":0, "disY":0, "angle":0}
			else:
				centerX = self.ballData["centerX"]
				centerY = self.ballData["centerY"]
				radius = self.ballData["radius"]
				cameraPosition = self.motionProxy.getPosition("CameraBottom", 2, True)
				cameraX = cameraPosition[0]
				cameraY = cameraPosition[1]
				cameraHeight = cameraPosition[2]
				headPitches = self.motionProxy.getAngles("HeadPitch", True)
				headPitch = headPitches[0]
				headYaws = self.motionProxy.getAngles("HeadYaw", True)
				headYaw = headYaws[0]
				ballPitch = (centerY-240.0)*self.cameraPitchRange/480.0   # y (pitch angle)
				ballYaw = (320.0-centerX)*self.cameraYawRange/640.0    # x (yaw angle)
				dPitch = (cameraHeight-ballRadius)/np.tan(cameraDirection/180*np.pi+headPitch+ballPitch)
				dYaw = dPitch/np.cos(ballYaw)
				ballX = dYaw*np.cos(ballYaw+headYaw)+cameraX
				ballY = dYaw*np.sin(ballYaw+headYaw)+cameraY
				ballYaw = np.arctan2(ballY, ballX)
				self.ballPosition["disX"] = ballX
				if (standState == "standInit"):
					ky = 42.513*ballX**4 - 109.66*ballX**3 + 104.2*ballX**2 - 44.218*ballX + 8.5526
					#ky = 12.604*ballX**4 - 37.962*ballX**3 + 43.163*ballX**2 - 22.688*ballX + 6.0526
					ballY = ky*ballY
					ballYaw = np.arctan2(ballY,ballX)
				self.ballPosition["disY"] = ballY
				self.ballPosition["angle"] = ballYaw



	def __updateBallPosition(self, standState):
		bottomCameraDirection = {"standInit":49.2/180*np.pi, "standUp":39.7/180*np.pi}
		try:
			cameraDirection = bottomCameraDirection[standState]
		except KeyError:
			print("Error! unknown standState, please check the value of stand state!")
		else:
			if self.ballData["radius"] == 0:
				self.ballPosition= {"disX":0, "disY":0, "angle":0}
			else:
				centerX = self.ballData["centerX"]
				centerY = self.ballData["centerY"]
				radius = self.ballData["radius"]
				cameraPos = self.motionProxy.getPosition(self.cameraName, motion.FRAME_WORLD, True)
				cameraX, cameraY, cameraHeight = cameraPos[:3]
				headYaw, headPitch = self.motionProxy.getAngles("Head", True)
				cameraPitch = headPitch + cameraDirection
				imgCenterX = self.frameWidth/2
				imgCenterY = self.frameHeight/2
				centerX = self.ballData["centerX"]
				centerY = self.ballData["centerY"]
				imgPitch = (centerY-imgCenterY)/(self.frameHeight)*self.cameraPitchRange
				imgYaw = (imgCenterX-centerX)/(self.frameWidth)*self.cameraYawRange
				ballPitch = cameraPitch + imgPitch
				#ballPitch = 38/180.0*3.14
				ballYaw = imgYaw + headYaw
				#ballYaw = 31/180.0*3.14
				dist = (cameraHeight-self.ballRadius)/np.tan(ballPitch) + np.sqrt(cameraX**2+cameraY**2)
				#print("height = ", cameraHeight)
				#print("cameraPitch = ", cameraPitch*180/3.14)
				#print("imgYaw = ", imgYaw/3.14*180)
				#print("headYaw = ", headYaw/3.14*180)
				#print("ballYaw = ",ballYaw/3.14*180)
				#print("ballPitch = ", ballPitch/3.14*180)
				disX = dist*np.cos(ballYaw)
				disY = dist*np.sin(ballYaw)
				#print("disX = ", disX)
				#print("disY = ", disY)
				self.ballPosition["disX"] = disX
				self.ballPosition["disY"] = disY
				self.ballPosition["angle"] = ballYaw

	def __writeFrame(self, saveDir="./ballData"):

		if not os.path.exists(saveDir):
			os.makedirs(saveDir)
		saveName=str(int(time.time()))
		saveImgPath = os.path.join(saveDir, saveName+".jpg")
		try:
			cv2.imwrite(saveImgPath, self.frameArray)
		except:
			print("Error when saveing current frame!")


	def updateBallData(self, client="python_client", standState="standInit", color="red",
					   colorSpace="BGR", fitting=False, minHSV1=np.array([0,43,46]),
					   maxHSV1=np.array([10,255,255]), minHSV2=np.array([156,43,46]),
					   maxHSV2=np.array([180,255,255]), saveFrameBin=False):

		self.updateFrame(client)
		minDist = int(self.frameHeight/30.0)
		minRadius = 1
		maxRadius = int(self.frameHeight/10.0)
		if colorSpace == "BGR":
			grayFrame = self.__getChannelAndBlur(color)
		else:
			grayFrame = self.__binImageHSV(minHSV1, maxHSV1, minHSV2, maxHSV2)
		if saveFrameBin:
			self._frameBin = grayFrame.copy()
		circles = self.__findCircles(grayFrame, minDist, minRadius, maxRadius)
		circle = self.__selectCircle(circles)
		if circle.shape[0] == 0:
			self.ballData = {"centerX":0, "centerY":0, "radius":0}
			self.ballPosition= {"disX":0, "disY":0, "angle":0}
		else:
			circle = circle.reshape([-1,3])
			self.ballData = {"centerX":circle[0][0], "centerY":circle[0][1], "radius":circle[0][2]}
			if fitting == True:
				self.__updateBallPositionFitting(standState=standState)
			else:
				self.__updateBallPosition(standState=standState)

			if self.writeFrame == True:
				self.__writeFrame()


	def getBallPosition(self):

		disX = self.ballPosition["disX"]
		disY = self.ballPosition["disY"]
		angle = self.ballPosition["angle"]
		return [disX, disY, angle]


	'''返回红球相对于机器人的位置'''
	def getBallInfoInImage(self):

		centerX = self.ballData["centerX"]
		centerY = self.ballData["centerY"]
		radius = self.ballData["radius"]
		return [centerX, centerY, radius]


	def showBallPosition(self):

		if self.ballData["radius"] == 0:
			#print("no ball found.")
			print("ball postion = ", (self.ballPosition["disX"], self.ballPosition["disY"]))
			cv2.imshow("ball position", self.frameArray)
		else:
			#print("ballX = ", self.ballData["centerX"])
			#print("ballY = ", self.ballData["centerY"])
			#print("ball postion = ", (self.ballPosition["disX"], self.ballPosition["disY"]))
			#print("ball direction = ", self.ballPosition["angle"]*180/3.14)
			frameArray = self.frameArray.copy()
			cv2.circle(frameArray, (self.ballData["centerX"],self.ballData["centerY"]),
					   self.ballData["radius"], (250,150,150),2)
			cv2.circle(frameArray, (self.ballData["centerX"],self.ballData["centerY"]),
					   2, (50,250,50), 3)
			cv2.imshow("ball position", frameArray)
			return frameArray


	def sliderHSV(self, client):

		def __nothing():
			pass
		windowName = "slider for ball detection"
		cv2.namedWindow(windowName) # 创建窗口（不写也可以显示窗口）02:鼠标控制图像大小--第二个参数为1/ cv.WINDOW_AUTOSIZE/不写时，鼠标不能控制图像大小；第二个参数为0/cv.WINDOW_NORMAL时，鼠标可以控制图像大小。
		cv2.createTrackbar("minS1", windowName, 43, 60, __nothing)
		'''函数createTrackbar()的用法'''
		cv2.createTrackbar("minV1", windowName, 46, 65, __nothing)
		cv2.createTrackbar("maxH1", windowName, 10, 20, __nothing)
		cv2.createTrackbar("minH2", windowName, 156, 175, __nothing)
		while 1:
			self.updateFrame(client)
			minS1 = cv2.getTrackbarPos("minS1", windowName)
			'''函数getTrackbarPos()的用法'''
			minV1 = cv2.getTrackbarPos("minV1", windowName)
			maxH1 = cv2.getTrackbarPos("maxH1", windowName)
			minH2 = cv2.getTrackbarPos("minH2", windowName)
			minHSV1 = np.array([0, minS1, minV1])
			maxHSV1 = np.array([maxH1, 255, 255])
			minHSV2 = np.array([minH2, minS1, minV1])
			maxHSV2=np.array([180,255,255])
			self.updateBallData(client, colorSpace="HSV", minHSV1=minHSV1,
								maxHSV1=maxHSV1, minHSV2=minHSV2,
								maxHSV2=maxHSV2, saveFrameBin=True, fitting=True)
			cv2.imshow(windowName, self._frameBin)
			self.showBallPosition()
			k = cv2.waitKey(10) & 0xFF
			if k == 27:
				break
		cv2.destroyAllWindows()




#---------------------------mian-visual-test------------------------#

#------------------------slider.py---------------------#

if __name__ == "__main__":
	IP = "192.168.3.16"
	# for ball detection
	#detector = BallDetect(IP, resolution=vd.kVGA)
	#detector.sliderHSV(client="test3")

	visualBasis = VisualBasis(IP,cameraId=0, resolution=vd.kVGA)
	ballDetect = BallDetect(IP, resolution=vd.kVGA, writeFrame=True)

	visualBasis.updateFrame()
	visualBasis.showFrame()
	visualBasis.printFrameData()
	cv2.waitKey(1000)

	# posture initialization
	motionProxy = ALProxy("ALMotion", IP, 9559)
	postureProxy = ALProxy("ALRobotPosture", IP, 9559)
	motionProxy.wakeUp()
	postureProxy.goToPosture("StandInit", 0.5)

	i = 0
	while 1:
		i = i+1
		time1 = time.time()
		ballDetect.updateBallData(client="xxxx", colorSpace="HSV", fitting=True)
		time2 = time.time()
		img = ballDetect.showBallPosition()
		motionProxy.wakeUp()
		cv2.waitKey(1000)
		if img is not None:
			print("i=",i)
			break

	#print  "start collecting..."
	for i in range(8):
		imgName = "stick_" + str(i+127) + ".jpg"
		imgDir = os.path.join("D:\daima\TuPianjiqiren", imgName)

		visualBasis.updateFrame()
		visualBasis.showFrame()
		visualBasis.saveFrame(imgDir)
		time.sleep(3)





