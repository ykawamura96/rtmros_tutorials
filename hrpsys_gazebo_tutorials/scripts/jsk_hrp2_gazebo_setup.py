#!/usr/bin/env python
# -*- coding: utf-8 -*-

try:
    from hrpsys_ros_bridge_tutorials.hrp2_hrpsys_config import *
except:
    print "Do not import hrpsys_ros_bridge_tutorials.hrp2_hrpsys_config, import hrp2_hrpsys_config directly."
    from hrp2_hrpsys_config import *

pkg = 'jsk_hrp2_ros_bridge'
import imp
try:
    imp.find_module(pkg)
except:
    print "No such package ", pkg

try:
    from jsk_hrp2_ros_bridge.HRP3HandControllerService_idl import *
except:
    print """No HRP3hand module is availabe on this machine, 
it may cause some error"""

# setting
rtm.nsport = 15005

class JSKHRP2RealHrpsysConfigurator(JSKHRP2HrpsysConfigurator):
    hc = None
    hc_svc = None
    ROBOT_NAME = None

    def connectComps(self):
        JSKHRP2HrpsysConfigurator.connectComps(self)
        if self.rh.port("servoState") != None:
            if self.hc:
                connectPorts(self.rh.port("servoState"), self.hc.port("servoStateIn"))

    def getRTCList(self):
        rtclist = [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            #['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            #['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['octd', "ObjectContactTurnaroundDetector"],
            ['es', "EmergencyStopper"],
            ['rfu', "ReferenceForceUpdater"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            #['tc', "TorqueController"],
            ['te', "ThermoEstimator"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['tl', "ThermoLimiter"],
            ['bp', "Beeper"],
            ['log', "DataLogger"],
            ]
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            rtclist.append(['hc', "HRP3HandController"])
        if self.ROBOT_NAME.find("HRP2JSKNT") == -1: # if HRP2W, HRP2G and HRP2JSK
            tmpidx=rtclist.index(['ic', "ImpedanceController"])
            rtclist[0:tmpidx+1]+rtclist[tmpidx+1:]
            rtclist=rtclist[0:tmpidx+1]+[['gc', "GraspController"]]+rtclist[tmpidx+1:]
        print >>sys.stderr, "RTC ", rtclist, "[",self.ROBOT_NAME.find("HRP2JSKNT"),"][",self.ROBOT_NAME,"]"
        return rtclist

    def init(self):
        # Use grxuser's robot model for all local users
        url="/home/ykawamura/catkin_ws/src/rtm-ros-robotics/rtmros_hrp2/hrp2_models/"+self.ROBOT_NAME+"_for_OpenHRP3/"+self.ROBOT_NAME+"main.wrl"
        JSKHRP2HrpsysConfigurator.init(self, self.ROBOT_NAME, url)

    def setupLogger(self):
        # Stop all rtcs other than DataLogger for fast connecting of logger data ports.
        rtcList = self.getRTCInstanceList()
        for r in rtcList:
            if r.name() != "log":
                r.stop()
        JSKHRP2HrpsysConfigurator.setupLogger(self)
        # Start all rtcs again
        for r in rtcList:
            if r.name() != "log":
                r.start()
        #default = 4000 too shoort!
        #self.log_svc.maxLength(400000) # 2000 min
        self.log_svc.maxLength(int((5 * 60) / 0.004)) # 5 min, controller 0.004[s]
        self.log_svc.clear()

    # activateComps and waitForROSBridge are just for debugging.
    def activateComps(self):
#        self.waitForROSBridge()
        JSKHRP2HrpsysConfigurator.activateComps(self)

    def waitForROSBridge(self):
        bridge = False
        while bridge == False:
            import time; time.sleep(5)
            tmpbridge = rtm.findRTC('HrpsysSeqStateROSBridge0')
            if tmpbridge != None:
                bridge = tmpbridge.isActive()
            print self.configurator_name, "wait for setup ROSBridge : ",bridge, tmpbridge

    def isServoOn(self, jname='any'):
        '''!@brief
        Check whether servo control has been turned on.
        @param jname str: Name of a link (that can be obtained by "hiro.Groups"
                      as lists of groups).
        @return bool: True if servo is on
        '''
        if self.simulation_mode:
            return True
        else:
            s_s = self.getActualState().servoState
            if jname.lower() == 'any' or jname.lower() == 'all':
                for s in s_s:
                    # print self.configurator_name, 's = ', s
                    if (s[0] & 2) == 0:
                        return False
                return True
            elif jname.lower() == 'some':
                for s in s_s:
                    # print self.configurator_name, 's = ', s
                    if (s[0] & 2) != 0:
                        return True
                return False
            else:
                jid = eval('self.' + jname)
                print self.configurator_name, s_s[jid]
                if s_s[jid][0] & 1 == 0:
                    return False
                else:
                    return True
        return False

    # TODO : we should use original servoOn() and servoOff(), but it does not work?
    def servoOn(self, jname='all', destroy=1, tm=3, wait=True):
        # check servo state
        if self.isServoOn('some'):
            return 1

        # check jname is acceptable
        if jname == '':
            jname = 'all'

        if wait:
            c = False
            while (c != 'Y' and c != 'y'):
                c = raw_input("press 'Y' for servo ON and power ON. >> ")

        self.rh_svc.power(jname ,OpenHRP.RobotHardwareService.SWITCH_ON)
        for jgroups in self.Groups:
            self.seq_svc.removeJointGroup(jgroups[0])
        for jgroups in self.Groups:
            self.seq_svc.waitInterpolationOfGroup(jgroups[0])
        for jgroups in self.Groups:
            self.seq_svc.addJointGroup(jgroups[0], jgroups[1])
        # move to idle mode for filter type RTCs
        for rtc in self.getJointAngleControllerList():
            rtc.stop();
            rtc.start();
        if self.rfu_svc != None:
            self.rfu_svc.stopReferenceForceUpdater('larm') # stop RFU
            self.rfu_svc.stopReferenceForceUpdater('rarm') # stop RFU
        time.sleep(1)
        self.sh_svc.goActual()
        time.sleep(0.1)
        self.rh_svc.setServoGainPercentage("all", 100.0)
        self.rh_svc.servo(jname,OpenHRP.RobotHardwareService.SWITCH_ON)
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            time.sleep(0.1)
            #self.rh_svc.servo("LARM_JOINT6",OpenHRP.RobotHardwareService.SWITCH_OFF)
            self.rh_svc.servo("RARM_JOINT7",OpenHRP.RobotHardwareService.SWITCH_OFF)
            self.rh_svc.servo("LARM_JOINT7",OpenHRP.RobotHardwareService.SWITCH_OFF)
            self.rh_svc.setServoGainPercentage("RLEG_JOINT6", 30.0)
            self.rh_svc.setServoGainPercentage("LLEG_JOINT6", 30.0)
        elif self.ROBOT_NAME.find("HRP2W") != -1: # If you want to use external hands (Ex. tomato tasks) instead of HRP2W grippers, Comment in the following "RARM_JOINT7" and "LARM_JOINT7" servo OFF mode part
            time.sleep(0.1)
            self.rh_svc.servo("RARM_JOINT7",
                              OpenHRP.RobotHardwareService.SWITCH_OFF)
            self.rh_svc.servo("LARM_JOINT7",
                              OpenHRP.RobotHardwareService.SWITCH_OFF)
        elif self.ROBOT_NAME.find("HRP2G") != -1: # If you want to use external hands (Ex. MBZIRC wrench tasks) instead of HRP2G grippers, Comment in the following "RARM_JOINT7" and "LARM_JOINT7" servo OFF mode part
            time.sleep(0.1)
            self.rh_svc.servo("RARM_JOINT7",
                              OpenHRP.RobotHardwareService.SWITCH_OFF)
            self.rh_svc.servo("LARM_JOINT7",
                              OpenHRP.RobotHardwareService.SWITCH_OFF)
        time.sleep(2)
        return 0

    def servoOff(self, jname='all', wait=True):
        # if the servos aren't on switch power off
        # if not self.isServoOn(jname):
        #     if jname.lower() == 'all':
        #         self.rh_svc.power('all', OpenHRP.RobotHardwareService.SWITCH_OFF)
        #     return 1

        # if jname is not set properly set to all -> is this safe?
        if jname == '':
            jname = 'all'

        if wait:
            c = False
            while (c != 'Y' and c != 'y'):
                c = raw_input("press 'Y' for servo OFF and power OFF. >> ")

        self.rh_svc.servo(jname ,OpenHRP.RobotHardwareService.SWITCH_OFF)
        self.rh_svc.power(jname ,OpenHRP.RobotHardwareService.SWITCH_OFF)
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1 and self.hc_svc != None:
            self.hc_svc.handServoOff()
        return 0

    def loadForceMomentOffsetFile (self):
        # Use grxuser's robot model for all local users
        if self.ROBOT_NAME == "HRP2JSKNT":
            #self.rmfo_svc.loadForceMomentOffsetParams(os.environ["HOME"]+"/jsk_hrp2_ros_bridge/force_sensor_calib_data/hand-calib_HRP2JSKNT_20150127")
            self.rmfo_svc.loadForceMomentOffsetParams("/home/ykawamura/catkin_ws/src/rtm-ros-robotics/rtmros_tutorials/hrpsys_gazebo_tutorials/force_sensor_calib_data/hand-calib_HRP2JSKNT_20150127")
        elif self.ROBOT_NAME == "HRP2JSKNTS":
            #self.rmfo_svc.loadForceMomentOffsetParams(os.environ["HOME"]+"/jsk_hrp2_ros_bridge/force_sensor_calib_data/hand-calib_HRP2JSKNTS_20161025")
            self.rmfo_svc.loadForceMomentOffsetParams("/home/ykawamura/catkin_ws/src/rtm-ros-robotics/rtmros_tutorials/hrpsys_gazebo_tutorials/config/hand-calib_HRP2JSKNTS")
        elif self.ROBOT_NAME == "HRP2JSK":
            print "None"
        else:
            print "None"

    def startImpedanceController(self):
        self.ic_svc.startImpedanceController("larm")
        self.ic_svc.startImpedanceController("rarm")

    def stopImpedanceController(self):
        self.ic_svc.stopImpedanceController("larm")
        self.ic_svc.stopImpedanceController("rarm")

    def servoOnWithResetPose(self, wait=True):
        if self.servoOn(wait=wait) == 0:
            self.seq_svc.setJointAngles(self.hrp2ResetPose(), 3.0)
            print "go to reset-pose"
            self.seq_svc.waitInterpolation()
            if self.ROBOT_NAME.find("HRP2JSKNT") != -1 and self.hc_svc != None:
                print "hand joint calib"
                self.hc_svc.handServoOn()
                self.hc_svc.handJointCalib()

    def servoOnWithResetPoseDefaultUnstableControllers(self, wait=True):
        self.servoOnWithResetPose(wait)
        self.startDefaultUnstableControllers()

    def hrp3HandResetPose(self):
        self.hc_svc.setJointAngles([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 2)
        self.hc_svc.waitInterpolation()

    def hrp3HandGraspPose(self):
        self.hc_svc.setJointAngles([77.9709, -11.4732, 8.28742, 0.0, 106.185, 86.0974, 77.9709, -11.4732, 8.28742, 0.0, 106.185, 86.0974], 2)
        self.hc_svc.waitInterpolation()

    def hrp3HandHookPose(self):
        self.hc_svc.setJointAngles([90.0, 90.0, 0.0, 10.0, -20.0, -20.0, 90.0, 90.0, 0.0, 10.0, -20.0, -20.0], 2)
        self.hc_svc.waitInterpolation()

    def hrp3HandCalib(self):
        self.hc_svc.handJointCalib()

    def hrp3HandServoOn(self):
        self.hc_svc.handServoOn()

    def hrp3HandServoOff(self):
        self.hc_svc.handServoOff()

    def hrp3HandReconnect(self):
        self.hc_svc.handReconnect()

    def getConfFileRTCSettingsString (self, additional_rtc_list, robot_conf_file_path):
        '''!@brief
        Get string for RTC setting for the rtcd argument
        @return string : RTC setting string such as "-o 'example.DataLogger.config_file:XXX' ..."
        '''
        rtc_list = map (lambda x : x[1], self.getRTCList())
        for add_rtc in additional_rtc_list:
            rtc_list.append(add_rtc)
        tmpstr = ""
        for rtc in rtc_list:
            tmpstr += "-o 'example."+rtc+".config_file:"+robot_conf_file_path+"' "
        return tmpstr

    def getHRP2ConfFileRTCSettingsString (self):
        '''!@brief
        Get string for RTC setting for the rtcd argument for HRP2
        @return string : RTC setting string for HRP2 robots
        '''
        # Use grxuser's robot model for all local users
        robot_conf_file_path = "/home/ykawamura/catkin_ws/src/rtm-ros-robotics/rtmros_tutorials/hrpsys_gazebo_tutorials/config/"+self.ROBOT_NAME+".conf"
        additional_rtc_list = []
        # Additional 1. RobotHardware is not included in getRTCList
        additional_rtc_list.append('RobotHardware')
        # Additional 2. Do not used in getRTCList?
        additional_rtc_list.append('VirtualForceSensor')
        additional_rtc_list.append('TorqueFilter')
        additional_rtc_list.append('ServoController')
        additional_rtc_list.append('TorqueController')
        return self.getConfFileRTCSettingsString(additional_rtc_list, robot_conf_file_path)

    def check_argument (self):
        import sys
        if len(sys.argv) == 2 and sys.argv[1] == "--init":
            self.init()
            self.setStAbcParametersReal()
            self.loadForceMomentOffsetFile()
        elif len(sys.argv) == 2 and sys.argv[1] == "--getRTCList":
            print self.getHRP2ConfFileRTCSettingsString()
            exit(0)
        else:
            self.findComps()
            self.waitForRobotHardware()
            self.checkSimulationMode()
            ### servo ###
            if len(sys.argv) == 2 and sys.argv[1] == "--servoOn":
                self.servoOnWithResetPose()
                exit()
            elif len(sys.argv) == 2 and sys.argv[1] == "--servoOff":
                self.servoOff()
                exit()
            ### auto-balancer ###
            elif len(sys.argv) == 2 and sys.argv[1] == "--startABC":
                self.startAutoBalancer()
                exit()
            elif len(sys.argv) == 2 and sys.argv[1] == "--stopABC":
                self.stopAutoBalancer()
                exit()
            ### stabilizer ###
            elif len(sys.argv) == 2 and sys.argv[1] == "--startST":
                self.startStabilizer()
                exit()
            elif len(sys.argv) == 2 and sys.argv[1] == "--stopST":
                self.stopStabilizer()
                exit()
            ### impedance ###
            elif len(sys.argv) == 2 and sys.argv[1] == "--startIMP":
                self.startImpedanceController()
                exit()
            elif len(sys.argv) == 2 and sys.argv[1] == "--stopIMP":
                self.stopImpedanceController()
                exit()
            ### pose ###
            elif len(sys.argv) == 2 and sys.argv[1] == "--resetPose":
                self.setResetPose()
                exit()
            elif len(sys.argv) == 2 and sys.argv[1] == "--resetManipPose":
                self.setResetManipPose()
                exit()
            elif len(sys.argv) == 2 and sys.argv[1] == "--initPose":
                self.setInitPose()
                exit()
            ### hand ###
            elif len(sys.argv) == 2 and sys.argv[1] == "--handCalib":
                self.hrp3HandCalib()
                exit()
            elif len(sys.argv) == 2 and sys.argv[1] == "--handResetPose":
                self.hrp3HandResetPose()
                exit()
            elif len(sys.argv) == 2 and sys.argv[1] == "--handGraspPose":
                self.hrp3HandGraspPose()
                exit()
            elif len(sys.argv) == 2 and sys.argv[1] == "--handHookPose":
                self.hrp3HandHookPose()
                exit()

    def setStAbcParametersReal (self):
        if self.ROBOT_NAME == "HRP2JSKNT":
            self.setStAbcParametershrp2016cReal()
        elif self.ROBOT_NAME == "HRP2JSKNTS":
            self.setStAbcParametershrp2017cReal() # for hrp2017
        elif self.ROBOT_NAME == "HRP2JSK":
            self.setStAbcParametershrp2007cReal() 

    # for eefm Stabilizer, hrp2017, new
    def setStAbcParametershrp2017cReal(self):
        # KF parameters
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        self.kf_svc.setKalmanFilterParam(kfp)

    def setStAbcParametershrp2016cReal(self):
        # KF parameters
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        self.kf_svc.setKalmanFilterParam(kfp)

    def setStAbcParametershrp2007cReal(self):
        # KF parameters
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        self.kf_svc.setKalmanFilterParam(kfp)

    def __init__(self, robotname=""):
        if robotname=="":
            if os.environ["HRP2NO"] == '17':
                self.ROBOT_NAME = "HRP2JSKNTS"
            elif os.environ["HRP2NO"] == '16':
                self.ROBOT_NAME = "HRP2JSKNT"
            elif os.environ["HRP2NO"] == '7':
                self.ROBOT_NAME = "HRP2JSK"
            elif os.environ["HRP2NO"] == '8':
                self.ROBOT_NAME = "HRP2W"
            elif os.environ["HRP2NO"] == '18':
                self.ROBOT_NAME = "HRP2G"
        else:
            self.ROBOT_NAME = robotname
        print >>sys.stderr, "In jsk_hrp2_setup, ROBOT_NAME is ", self.ROBOT_NAME
        JSKHRP2HrpsysConfigurator.__init__(self, self.ROBOT_NAME)
