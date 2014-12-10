#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ActroidKinematics.py
 @brief ModuleDescription
 @date $Date$


"""
import sys
import time
import numpy as np
import math
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist


# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
actroidkinematics_spec = ["implementation_id", "ActroidKinematics", 
		 "type_name",         "ActroidKinematics", 
		 "description",       "ModuleDescription", 
		 "version",           "1.0.0", 
		 "vendor",            "VenderName", 
		 "category",          "Category", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 ""]
# </rtc-template>

##
# @class ActroidKinematics
# @brief ModuleDescription
# 
# 
class ActroidKinematics(OpenRTM_aist.DataFlowComponentBase):
	
	##
	# @brief constructor
	# @param manager Maneger Object
	#




	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_posein = RTC.TimedDoubleSeq(RTC.Time(0,0),[])
		"""
		"""
		self._poseinIn = OpenRTM_aist.InPort("posein", self._d_posein)
		self._d_poseout = RTC.TimedDoubleSeq(RTC.Time(0,0),[])
		"""
		"""
		self._poseoutOut = OpenRTM_aist.OutPort("poseout", self._d_poseout)


		


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		
		# </rtc-template>


		 
	##
	#
	# The initialize action (on CREATED->ALIVE transition)
	# formaer rtc_init_entry() 
	# 
	# @return RTC::ReturnCode_t
	# 
	#
	def onInitialize(self):
		# Bind variables and configuration variable
		
		# Set InPort buffers
		self.addInPort("posein",self._poseinIn)
		
		# Set OutPort buffers
		self.addOutPort("poseout",self._poseoutOut)
		
		# Set service provider to Ports
		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports
		
		return RTC.RTC_OK
	
	#	##
	#	# 
	#	# The finalize action (on ALIVE->END transition)
	#	# formaer rtc_exiting_entry()
	#	# 
	#	# @return RTC::ReturnCode_t
	#
	#	# 
	#def onFinalize(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The startup action when ExecutionContext startup
	#	# former rtc_starting_entry()
	#	# 
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onStartup(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The shutdown action when ExecutionContext stop
	#	# former rtc_stopping_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onShutdown(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The activated action (Active state entry action)
	#	# former rtc_active_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	# 
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onActivated(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The deactivated action (Active state exit action)
	#	# former rtc_active_exit()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onDeactivated(self, ec_id):
	#
	#	return RTC.RTC_OK
	
		##
		#
		# The execution action that is invoked periodically
		# former rtc_active_do()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onExecute(self, ec_id):
                try:
                        if self._poseinIn.isNew():
                                data = self._poseinIn.read()
                                th1 = data.data[8]
                                th2 = data.data[9]
                                th3 = data.data[10]
                                th4 = data.data[11]
                                th5 = data.data[12]
                                th6 = data.data[13]
                                th7 = data.data[14]
                                print th1,th2,th3,th4,th5,th6,th7
                                
                                def func(x,y,z,th):
                                        tranX(x,y,z,th)
                                        s = sin(th)
                                        c = cos(th)
                        
                                        R7 = tranX(0,0,0,th7)
                                        R6 = tranX(0,0,0,th6)
                                        R5 = tranX(0,0,0,th5)
                                        R4 = tranX(0,0,0,th4)
                                        R3 = tranX(0,0,0,th3)
                                        R2 = tranX(0,0,0,th2)
                                        R1 = tranX(0,0,0,th1)
                                        print "good"  # ←プリントされない

                                        P = np.array([[1,0,0,x],[0,c,s,y],[0,-s,c,z],[0,0,0,1]])

                                        P = R1*R2*R3*R4*R5*R6*R7*Roffset*np.array([[0],[0],[1]])
                                        self._d_poseout.data = P[x,y]
                                        self._poseoutOut.write()
                                        print self._d_poseout.data
                                        return P   
	
                        return RTC.RTC_OK
                
                except Exception, e:
                        print 'Exception : ', e
                        traceback.print_exc()
                        #これは print_exception(sys.exc_type, sys.exc_value, sys.exc_traceback, limit, file) の省略表現
                        pass

                return RTC.RTC_OK
	
	
	#	##
	#	#
	#	# The aborting action when main logic error occurred.
	#	# former rtc_aborting_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onAborting(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The error action in ERROR state
	#	# former rtc_error_do()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onError(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The reset action that is invoked resetting
	#	# This is same but different the former rtc_init_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onReset(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The state update action that is invoked after onExecute() action
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#

	#	#
	#def onStateUpdate(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The action that is invoked when execution context's rate is changed
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onRateChanged(self, ec_id):
	#
	#	return RTC.RTC_OK
	
def func(x,y,z,th):
        tranX = (x,y,z,th)
        s = sin(th)
        c = cos(th)
        
        R7 = tranX(0,0,0,th7)
        R6 = tranX(0,0,0,th6)
        R5 = tranX(0,0,0,th5)
        R4 = tranX(0,0,0,th4)
        R3 = tranX(0,0,0,th3)
        R2 = tranX(0,0,0,th2)
        R1 = tranX(0,0,0,th1)

        P = np.array([[1,0,0,x],[0,c,s,y],[0,-s,c,z],[0,0,0,1]])
        print P
        
        P = R1*R2*R3*R4*R5*R6*R7*Roffset*np.array([[0],[0],[1]])
        return P

def ActroidKinematicsInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=actroidkinematics_spec)
    manager.registerFactory(profile,
                            ActroidKinematics,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    ActroidKinematicsInit(manager)

    # Create a component
    comp = manager.createComponent("ActroidKinematics")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

