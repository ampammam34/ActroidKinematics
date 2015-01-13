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
import scipy as sp
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
		self._d_poseout = RTC.TimedPose3D(RTC.Time(0,0),0)
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
                th = []
                try:
                        def rotationXandOffset(x, y, z, th):
                                s = math.sin(th)
                                c = math.cos(th)
                                P = np.array([[1,0,0,x],[0,c,s,y],[0,-s,c,z],[0,0,0,1]])
                                return P

                        def rotationYandOffset(x, y, z, th):
                                s = math.sin(th)
                                c = math.cos(th)
                                P = np.array([[c,0,s,x],[0,1,0,y],[-s,0,c,z],[0,0,0,1]])
                                return P

                        def rotationZandOffset(x, y, z, th):
                                s = math.sin(th)
                                c = math.cos(th)
                                P = np.array([[c,s,0,x],[-s,c,0,y],[0,0,1,z],[0,0,0,1]])
                                return P


                        #if __name__ == '__main__':
                        if self._poseinIn.isNew():
                                data = self._poseinIn.read()
                                for num in range(8, 15):
                                        value = data.data[num]
                                        th.append(value)
                                l1 = 10
                                l2 = 12
                                l3 = 15
                                T = [0]*7

                                T1 = rotationYandOffset(0, 0, 0, th[0])  
                                T2 = rotationXandOffset(0, 0, 0, th[1]) 
                                T3 = rotationZandOffset(0, 0, l1, th[2])
                                T4 = rotationYandOffset(0, 0, 0, th[3]) 
                                T5 = rotationZandOffset(0, 0, l2, th[4]) 
                                T6 = rotationYandOffset(0, 0, 0, th[5])  
                                T7 = rotationXandOffset(l3, 0, 0, th[6])
    
                                Hand = np.array([[0],[0],[0],[1]])

                                T = [T1,T2,T3,T4,T5,T6,T7]

                                target_T = sp.dot(T1,sp.dot(T2,sp.dot(T3,sp.dot(T4,sp.dot(T5,sp.dot(T6,sp.dot(T7,Hand)))))))

                                print 'Hand Positoin is ', target_T
                                #　最初のデータからの結果しか出ない。
    
                                #raw_input(); #リターンキーを押下するまでロック
	
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

