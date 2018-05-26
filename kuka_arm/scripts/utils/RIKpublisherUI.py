#!/usr/bin/env python

import os
import sys
import threading
import math

import rospy
import rospkg

# binding provider is PyQt5; use its own specific API
# Check here for a good tutorial : http://zetcode.com/gui/pyqt5/
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

import tf

# messages definitions
from geometry_msgs.msg import Pose, PoseArray
# logger library to get the saved paths from move it
from RPathLogger import *

DEFAULT_RANGE_TICKS = 100

class SliderWrapper :

    def __init__( self, layout, slider, label, text, rmin, rmax, ticks = 100 ) :
        # widgets
        self.m_wslider = slider
        self.m_wlabel = label
        self.m_wtext = text
        # layout holder
        self.m_layout = layout
        # properties
        self.m_min = rmin
        self.m_max = rmax
        self.m_range = rmax - rmin
        self.m_ticks = ticks

        self._updateValue()

    def getLayout( self ) :
        return self.m_layout

    def getMin( self ) : 
        return self.m_min

    def getMax( self ) : 
        return self.m_max

    def getRange( self ) : 
        return self.m_range

    def getTicks( self ) :
        return self.m_ticks

    def getValue( self ) :
        return self.m_value

    def _updateValue( self ) :
        self.m_value = self.m_min + self.m_range * ( self.m_wslider.value() / float( self.m_ticks ) )
        self.m_wtext.setText( str( self.m_value ) )


class RIKpublisherUI( QWidget ) :


    def __init__( self ) :

        super( RIKpublisherUI, self ).__init__()

        # Publisher for the slider valuess
        self.m_singlePosePublisher = rospy.Publisher( "IK_pose_reference", Pose, queue_size = 10 )
        # Publisher for the reference trajectory
        self.m_trajectoryPublisher = rospy.Publisher( "/IK_trajectory_reference", PoseArray, queue_size = 10 )

        # Create threading resources to handle ui-rospy interaction
        self.m_rate = rospy.Rate( 10 )
        self.m_lock = threading.Lock()

        # Thread to run the ros functionality
        self.m_workerRos = threading.Thread( target = self.workerFcn )

        # Flags to check whether or not to publish the target or drop-off trajectory
        self.m_publishFlags = { 'target' : False, 'dropoff' : False }
        # Logger to load the stored paths
        self.m_pathLogger = RPathLogger()

        # Make UI
        _ui_vbox = QVBoxLayout()

        self.m_chbox_mode = QCheckBox( 'Use trajectory' )
        _ui_vbox.addWidget( self.m_chbox_mode )

        self.m_sld_x = self._buildSlider( -3.1, 3.1, 'x' )
        self.m_sld_y = self._buildSlider( -3.1, 3.1, 'y' )
        self.m_sld_z = self._buildSlider( -2.0, 3.5, 'z' )
        self.m_sld_roll  = self._buildSlider( -180, 180, 'roll' )
        self.m_sld_pitch = self._buildSlider( -180, 180, 'pitch' )
        self.m_sld_yaw   = self._buildSlider( -180, 180, 'yaw' )

        _ui_vbox.addLayout( self.m_sld_x.getLayout() )
        _ui_vbox.addLayout( self.m_sld_y.getLayout() )
        _ui_vbox.addLayout( self.m_sld_z.getLayout() )
        _ui_vbox.addLayout( self.m_sld_roll.getLayout() )
        _ui_vbox.addLayout( self.m_sld_pitch.getLayout() )
        _ui_vbox.addLayout( self.m_sld_yaw.getLayout() )

        self.m_btnTrajectoryTarget  = QPushButton( 'Send target trajectory' )
        self.m_btnTrajectoryDropoff = QPushButton( 'Send drop-off trajectory' )

        self.m_btnTrajectoryTarget.clicked.connect( self.onSendTargetTrajectory )
        self.m_btnTrajectoryDropoff.clicked.connect( self.onSendDropoffTrajectory )

        _ui_vbox.addWidget( self.m_btnTrajectoryTarget )
        _ui_vbox.addWidget( self.m_btnTrajectoryDropoff )

        self.setLayout( _ui_vbox )
        
        # clear time and start everything
        self.m_workerRos.start()

    def _buildSlider( self, rmin, rmax, name ) :
        _vbox = QVBoxLayout()
        _hbox = QHBoxLayout()

        _label = QLabel( name )
        _text = QLineEdit()
        _text.setDisabled( True )
        _slider = QSlider( Qt.Horizontal )
        _slider.setRange( 0, DEFAULT_RANGE_TICKS )
        _slider.setValue( DEFAULT_RANGE_TICKS / 2 )
        _slider.valueChanged.connect( self.onSliderValueChanged )

        _hbox.addWidget( _label )
        _hbox.addWidget( _text )
        _vbox.addLayout( _hbox )
        _vbox.addWidget( _slider )

        _sliderWrapped = SliderWrapper( _vbox, _slider, _label, _text, 
                                        rmin, rmax, DEFAULT_RANGE_TICKS )

        return _sliderWrapped

    def onSliderValueChanged( self ) :
        # Update all sliders
        self.m_lock.acquire( True )

        self.m_sld_x._updateValue()
        self.m_sld_y._updateValue()
        self.m_sld_z._updateValue()
        self.m_sld_roll._updateValue()
        self.m_sld_pitch._updateValue()
        self.m_sld_yaw._updateValue()

        self.m_lock.release()

    def onSendTargetTrajectory( self ) :
        self.m_lock.acquire( True )
        self.m_publishFlags['target'] = True
        self.m_publishFlags['dropoff'] = False
        self.m_lock.release()

    def onSendDropoffTrajectory( self ) :
        self.m_lock.acquire( True )
        self.m_publishFlags['dropoff'] = True
        self.m_publishFlags['target'] = False
        self.m_lock.release()

    def _makeTrajectoryMsg( self, trajectoryId = 'target' ) :

        _trajectory = None

        if trajectoryId == 'target' :
            _trajectory = self.m_pathLogger.getTrajectoryToTarget()
        else :
            _trajectory = self.m_pathLogger.getTrajectoryToDropoff()

        _poseArray = PoseArray()

        for i in range( len( _trajectory ) ) :
            _pose = Pose()
            _pose.position.x = _trajectory[i].position[0]
            _pose.position.y = _trajectory[i].position[1]
            _pose.position.z = _trajectory[i].position[2]
            _pose.orientation.x = _trajectory[i].orientation[0]
            _pose.orientation.y = _trajectory[i].orientation[1]
            _pose.orientation.z = _trajectory[i].orientation[2]
            _pose.orientation.w = _trajectory[i].orientation[3]

            _poseArray.poses.append( _pose )

        return _poseArray

    def _makePoseMsg( self ) :

        _pose = Pose()
        _pose.position.x = self.m_sld_x.getValue()
        _pose.position.y = self.m_sld_y.getValue()
        _pose.position.z = self.m_sld_z.getValue()

        _quat = tf.transformations.quaternion_from_euler( self.m_sld_roll.getValue(),
                                                          self.m_sld_pitch.getValue(),
                                                          self.m_sld_yaw.getValue() )

        _pose.orientation.x = _quat[0]
        _pose.orientation.y = _quat[1]
        _pose.orientation.z = _quat[2]
        _pose.orientation.w = _quat[3]

        return _pose

    def workerFcn( self ) :
        print 'running rospy thread!'

        while not rospy.is_shutdown() :

            self.m_lock.acquire( True )

            if self.m_chbox_mode.isChecked() :

                if self.m_publishFlags['target'] :
                    self.m_trajectoryPublisher.publish( self._makeTrajectoryMsg( 'target' ) )
                    self.m_publishFlags['target'] = False

                elif self.m_publishFlags['dropoff'] :
                    self.m_trajectoryPublisher.publish( self._makeTrajectoryMsg( 'dropoff' ) )
                    self.m_publishFlags['dropoff'] = False

            else :
                self.m_singlePosePublisher.publish( self._makePoseMsg() )

            self.m_lock.release()

            self.m_rate.sleep()

    def closeEvent( self, event ) :
        # stop rospy manually
        rospy.signal_shutdown( 'Closing UI' )
        # accept and do the default behavior
        event.accept()
        # clean some stuff stuff
        self.m_workerRos.join()

if __name__ == "__main__" :

    rospy.init_node( "RIKpublisherUI", disable_signals = True )

    _app = QApplication( sys.argv )

    _ui = RIKpublisherUI()
    _ui.show()

    sys.exit( _app.exec_() )