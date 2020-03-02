#!/usr/bin/python
"""
Copyright (c) 2015,
Systems, Robotics and Vision
University of the Balearic Islands
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision, University of
      the Balearic Islands nor the names of its contributors may be used to
      endorse or promote products derived from this software without specific
      prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
Modified by Emilio Garcia-Fidalgo (emilio.garcia@uib.es) in 2019 
to be used in the Mussol Aerial Platform as a logging node
"""

PKG = 'cola2_xiroi' # this package name
NAME = 'topic_logger'

import roslib; roslib.load_manifest(PKG)
import rospy
from std_srvs.srv import Empty
from cola2_xiroi.srv import RecordRequest, RecordRequestResponse
import string
import os
import sys
import time
import datetime
import subprocess

dir = rospy.get_param("topic_logger/dir", default='/home/' + os.getlogin())
core_name = rospy.get_param("topic_logger/core_name")
dir_save_bagfiles = dir + '/bagfiles'

# Load Topic list
topics = rospy.get_param("topic_logger/topics")

image_topics = [] 
if rospy.has_param('topic_logger/image_topics'):
  image_topics = rospy.get_param("topic_logger/image_topics")

pc_topics = []
if rospy.has_param('/topic_logger/pc_topics'):
  pc_topics = rospy.get_param("/topic_logger/pc_topics")

class TopicLogger:
  _recording = False

  def record(self, topics, folder, mission , split = False, size = 10000):
    args = ''
    if len(sys.argv) > 1:
      args = ' -o ' + folder + "/" + mission

    # Normal topics
    for t in topics:
      args = args + " " + t
    command = "rosbag record --lz4"
    if split:
      command = command + " --split --size " + str(size)
    command = command + args
    return subprocess.Popen(command, shell=True, cwd=folder)


  # Start recording the topics
  def start_recording(self, req):
    if (not self._recording):
      rospy.loginfo('Recording start requested')

      now = datetime.datetime.now()
      fullday   = now.strftime("%Y%m%d")
      fullhour  = now.strftime("%H_%M_%S")
      dir_save_curr_bagfile = os.path.join(dir_save_bagfiles,fullday, req.mission)

      if not os.path.exists(dir_save_curr_bagfile):
        os.makedirs(dir_save_curr_bagfile)

      self._recording = True
      # Current topics
      self.p = self.record(topics, dir_save_curr_bagfile, 'topics_'+ core_name, split=False, size=10000)
      rospy.loginfo('Recording process started with PID: {0}'.format(self.p.pid))

      # Image topics
      if image_topics:
        self.p_img = self.record(image_topics, dir_save_curr_bagfile, 'images_'+ core_name, split=False, size=10000)
        rospy.loginfo('Recording process started with PID: {0}'.format(self.p_img.pid))

      param_filename = "params_"+ core_name + ".yaml"
      rospy.loginfo('Saving parameters in {0}'.format(dir_save_curr_bagfile + '/' + param_filename))
      process = subprocess.Popen('rosparam dump ' + param_filename, shell=True, cwd=dir_save_curr_bagfile)
      time.sleep(10)
      process.kill()
    else:
      rospy.logwarn('Already recording...')
      return RecordRequestResponse(False)
    return RecordRequestResponse(True)

   #stop to record the topics
  def stop_recording(self, req):
    if (self._recording):
      rospy.loginfo('Recording stop requested')
      list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
      list_output = list_cmd.stdout.read()
      retcode = list_cmd.wait()
      assert retcode == 0, "List command returned %d" % retcode
      for str in list_output.split("\n"):
        if (str.startswith('/'+core_name+'/record')):
          os.system("rosnode kill " + str)
      self._recording = False
    else:
      rospy.logwarn('Stop recording requested, but nobody is recording.')
    return []

  def __init__(self):
    rospy.init_node(NAME)

    self.start_srv = rospy.Service(rospy.get_name() + '/start_recording', RecordRequest, self.start_recording)
    self.stop_srv  = rospy.Service(rospy.get_name() + '/stop_recording', Empty, self.stop_recording)
    rospy.spin()

if __name__ == "__main__":
  if (not os.path.exists(dir_save_bagfiles)):
    os.makedirs(dir_save_bagfiles)
  TopicLogger()