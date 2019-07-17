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
"""

PKG = 'utils' # this package name
NAME = 'topic_logger'

import roslib; roslib.load_manifest(PKG)
import rospy
from std_srvs.srv import Empty
from utils.srv import RecordRequest, RecordRequestResponse
import string
import os
import shutil
import sys
import signal
import time
import subprocess

dir_save_bagfiles = '/home/' + os.getlogin() + '/bagfiles'
dirs_save_missions = ['/home/' + os.getlogin() + '/missions',
                      '/var/www/html/robot_missions/missions',
                      '/var/www/html/robotmissions/missions']

# Load Topic list
topics = rospy.get_param("/topic_logger/topics")

image_topics = []
if rospy.has_param('/topic_logger/image_topics'):
  image_topics = rospy.get_param("/topic_logger/image_topics")

def touch(path):
  with open(path, 'a'):
    os.utime(path, None)

def search_file(filename, search_path):
  """Given a search path, find file
  """
  file_found = False
  paths = string.split(search_path, os.pathsep)
  for path in paths:
    for root, dirs, files in os.walk(path):
      for name in files:
        if name == filename:
          if os.path.exists(os.path.join(root, name)):
            file_found = True
            return os.path.abspath(os.path.join(root, name))
            break
  else:
    return None

class TopicLogger:
  _recording = False;

  ##
  ## @brief      Record topics in a bagfile and return the subprocess
  ##
  ## @param      self     The object
  ## @param      topics   The topics
  ## @param      process  The process
  ##
  def record(self, topics, folder, platform, date, split = True, size = 1000):
    args = ''
    if len(sys.argv) > 1:
      args = ' -o ' + folder + "/" + platform + date
    # Normal topics
    for t in topics:
      args = args + " " + t
    command = "rosbag record"
    if split:
      command = command + " --split --size=" + str(size)
    command = command + args
    return subprocess.Popen(command, shell=True, cwd=folder)


  # Start recording the topics
  def start_recording(self, req):
    if (not self._recording):
      rospy.loginfo('Recording start requested')

      dir_save_curr_bagfile = os.path.join(dir_save_bagfiles, req.date)

      if 'gs' in rospy.get_name():
        dir_save_curr_bagfile = os.path.join(dir_save_curr_bagfile, 'gs')

      if not os.path.exists(dir_save_curr_bagfile):
        os.makedirs(dir_save_curr_bagfile)

      # Copy the mission file ONLY in the ground station
      platform = 'auv'
      if 'gs' in rospy.get_name():
        platform = 'gs'
        mission = req.mission
        if not('.yaml' in mission):
          mission = mission + '.yaml'
        if not self.copy_mission(mission, dir_save_curr_bagfile):
          return RecordRequestResponse(False)

      self._recording = True
      self.p = self.record(topics, dir_save_curr_bagfile, platform, req.date, size = 10000)
      rospy.loginfo('Recording process started with PID: {0}'.format(self.p.pid))

      # Image topics
      if image_topics:
        dir_save_img_bagfile = os.path.join(dir_save_curr_bagfile, 'img')
        if not os.path.exists(dir_save_img_bagfile):
          os.makedirs(dir_save_img_bagfile)
        self.p_img = self.record(image_topics, dir_save_img_bagfile, 'images', req.date)
        rospy.loginfo('Recording process started with PID: {0}'.format(self.p_img.pid))

      param_filename = req.date + "_params.yaml"
      rospy.loginfo('Saving parameters in {0}'.format(dir_save_curr_bagfile + '/' + param_filename))
      process = subprocess.Popen('rosparam dump ' + param_filename, shell=True, cwd=dir_save_curr_bagfile)
      time.sleep(10)
      process.kill()
    else:
      rospy.logwarn('Already recording...')
      return RecordRequestResponse(False)
    return RecordRequestResponse(True)

  def copy_mission(self, mission_name, dst_folder):
    """
    @brief      Copy the mission file in the destination folder

    @param      mission_name  The mission name
    @param      dst_folder    The destination folder

    @return     True if the mission file has been found and copied
    """
    if 'rov' in mission_name:
      touch(os.path.join(dst_folder, mission_name))
      return True
    elif 'auv' in mission_name:
      for dir_save_missions in dirs_save_missions:
        mission_file = search_file(mission_name, dir_save_missions)
        if mission_file is not None:
          rospy.loginfo('Mission found at: {0}'.format(mission_file))
          shutil.copy(mission_file, dst_folder)
          return True
      else:
        rospy.logerr('Mission {0} not found. Exiting...'.format(mission_name))
        return False
    else:
      rospy.logerr('Mission name has to be either ROV or AUV! Exiting...')
      return False

  #stop to record the topics
  def stop_recording(self, req):
    if (self._recording):
      rospy.loginfo('Recording stop requested')
      list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
      list_output = list_cmd.stdout.read()
      retcode = list_cmd.wait()
      assert retcode == 0, "List command returned %d" % retcode
      for str in list_output.split("\n"):
        if (str.startswith('/record')):
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
