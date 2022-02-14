#!/usr/bin/env python
import os
import rospy
import paramiko
import std_msgs.msg
from cola2_xiroi.msg import WirelessLink
ssh = paramiko.SSHClient()
msg = WirelessLink()


def wireless_quality(pep):
	hostname = rospy.get_param('host_name', '192.168.1.225')
	username = rospy.get_param('username', 'ubnt')
	password = rospy.get_param('password', 'underwaterpass')
	update_rate = rospy.get_param('~update_rate', 10)	
	pub_wireles_info = rospy.Publisher('network_analysis/wireless_quality', WirelessLink, queue_size=10)
	rate = rospy.Rate(update_rate)

	ssh.load_system_host_keys()
	ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
	ssh.connect(hostname, username=username, password=password)
	print("Connected...")

	while not rospy.is_shutdown():
		msg.header.stamp = rospy.Time.now()
		values = get_info_from_device()

		SSID = values[4]
		SSID = SSID.split (":")
		msg.SSID = SSID[1]

		frequency = values[11]
		frequency = frequency.split (":")
		frequency = frequency[1]
		frequency = frequency.split (" ")
		msg.frequency = float(frequency[0])

		bitrate = values[18]
		bitrate = bitrate.split (":")
		bitrate = bitrate[1]
		bitrate = bitrate.split (" ")
		msg.bitrate = float(bitrate[0])

		tx_power = values[19]
		tx_power = tx_power.split ("=")
		tx_power = tx_power[1]
		tx_power = tx_power.split (" ")
		msg.tx_power = float(tx_power[0])

		link_quality = values[43]
		link_quality = link_quality.split ("=")
		msg.link_quality = link_quality[1]

		signal_level = values[44]
		signal_level = signal_level.split ("=")
		signal_level = signal_level[1]
		signal_level = signal_level.split (" ")
		msg.signal_level = int(signal_level[0])

		noise_level = values[45]
		noise_level = noise_level.split ("=")
		noise_level = noise_level[1]
		noise_level = noise_level.split (" ")
		msg.noise_level = int(noise_level[0])

	  	pub_wireles_info.publish(msg)
    		rate.sleep()

def get_info_from_device():

	try:

		cmd ="iwconfig" 
		stdin, stdout, stderr = ssh.exec_command(cmd)
		out = stdout.read().strip()
		return  out.split("  ")

	except:
    		rospy.loginfo("The specified interface does not exist or is disconnected. Please check it")
		pass

if __name__ == '__main__':

	try:
		rospy.init_node('wireless_quality')
		wireless_quality = wireless_quality(rospy.get_name())
		rospy.spin()
	except rospy.ROSInterruptException:
		pass




