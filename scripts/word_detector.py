#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
import rospy
import speech_recognition as SR

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from respeaker_ros.msg import StampedAudio
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import ColorRGBA

keywords=[
("stand up", .98),
("stand", 1.16),
("up", 1.21),

("sit down", 1.0),
("sit up", .99),
("sit", 1.15),

("lie down", 1.0),
("down", 1.2),

("k dog", .86),
("hello", 1.1),
("hi there", .99)
]

class SpeechToText(object):
    def __init__(self):
        # format of input audio data
        self.sample_rate = rospy.get_param("~sample_rate", 16000)
        self.sample_width = rospy.get_param("~sample_width", 2)
        # language of STT service
        self.language = rospy.get_param("~language", "en-US")
        
        self.pub_led = rospy.Publisher("status_led", ColorRGBA, queue_size=10)
        self.pub_speech = rospy.Publisher(
            "speech_to_text", SpeechRecognitionCandidates, queue_size=1)
        self.sub_audio = rospy.Subscriber("speech_audio", StampedAudio, self.audio_cb, queue_size=2)

        self.recognizer = SR.Recognizer()

    def audio_cb(self, msg):

        if rospy.get_rostime() - msg.stamp > rospy.Duration(2):
            rospy.loginfo("Old speech discarded")
            return

        data = SR.AudioData(bytes(msg.data), self.sample_rate, self.sample_width)
        with open(str(len(msg.data)) + ".wav","wb") as f:
            f.write(data.get_wav_data())

        try:
            rospy.loginfo("Waiting for result %d" % len(data.get_raw_data()))
            result = self.recognizer.recognize_sphinx(
                data, language=self.language, show_all=False, keyword_entries=keywords)
            rospy.loginfo(result)

            msg = SpeechRecognitionCandidates(transcript=result)
            self.pub_speech.publish(msg)

        except SR.UnknownValueError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
            rospy.loginfo("value error")
            #self.pub_led.publish(1,0,0,1)
        except SR.RequestError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
            rospy.loginfo("request error")
            #self.pub_led.publish(1,0,0,1)


if __name__ == '__main__':
    rospy.init_node("speech_to_text")
    stt = SpeechToText()
    rospy.spin()
