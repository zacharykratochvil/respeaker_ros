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


class SpeechToText(object):
    def __init__(self):
        # format of input audio data
        self.sample_rate = 16000 #rospy.get_param("~sample_rate", 16000)
        self.sample_width = 2 #rospy.get_param("~sample_width", 2)
        # language of STT service
        self.language = rospy.get_param("~language", "en-US")
        # ignore voice input while the robot is speaking
        self.self_cancellation = rospy.get_param("~self_cancellation", True)
        # time to assume as SPEAKING after tts service is finished
        self.tts_tolerance = rospy.Duration.from_sec(
            rospy.get_param("~tts_tolerance", 1.0))

        self.recognizer = SR.Recognizer()

        self.tts_action = None
        self.last_tts = None
        self.is_canceling = False
        if self.self_cancellation:
            self.tts_action = actionlib.SimpleActionClient(
                "sound_play", SoundRequestAction)
            if self.tts_action.wait_for_server(rospy.Duration(5.0)):
                self.tts_timer = rospy.Timer(rospy.Duration(0.1), self.tts_timer_cb)
            else:
                rospy.logerr("action '%s' is not initialized." % rospy.remap_name("sound_play"))
                self.tts_action = None

        self.pub_speech = rospy.Publisher(
            "speech_to_text", SpeechRecognitionCandidates, queue_size=2)
        self.sub_audio = rospy.Subscriber("speech_audio", StampedAudio, self.audio_cb, queue_size=2)

    def tts_timer_cb(self, event):
        stamp = event.current_real
        active = False
        for st in self.tts_action.action_client.last_status_msg.status_list:
            if st.status == GoalStatus.ACTIVE:
                active = True
                break
        if active:
            if not self.is_canceling:
                rospy.logdebug("START CANCELLATION")
                self.is_canceling = True
                self.last_tts = None
        elif self.is_canceling:
            if self.last_tts is None:
                self.last_tts = stamp
            if stamp - self.last_tts > self.tts_tolerance:
                rospy.logdebug("END CANCELLATION")
                self.is_canceling = Falser

    def audio_cb(self, msg):

        if self.is_canceling:
            rospy.loginfo("Speech is cancelled")
            return
        if rospy.get_rostime().secs - msg.stamp.secs > 2:
            rospy.loginfo("Old speech discarded")
            return

        data = SR.AudioData(bytes(msg.data), self.sample_rate, self.sample_width)
        with open(str(len(msg.data)) + ".wav","wb") as f:
            f.write(data.get_wav_data())

        try:
            rospy.loginfo("Waiting for result %d" % len(data.get_raw_data()))
            result = self.recognizer.recognize_sphinx(
                data, language=self.language, show_all=False) #, key=None
            rospy.loginfo(result)

            msg = SpeechRecognitionCandidates(transcript=result)
            self.pub_speech.publish(msg)

        except SR.UnknownValueError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
            rospy.loginfo("value error")
        except SR.RequestError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
            rospy.loginfo("request error")


if __name__ == '__main__':
    rospy.init_node("speech_to_text")
    stt = SpeechToText()
    rospy.spin()
