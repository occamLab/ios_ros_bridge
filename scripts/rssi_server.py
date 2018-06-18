#!/usr/bin/env python

from udp import UDPhandle
import rospy
from std_msgs.msg import String, Float64, Header
from ios_streamer.msg import Rf_stamped

ts = 0

def most_recent_ts(msg):
    global ts
    ts = float(msg.data)


rospy.init_node("RSSI")

pub = rospy.Publisher("RSSI", Rf_stamped, queue_size=10)
ios_time = rospy.Subscriber("/ios_clock", Float64, most_recent_ts)

port = 11118

start_delim = "RSSISTART\n"
end_delim = "RSSIEND\n"


@UDPhandle(port=port, start_delim=start_delim, end_delim=end_delim)
def handle_pkt(pkt=None):
    global ts
    ios_time = float(pkt[pkt.find("RSSISCANTIMESTART") + len("RSSISCANTIMESTART"):pkt.find("RSSISCANTIMEEND")])
    print "ios_time", ios_time
    pkt = pkt[pkt.find("RSSISCANTIMEEND")+len("RSSISCANTIMEEND"):]
    out = Rf_stamped(header=Header(stamp=rospy.Time(ios_time + ts), frame_id="device"), data=str(pkt))
    pub.publish(out)

handle_pkt()
