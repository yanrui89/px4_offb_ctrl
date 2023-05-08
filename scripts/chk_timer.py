import rospy

class timer_chker():
    def __init__(self):
        time_now = rospy.Time.now()

    def get_time(self):
        while True:
            print(rospy.Time.now())


if __name__ =="__main__":
    rospy.init_node("chk_timer3")
    time_chk = timer_chker()
    time_chk.get_time()
    rospy.spin()
