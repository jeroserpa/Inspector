import rospy
from sensor_msgs.msg import Imu



 
def translator():
    def callback_acc(data):
     #print(data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z)


        xn=data.linear_acceleration.z
        yn=data.linear_acceleration.x
        zn=data.linear_acceleration.y
        accel = Imu()
        accel.header = data.header
        accel.linear_acceleration.x = xn
        accel.linear_acceleration.y = yn
        accel.linear_acceleration.z = zn
        accel.linear_acceleration_covariance = data.linear_acceleration_covariance
        
        pub_accel.publish(accel)
       
    
    def callback_gyro(data):
     #print(data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z)


        vxn=data.angular_velocity.z
        vyn=data.angular_velocity.x
        vzn=data.angular_velocity.y
        gyro = Imu()
        gyro.header = data.header
        gyro.angular_velocity.x = vxn
        gyro.angular_velocity.y = vyn
        gyro.angular_velocity.z = vzn
        gyro.angular_velocity_covariance = data.angular_velocity_covariance
        
        pub_gyro.publish(gyro)
        #print(gyro)



    rospy.init_node('translator', anonymous=True) #init node
    rospy.Subscriber("t265/accel/sample", Imu, callback_acc) #subscribe to accel topic
    rospy.Subscriber("t265/gyro/sample", Imu, callback_gyro) #subscribe to gyro topic
    
    pub_accel = rospy.Publisher('t265/accel/rotated', Imu, queue_size=10) #init publisher of corrected accel 
    pub_gyro = rospy.Publisher('t265/gyro/rotated', Imu, queue_size=10) #init publisher of corrected gyro

    rospy.spin()

    



if __name__ == '__main__':
    translator()