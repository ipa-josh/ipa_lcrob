#!/usr/bin/python

import rosbag, os, math, argparse, tf
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

#settings
min_range = 0.1
max_range = 2.5
angle_factor = 4
min_angle = -1.5
max_angle =  1.5
deviation = 0.05
odom_noise= 0.0
img_noise = 0.0
noises=[]
parser = argparse.ArgumentParser(description='')

parser.add_argument('fn', type=str)
parser.add_argument('--min_range', type=float)
parser.add_argument('--max_range', type=float)
parser.add_argument('--angle_factor', type=int)
parser.add_argument('--min_angle', type=float)
parser.add_argument('--max_angle', type=float)
parser.add_argument('--deviation', type=float)
parser.add_argument('--odom_noise', type=float)
parser.add_argument('--img_noise', type=float)

args = parser.parse_args()

if args.min_range!=None:
	min_range = args.min_range
if args.max_range!=None:
	max_range = args.max_range
if args.angle_factor!=None:
	angle_factor = args.angle_factor
if args.min_angle!=None:
	min_angle = args.min_angle
if args.max_angle!=None:
	max_angle = args.max_angle
if args.deviation!=None:
	deviation = args.deviation
if args.odom_noise!=None:
	odom_noise = args.odom_noise
if args.img_noise!=None:
	img_noise = args.img_noise

min_angle+=math.pi/2
max_angle+=math.pi/2

def simulate_odom(msg):
	global odom_noise
	
	if odom_noise<=0: return msg
	
	msg_out = Odometry()
	msg_out.header = msg.header
	msg_out.child_frame_id = msg.child_frame_id
	msg_out.pose = msg.pose
	msg_out.pose.covariance  = msg.pose.covariance
	msg_out.twist.covariance = msg.twist.covariance
	
	noise = np.random.normal(0, odom_noise, 6)
	
	msg_out.twist.twist.linear.x = msg.twist.twist.linear.x*(noise[0]+1)
	msg_out.twist.twist.linear.y = msg.twist.twist.linear.y*(noise[1]+1)
	msg_out.twist.twist.linear.z = msg.twist.twist.linear.z*(noise[2]+1)
	
	msg_out.twist.twist.angular.x = msg.twist.twist.angular.x*(noise[3]+1)
	msg_out.twist.twist.angular.y = msg.twist.twist.angular.y*(noise[4]+1)
	msg_out.twist.twist.angular.z = msg.twist.twist.angular.z*(noise[5]+1)
	

	return msg_out
	
def noisy(noise_typ,image,img_noise):
   if noise_typ == "gauss":
      row,col,ch= image.shape
      mean = 0
      var = 0.1*img_noise
      sigma = var**0.5
      gauss = (np.random.normal(mean,sigma,(row,col,ch))*256).astype(np.uint8)
      #print gauss
      gauss = gauss.reshape(row,col,ch)
      noisy = image + gauss
      return noisy
   elif noise_typ == "s&p":
      row,col,ch = image.shape
      s_vs_p = 0.5
      amount = 0.004
      out = image
      # Salt mode
      num_salt = np.ceil(amount * image.size * s_vs_p)
      coords = [np.random.randint(0, i - 1, int(num_salt))
              for i in image.shape]
      out[coords] = 1

      # Pepper mode
      num_pepper = np.ceil(amount* image.size * (1. - s_vs_p))
      coords = [np.random.randint(0, i - 1, int(num_pepper))
              for i in image.shape]
      out[coords] = 0
      return out
   elif noise_typ == "poisson":
      vals = len(np.unique(image))
      vals = 2 ** np.ceil(np.log2(vals))
      noisy = np.random.poisson(image * vals) / float(vals)
      return noisy
   elif noise_typ =="speckle":
      row,col,ch = image.shape
      gauss = np.random.randn(row,col,ch)
      gauss = gauss.reshape(row,col,ch)        
      noisy = image + image * gauss
      return noisy
      
def simulate_img(img):
    global img_noise
    
    if img_noise<=0: return img
    
    try:
      bridge = CvBridge()
      cv_image = bridge.imgmsg_to_cv2(img, img.encoding)
      cv_image = noisy("gauss", cv_image, img_noise)
      msg = bridge.cv2_to_imgmsg(cv_image, img.encoding)
      msg.header = img.header
      return msg
    except CvBridgeError as e:
      print(e)
    
    return img #backup strategy

def simulate_sensor(scan_in):
	global min_range,max_range,angle_factor,min_angle,max_angle,deviation,noises
	
	scan_out = LaserScan()
	scan_out.header = scan_in.header
	scan_out.angle_min = scan_in.angle_max
	scan_out.angle_max = scan_in.angle_min
	scan_out.angle_increment = scan_in.angle_increment*angle_factor
	scan_out.time_increment = scan_in.time_increment
	scan_out.scan_time = scan_in.scan_time
	scan_out.range_min = min_range
	scan_out.range_max = max_range
	#scan_out.ranges = scan_in.ranges[:]
	#scan_out.intensities = scan_in.intensities[:]
	
	for i in xrange(len(scan_in.ranges)):
		a = i*scan_in.angle_increment+scan_in.angle_min
		
		if a<min_angle or a>max_angle or i%angle_factor!=0: continue
		
		scan_out.angle_min = min(scan_out.angle_min, a)
		scan_out.angle_max = max(scan_out.angle_max, a)
		
		noise = 0
		if scan_in.ranges[i]!=np.nan and scan_in.ranges[i]!=np.inf and scan_in.ranges[i]!=-np.inf:
			if deviation>0:
				noise = np.random.normal(0, math.pow(1+scan_in.ranges[i], 2)*deviation,1)[0]
			
			d = scan_in.ranges[i] + noise
			if d>max_range or d<min_range:
				d = np.inf
			else:
				noises.append(noise)
				
			scan_out.ranges.append( d )
		else:
			scan_out.ranges.append( scan_in.ranges[i] )
		scan_out.intensities.append( scan_in.intensities[i] )
		
	return scan_out
	
fn = args.fn

bagin = rosbag.Bag(fn)
bag   = rosbag.Bag(os.path.splitext(fn)[0]+".out.bag", 'w')
desc  = open(os.path.splitext(fn)[0]+".desc.txt", "w")

try:
    desc.write("min_range"+"\t"+str(min_range)+"\n")
    desc.write("max_range"+"\t"+str(max_range)+"\n")
    desc.write("angle_factor"+"\t"+str(angle_factor)+"\n")
    desc.write("min_angle"+"\t"+str(min_angle)+"\n")
    desc.write("max_angle"+"\t"+str(max_angle)+"\n")
    desc.write("deviation"+"\t"+str(deviation)+"\n")
    desc.write("odom_noise"+"\t"+str(odom_noise)+"\n")
    desc.write("img_noise"+"\t"+str(img_noise)+"\n")
    desc.close()

    n=0
    for topic, msg, t in bagin.read_messages():
        if topic=="/scan":
            bag.write("/scan_old", msg, t)
            bag.write(topic, simulate_sensor(msg), t)
        elif topic=="/odom":
            bag.write("/odom_old", msg, t)
            bag.write(topic, simulate_odom(msg), t)
        elif topic=="/usb_cam/image_raw":
            bag.write(topic+"_old", msg, t)
            bag.write(topic, simulate_img(msg), t)
        else:
            bag.write(topic, msg, t)
            
        #if n>10000: break
        n+=1
finally:
    bag.close()
    bagin.close()
    print "noise statistics: ",np.mean(noises),np.std(noises)
