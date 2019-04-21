# CS401 Project 2

**Name**: 徐逸飞（Yifei Xu), 伍凯铭(Kaiming Wu)

**SID**: 11611209, 11611326

### Message

**/camera/rgb/image_color**

![msg_image_color](image/msg_image_color.png)

The message of image_color.



**/camera/depth_registered/image_raw**

![msg_depth](image/msg_depth.png)

The message of image_raw. The reason why they are all zeros is that we set the robot in an open area.



**/camera/rgb/image_mono**

![image_mono](image/msg_image_mono.png)

The message of image_color.



**/odom**

![odom](image/odom.png)

Odom let users get the velocity of the turtlebot at a desired time.



**/tf**

![tf](image/tf.png)

tf is a package that lets the user keep track of multiple coordinate frames over time. It maintains the relationship between coordinate frames in a tree structure buffered in time. Users are able to transform points, vectors, etc between any two coordinate frames at any desired point in time.



**/tf_static**

![tf_static](image/tf_static.png)

Tf_static stores the last movement of a given time duration.



### Camera Image

**/camera/rgb/image_color**

![image_color](image/image_color.png)

The normal rgb data from Kinect.



**/camera/depth_registered/image_raw**

![image_raw](image/image_raw.png)

The raw data from Kinect.



**/camera/rgb/image_mono**

![image_mono](image/image_mono.png)



Monochrome image is generated and only one kind of colors are accepted.