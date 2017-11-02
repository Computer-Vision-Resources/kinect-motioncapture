/*

Processing code to obtain joint angles and write to server

Joint angles are obtained in a 4x4 matrix using simpleopenni and then converted to quaternion
Quaternion data is written to server in packets
Confidence value hasn't been made use of as of now


*/

import processing.net.*;
Client myClient;

import SimpleOpenNI.*;
SimpleOpenNI  kinect;

PMatrix3D torso_orientation;
PMatrix3D left_elbow_orientation;
PMatrix3D left_shoulder_orientation;
PMatrix3D right_elbow_orientation;
PMatrix3D right_shoulder_orientation;
PMatrix3D left_knee_orientation;
PMatrix3D left_hip_orientation;
PMatrix3D right_knee_orientation;
PMatrix3D right_hip_orientation;


float torso_confidence;
float left_shoulder_confidence;
float left_elbow_confidence;
float right_shoulder_confidence;
float right_elbow_confidence;
float left_knee_confidence;
float left_hip_confidence;
float right_knee_confidence;
float right_hip_confidence;


void setup()
{
  
  myClient = new Client(this, "127.0.0.1", 5049);
  
  /*if(!myClient.active())
  {
    while(!myClient.active())
    {
      myClient = new Client(this, "127.0.0.1", 5008);
    }
  }
  */
  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableRGB();
  kinect.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
  
  torso_orientation = new PMatrix3D();
  left_elbow_orientation = new PMatrix3D();
  left_shoulder_orientation = new PMatrix3D();
  right_elbow_orientation = new PMatrix3D();
  right_shoulder_orientation = new PMatrix3D();
  left_knee_orientation = new PMatrix3D();
  left_hip_orientation = new PMatrix3D();
  right_knee_orientation = new PMatrix3D();
  right_hip_orientation = new PMatrix3D();

  size(640, 960);
  fill(255, 0, 0);
}

void draw() 
{
  kinect.update();
  image(kinect.depthImage(), 0, 0);
  image(kinect.rgbImage(),0,480);

  IntVector userList = new IntVector();
  kinect.getUsers(userList);

  if (userList.size() > 0) {
    int userId = userList.get(0);

    if (kinect.isTrackingSkeleton(userId)) 
    {
      
      updateAngles(userId);
      
      // torso_orientation.print();
      
      quat_write(torso_orientation, 0);
      quat_write(left_shoulder_orientation, 1);
      quat_write(left_elbow_orientation, 2);
      quat_write(right_shoulder_orientation, 3);
      quat_write(right_elbow_orientation, 4);
      quat_write(left_hip_orientation, 5);
      quat_write(left_knee_orientation, 6);
      quat_write(right_hip_orientation, 7);
      quat_write(right_knee_orientation, 8);
      
    }
  }
}

void quat_write(PMatrix3D orientation, int id)
{
  float tr = orientation.m00 + orientation.m11 + orientation.m22;
  
  float S, qw, qx, qy, qz;
  S = qw = qx = qy = qz = 0;
  
  if (tr > 0) 
  { 
    S = sqrt(tr+1.0) * 2; // S=4*qw 
    qw = 0.25 * S;
    qx = (orientation.m21 - orientation.m12) / S;
    qy = (orientation.m02 - orientation.m20) / S; 
    qz = (orientation.m10 - orientation.m01) / S; 
  }
  
  else if ((orientation.m00 > orientation.m11)&(orientation.m00 > orientation.m22))
  { 
    S = sqrt(1.0 + orientation.m00 - orientation.m11 - orientation.m22) * 2; // S=4*qx 
    qw = (orientation.m21 - orientation.m12) / S;
    qx = 0.25 * S;
    qy = (orientation.m01 + orientation.m10) / S; 
    qz = (orientation.m02 + orientation.m20) / S; 
  }
  
  else if (orientation.m11 > orientation.m22) 
  { 
    S = sqrt(1.0 + orientation.m11 - orientation.m00 - orientation.m22) * 2; // S=4*qy
    qw = (orientation.m02 - orientation.m20) / S;
    qx = (orientation.m01 + orientation.m10) / S; 
    qy = 0.25 * S;
    qz = (orientation.m12 + orientation.m21) / S; 
  }
  
  else
  { 
    S = sqrt(1.0 + orientation.m22 - orientation.m00 - orientation.m11) * 2; // S=4*qz
    qw = (orientation.m10 - orientation.m01) / S;
    qx = (orientation.m02 + orientation.m20) / S;
    qy = (orientation.m12 + orientation.m21) / S;
    qz = 0.25 * S;

  }
  
  myClient.write(id+" "+qw+" "+qx+" "+qy+" "+qz+" ; ");
  delay(1);
}

void updateAngles(int userId)
{
  torso_confidence = kinect.getJointOrientationSkeleton(userId, SimpleOpenNI.SKEL_TORSO, torso_orientation);
  left_shoulder_confidence = kinect.getJointOrientationSkeleton(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, left_shoulder_orientation);
  left_elbow_confidence = kinect.getJointOrientationSkeleton(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, left_elbow_orientation);
  right_shoulder_confidence = kinect.getJointOrientationSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, right_shoulder_orientation);
  right_elbow_confidence = kinect.getJointOrientationSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, right_elbow_orientation);
  left_knee_confidence = kinect.getJointOrientationSkeleton(userId, SimpleOpenNI.SKEL_LEFT_KNEE, left_knee_orientation);
  left_hip_confidence = kinect.getJointOrientationSkeleton(userId, SimpleOpenNI.SKEL_LEFT_HIP, left_hip_orientation);
  right_hip_confidence = kinect.getJointOrientationSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_HIP, right_hip_orientation);
  right_knee_confidence = kinect.getJointOrientationSkeleton(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, right_knee_orientation);
  
}
void onNewUser(int userId) 
{
  kinect.startTrackingSkeleton(userId);
}


