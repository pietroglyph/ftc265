# ftc265

ftc265 is a plug-and-play Intel RealSense T265 VSLAM camera wrapper for FTC based off my work in FRC writing a [roboRIO T265 wrapper](https://github.com/Spartronics4915/SpartronicsLib).

## Installation
Usage is as easy as downloading the [latest release's AAR file](https://0x778.tk/ftc265-release.aar) and adding it to your Android Studio project.

A dependency will be available from JCenter soon.

## Usage
Basic usage is as follows:

```java
// This is the transformation between the center of the camera and the center of the robot
Transform2d cameraToRobot = new Transform2d();
double encoderMeasurementCovariance = 0.8; // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());

T265Camera slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance);
slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin

// Call this when you're ready to get camera updates
Pose2d currentPose = startingPose;
slamra.start((T265Camera.CameraUpdate camUpdate) -> {
  // This is the callback for every camera pose update
  // **This lambda is called from another thread. You must synchronize cross thread memory accesses!**

  // We'll set a variable that can be accessed by other code
  // "synchronized" is very important. Do not remove it (see above warning.)
  synchronized (currentPose) {
    myPose = camUpdate.pose;
  }
});

// Meanwhile we can grab the pose variable whenever we want in our main thread
while (true) {
    synchronized (currentPose) {
        System.out.println(currentPose);
    }
}
```