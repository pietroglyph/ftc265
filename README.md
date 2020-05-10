# ftc265

ftc265 is a plug-and-play Intel RealSense T265 VSLAM camera wrapper for FTC based off my work in FRC writing a [roboRIO T265 wrapper](https://github.com/Spartronics4915/SpartronicsLib).

## Installation
Paste the following into `TeamCode/build.gradle`:

```gradle
repositories {
    jcenter()

    maven {
        url "https://maven.0x778.tk"
    }
}

dependencies {
    implementation 'com.spartronics4915.lib:ftc265:1.0.0'
}
```

You'll need to perform a Gradle sync in Android studio (if you use that IDE) after adding the new repositories and dependencies.

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
    currentPose = camUpdate.pose;
  }
});

// Meanwhile we can grab the pose variable whenever we want in our main thread
while (true) {
    synchronized (currentPose) {
        System.out.println(currentPose);
    }
}
```

There is also a ready-to-use example project [here](https://github.com/pietroglyph/ftc_app/tree/ftc265_template).

## FAQs

### Is this legal?
According to [this GDC clarification](https://ftcforum.firstinspires.org//forum/first-tech-challenge-skystone-presented-by-qualcomm-game-q-a-forum/robot-inspection-and-build-rules-aa/answers-raw-and-post-processed-materials/74292-sensors?p=75207#post75207), yes.

If that doesn't convince you, please take the following into account:
 1. In the opinion of the author, this is no more of a coprocessor than a servo. The T265 has an ASIC just like a servo might have a microcontroller, but critically, neither of these are reprogrammable. Similar devices that do offboard nonprogrammable video processing have been allowed (e.g. the Pixy cam, which was cited in the GDC clarification linked above.)
 2. This fits under the broad category of UVC-compatible devices.

### Is the T265 accurate in an FTC context?
In my testing, yes. I used this in-season at (more demanding) FRC speeds, and when testing on an FTC-field-sized area I consistiently got less than 1 inch of error.

### Is this compatible with the REV Control Hub?
I don't know. On normal Android devices the user must allow access to the T265 by tappong on onscreen prompts, which might present a problem with the control hub. Please contact me if you've tested on the Control Hub or if you're interested in testing (I am pietroglyph#9445 on the FTC and FRC Discords.)

### Does this support importing and exporting relocalization maps?
Yes.
