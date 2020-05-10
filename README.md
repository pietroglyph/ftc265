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
    // This will get the latest compatible version
    implementation 'com.spartronics4915.lib:ftc265:1.+'
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

### Why use VSLAM?
 1. The T265 is smaller and easier to mount and maintain than sprung odometry wheels. It is also likely as accurate.
 2. Awards.

### Is this compatible with the REV Control Hub?
I don't know. On normal Android devices the user must allow access to the T265 by tappong on onscreen prompts, which might present a problem with the control hub. Please contact me if you've tested on the Control Hub or if you're interested in testing (I am pietroglyph#9445 on the FTC and FRC Discords.)

### Does this support importing and exporting relocalization maps?
Yes.

### Why do you use FTCLib geometry classes instead of library X's geometry classes?
Unfortunately, there is no canonical set of geometry classes (`Pose2d`, `Transform2d`, etc.) in FTC so I just had to pick a set to use. I prefer the FTCLib geometry classes because they're directly copied from WPILib (which I'm quite familiar with.) I also think that these WPILib-style geometry classes are better than the alternatives.

### How did you make this work?
I had to write JNI bindings to C++ librealsense because no Java bindings exist for the T265. Then I had to patch librealsense to get around some bugs and Android incompatibilities (especially related to the maximum size for USB bulk transfers and the list of USB VID/PID combos); this is why I distribute a custom version of librealsense.

### Can I contribute?
Yes. Please report bugs or (even better) make a PR. FTC is not my main focus, and I will soon lose access to my team's T265 camera, so I would appreciate it if any nontrivial PRs are accompanied by a test on real hardware.

### Will you support this into the future?
This is contingent on the community's response and Intel's actions. If ftc265 becomes popular then I will continue to support it as best I can into the future until Intel provides a reasonable alternative. Please note that I have limited time and that FTC is not my main focus; I will endevor to fix issues and respond to you as fast as possible, but expect the standard deviation on my response time to be high :)

## License

Copyright 2020 Declan Freeman-Gleason

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this program.  If not, see <https://www.gnu.org/licenses/>.
