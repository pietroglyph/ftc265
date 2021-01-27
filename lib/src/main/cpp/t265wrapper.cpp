// Copyright 2020-2021 Declan Freeman-Gleason
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

#include "t265wrapper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <thread>
#include <vector>

#include <android/log.h>

// We use jlongs like pointers, so they better be large enough
static_assert(sizeof(jlong) >= sizeof(void *));

// Constants
constexpr auto originNodeName = "origin";
constexpr auto exportRelocMapStopDelay = std::chrono::seconds(10);
constexpr auto logTag = "ftc265";

// We cache all of these because we can
jclass holdingClass = nullptr; // This should always be the T265Camera jclass
jfieldID handleFieldId =
    nullptr; // Field id for the field "nativeCameraObjectPointer"
jclass exception = nullptr; // This is "CameraJNIException"

std::vector<deviceAndSensors *> toBeCleaned;
std::mutex cleanupMutex;

/*
 * A note on coordinate systems:
 *
 * Relative to the robot, the camera's Y is up and down, Z is forward and back,
 * and X is side to side.
 *
 * Conversions are:
 *   Camera Z -> Robot X * -1
 *   Camera X -> Robot Y * -1
 *   Camera Quaternion -> Counter-clockwise Euler angles (currently just yaw)
 *
 * For ease of use, all coordinates that come out of this wrapper are in this
 * "robot standard" coordinate system.
 */

// We do this so we don't have to fiddle with files
// Most of the below fields are supposed to be "ignored"
// See
// https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#wheel-odometry-calibration-file-format
constexpr auto odometryConfig = R"(
{
    "velocimeters": [
        {
            "scale_and_alignment": [
                1.0,
                0.0000000000000000,
                0.0000000000000000,
                0.0000000000000000,
                1.0,
                0.0000000000000000,
                0.0000000000000000,
                0.0000000000000000,
                1.0
            ],
            "noise_variance": %f,
            "extrinsics": {
                "T": [
                    %f,
                    0.0,
                    %f
                ],
                "T_variance": [
                    9.999999974752427e-7, 
                    9.999999974752427e-7, 
                    9.999999974752427e-7
                ],
                "W": [
                    0.0,
                    %f,
                    0.0
                ],
                "W_variance": [
                    9.999999974752427e-5, 
                    9.999999974752427e-5, 
                    9.999999974752427e-5
                ]
            }
        }
    ]
}
)";

extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void * /*reserved*/) {
  JNIEnv *env;
  if (vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) != JNI_OK) {
    __android_log_print(ANDROID_LOG_ERROR, logTag,
                        "Native code initialization failed! Wrong JNI version");

    return JNI_ERR;
  }

  __android_log_print(ANDROID_LOG_INFO, logTag, "Native code initialized");
  return JNI_VERSION_1_6;
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_spartronics4915_lib_T265Camera_newCamera(JNIEnv *env, jobject thisObj,
                                                  jstring mapPath) {
  // This locking scheme is very conservative. This is important. When the T265
  // is attached from a cold boot, it will first present as a Movidius device,
  // which is handled on the Java end and ultimately gets passed to this code to
  // be initialized. As we initialize it the camera appears to reboot (?),
  // disconnect, and then reconnect with the T265 VID/PID. If we do not protect
  // the entire init process *and* free process with a mutex then we'll end up
  // free'ing this camera while we're still initializing. This global mutex also
  // protects us from initialzing two cameras at once. Initialzing two cameras
  // at once isn't theoretically a problem if they aren't the same USB device,
  // but we don't discriminate and it's possible that the same USB device would
  // be picked by both inits. Ultimately, forcing inits and frees to run in
  // serial *globally* is in our best interest.
  std::scoped_lock lk(cleanupMutex);

  try {
    rs2::context ctx;

    ensureCache(env, thisObj);

    deviceAndSensors *devAndSensors = nullptr;
    try {
      devAndSensors = getDeviceFromClass(env, thisObj);
    } catch (std::runtime_error &) {
      // Ignore the exception and check manually below
    }
    if (devAndSensors && devAndSensors->isRunning)
      throw std::runtime_error("Can't make a new camera if the calling class "
                               "already has one (you need to call free first)");

    auto pipeline = new rs2::pipeline();

    // Set up a config to ensure we only get tracking capable devices
    auto config = rs2::config();
    config.disable_all_streams();
    config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    // Get the currently used device
    auto profile = config.resolve(*pipeline);
    auto device = profile.get_device();
    if (!device.is<rs2::tm2>()) {
      throw std::runtime_error(
          "The device you have plugged in is not tracking-capable");
    }

    // Get the odometry/pose sensors
    // For the T265 both odom and pose will be from the *same* sensor
    rs2::wheel_odometer *odom = nullptr;
    rs2::pose_sensor *pose = nullptr;
    for (const auto &sensor : device.query_sensors()) {
      if (sensor.is<rs2::wheel_odometer>())
        pose = new rs2::pose_sensor(sensor);
      if (sensor.is<rs2::pose_sensor>())
        odom = new rs2::wheel_odometer(sensor);
    }
    if (!odom)
      throw std::runtime_error(
          "Selected device does not support wheel odometry inputs");
    if (!pose)
      throw std::runtime_error("Selected device does not have a pose sensor");

    // Import the relocalization map, if the path is nonempty
    auto pathNativeStr = env->GetStringUTFChars(mapPath, 0);
    if (std::strlen(pathNativeStr) > 0)
      importRelocalizationMap(pathNativeStr, pose);
    env->ReleaseStringUTFChars(mapPath, pathNativeStr);

    /*
     * We define a callback that will run in another thread.
     * This is why we must take care to preserve a pointer to
     * a jvm object, which we attach to the other thread
     * so we can get a valid environment object and call the
     * callback in Java. We also make a global reference to
     * the current object, which will be cleaned up when the
     * user calls free() from Java.
     */
    JavaVM *jvm;
    int error = env->GetJavaVM(&jvm);
    if (error)
      throw std::runtime_error(
          "Couldn't get a JavaVM object from the current JNI environment");
    auto globalThis = env->NewGlobalRef(thisObj); // Must be cleaned up later

    // Need to new because the lifetime of this object is controlled by Java
    // code and we set a field in the Java class to this pointer
    devAndSensors = new deviceAndSensors(pipeline, odom, pose, globalThis);

    auto consumerCallback = [jvm, devAndSensors](const rs2::frame &frame) {
      JNIEnv *env = nullptr;
      try {
        // Attaching the thread is expensive and happens *every callback*...
        // TODO: Cache env?
        int error = jvm->AttachCurrentThread(&env, nullptr);
        if (error)
          throw std::runtime_error("Couldn't attach callback thread to jvm");

        auto poseData = frame.as<rs2::pose_frame>().get_pose_data();
        // rotation is a quaternion so we must convert to an euler angle (yaw)
        auto yaw = 2 * atan2f(poseData.rotation.y, poseData.rotation.w);

        auto callbackMethodID =
            env->GetMethodID(holdingClass, "consumePoseUpdate", "(FFFFFFI)V");
        if (!callbackMethodID)
          throw std::runtime_error("consumePoseUpdate method doesn't exist");

        auto velocityMagnitude =
            hypotf(-poseData.velocity.z, -poseData.velocity.x);
        env->CallVoidMethod(devAndSensors->globalThis, callbackMethodID,
                            -poseData.translation.z, -poseData.translation.x,
                            yaw, -poseData.velocity.z, -poseData.velocity.x,
                            poseData.angular_velocity.y,
                            poseData.tracker_confidence);

        std::scoped_lock lk(devAndSensors->frameNumMutex);
        devAndSensors->lastRecvdFrameNum = frame.get_frame_number();
      } catch (std::exception &e) {
        /*
         * Unfortunately if we get an exception while attaching the thread
         * we can't throw into Java code, so we'll just print to stderr
         */
        if (env)
          env->ThrowNew(exception, e.what());
        else
          __android_log_print(ANDROID_LOG_ERROR, logTag,
                              "Exception in frame consumer callback could not "
                              "be thrown in Java code. Exception was: %s",
                              e.what());
      }
    };

    // Start streaming
    pipeline->start(config, consumerCallback);

    toBeCleaned.push_back(devAndSensors);

    __android_log_print(ANDROID_LOG_INFO, logTag,
                        "Successfully initialized tracking device");

    return reinterpret_cast<jlong>(devAndSensors);
  } catch (std::exception &e) {
    ensureCache(env, thisObj);
    env->ThrowNew(exception, e.what());
    return 0;
  }
  return 0;
}

extern "C" JNIEXPORT void JNICALL
Java_com_spartronics4915_lib_T265Camera_sendOdometryRaw(
    JNIEnv *env, jobject thisObj, jint sensorId, jfloat xVel, jfloat yVel) {
  try {
    ensureCache(env, thisObj);

    auto devAndSensors = getDeviceFromClass(env, thisObj);
    if (!devAndSensors->isRunning)
      return;
    // throw std::runtime_error("Can't send odometry data when the device isn't
    // running");

    // jints are 32 bit and are signed so we have to be careful
    if (sensorId > UINT8_MAX || sensorId < 0)
      env->ThrowNew(exception,
                    "sensorId is out of range of a 8-bit unsigned integer");

    std::scoped_lock lk(devAndSensors->frameNumMutex);
    devAndSensors->wheelOdometrySensor->send_wheel_odometry(
        sensorId, devAndSensors->lastRecvdFrameNum,
        rs2_vector{.x = -yVel, .y = 0.0, .z = -xVel});
  } catch (std::exception &e) {
    env->ThrowNew(exception, e.what());
  }
}

extern "C" JNIEXPORT void JNICALL
Java_com_spartronics4915_lib_T265Camera_exportRelocalizationMap(
    JNIEnv *env, jobject thisObj, jstring savePath) {
  try {
    ensureCache(env, thisObj);

    auto pathNativeStr = env->GetStringUTFChars(savePath, 0);

    // Open file in binary mode
    auto file = std::ofstream(pathNativeStr, std::ios::binary);
    if (!file || file.bad())
      throw std::runtime_error(
          "Couldn't open file to write a relocalization map");

    // Get data from sensor and write
    auto devAndSensors = getDeviceFromClass(env, thisObj);
    if (!devAndSensors->isRunning)
      throw std::runtime_error(
          "Can't export a relocalization map when the device isn't running "
          "(have you already exported?)");

    // We can't export while the camera is running
    devAndSensors->pipeline->stop();
    devAndSensors->isRunning = false;

    // I know, this is really gross...
    // Unfortunately there is apparently no way to figure out if we're ready to
    // export the map. See:
    // https://github.com/IntelRealSense/librealsense/issues/4024#issuecomment-494258285
    std::this_thread::sleep_for(exportRelocMapStopDelay);

    // set_static_node fails if the confidence is not High... We don't currently
    // use the static node, but you really probably shouldn't be exporting your
    // map if your confidence isn't high anyway.
    auto success = devAndSensors->poseSensor->set_static_node(
        originNodeName, rs2_vector{0, 0, 0}, rs2_quaternion{0, 0, 0, 1});
    if (!success)
      throw std::runtime_error(
          "Couldn't set static node while exporting a relocalization map... "
          "Your confidence must be \"High\" for this to work.");

    auto data = devAndSensors->poseSensor->export_localization_map();
    file.write(reinterpret_cast<const char *>(data.begin().base()),
               data.size());
    file.close();

    env->ReleaseStringUTFChars(savePath, pathNativeStr);

    __android_log_print(ANDROID_LOG_INFO, logTag,
                        "Relocalization map exported");

    // TODO: Camera never gets started again...
    // If we try to call pipeline->start() it doesn't work. Bug in librealsense?
  } catch (std::exception &e) {
    // TODO: Make sleep time configurable (this will probably be a common issue
    // users run into)
    auto what = std::string(e.what());
    what +=
        " (If you got something like \"null pointer passed for argument "
        "\"buffer\"\", this means that you have a very large relocalization "
        "map, and you should increase exportRelocMapStopDelay in ";
    what += __FILE__;
    what += ")";
    env->ThrowNew(exception, what.c_str());
  }
}

void importRelocalizationMap(const char *path, rs2::pose_sensor *poseSensor) {
  // Open file and make a vector to hold contents
  auto file = std::ifstream(path, std::ios::binary);
  if (!file || file.bad())
    throw std::runtime_error("Couldn't open file to read a relocalization map");

  auto dataVec = std::vector<uint8_t>(std::istreambuf_iterator<char>(file),
                                      std::istreambuf_iterator<char>());

  // Pass contents to the pose sensor
  auto success = poseSensor->import_localization_map(dataVec);
  if (!success)
    throw std::runtime_error(
        "import_localization_map returned a value indicating failure");

  // TODO: Transform by get_static_node("origin", ...)?
  // Useful if users want to have the same origin. In practice this hasn't been
  // requested and users seem content having their origin after map import being
  // the real origin.

  file.close();
}

extern "C" JNIEXPORT void JNICALL
Java_com_spartronics4915_lib_T265Camera_setOdometryInfo(
    JNIEnv *env, jobject thisObj, jfloat xOffset, jfloat yOffset,
    jfloat angOffset, jdouble measureCovariance) {
  try {
    ensureCache(env, thisObj);

    // std::format cannot come soon enough :(
    auto size = snprintf(nullptr, 0, odometryConfig, measureCovariance,
                         -yOffset, -xOffset, angOffset);
    char buf[size];
    snprintf(buf, size, odometryConfig, measureCovariance, -yOffset, -xOffset,
             angOffset);
    auto vecBuf = std::vector<uint8_t>(buf, buf + size);

    auto devAndSensors = getDeviceFromClass(env, thisObj);
    devAndSensors->wheelOdometrySensor->load_wheel_odometery_config(vecBuf);
  } catch (std::exception &e) {
    env->ThrowNew(exception, e.what());
  }
}

extern "C" JNIEXPORT void JNICALL
Java_com_spartronics4915_lib_T265Camera_free(JNIEnv *env, jobject thisObj) {
  // It's important that we cannot initialize another camera while we're
  // free'ing or vice versa. This global mutex also prevents cleanup from being
  // called simeltaneously. In practice this mutex is only ever contended when
  // the T265 goes through a weird init sequence from cold boot (see comment in
  // the newCamera method.)
  std::scoped_lock lk(cleanupMutex);

  try {
    ensureCache(env, thisObj);

    auto devAndSensors = getDeviceFromClass(env, thisObj);

    toBeCleaned.erase(
        std::remove(toBeCleaned.begin(), toBeCleaned.end(), devAndSensors),
        toBeCleaned.end());
    env->DeleteGlobalRef(devAndSensors->globalThis);
    delete devAndSensors;

    env->SetLongField(thisObj, handleFieldId, 0);
  } catch (std::exception &e) {
    env->ThrowNew(exception, e.what());
  }
}

extern "C" JNIEXPORT void JNICALL
Java_com_spartronics4915_lib_T265Camera_cleanup(JNIEnv *env, jclass) {
  std::scoped_lock lk(cleanupMutex);

  try {
    while (!toBeCleaned.empty()) {
      auto back = toBeCleaned.back();
      env->DeleteGlobalRef(back->globalThis);
      delete back;

      toBeCleaned.pop_back();
    }

    __android_log_print(ANDROID_LOG_INFO, logTag,
                        "Native wrapper successfully shut down");
  } catch (std::exception &e) {
    if (exception) {
      env->ThrowNew(exception, e.what());
    } else {
      // JVM bits could have already been cleaned up when we throw
      __android_log_print(ANDROID_LOG_ERROR, logTag, "Exception after JVM cleanup: %s", e.what());
      std::cerr << e.what() << std::endl;
    }
  }
}

deviceAndSensors *getDeviceFromClass(JNIEnv *env, jobject thisObj) {
  auto pointer = env->GetLongField(thisObj, handleFieldId);
  if (pointer == 0)
    throw std::runtime_error("nativeCameraObjectPointer cannot be 0");
  return reinterpret_cast<deviceAndSensors *>(pointer);
}

void ensureCache(JNIEnv *env, jobject thisObj) {
  if (!holdingClass) {
    auto lHoldingClass = env->GetObjectClass(thisObj);
    holdingClass = reinterpret_cast<jclass>(env->NewGlobalRef(lHoldingClass));
  }
  if (!handleFieldId) {
    handleFieldId =
        env->GetFieldID(holdingClass, "mNativeCameraObjectPointer", "J");
  }
  if (!exception) {
    auto lException =
        env->FindClass("com/spartronics4915/lib/T265Camera$CameraJNIException");
    exception = reinterpret_cast<jclass>(env->NewGlobalRef(lException));
  }
}
