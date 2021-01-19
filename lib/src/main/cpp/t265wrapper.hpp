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

#include <jni.h>
#include <librealsense2/rs.hpp>
#include <mutex>
/* Header for class com_spartronics4915_lib_T265Camera */

#ifndef _Included_com_spartronics4915_lib_T265Camera
#define _Included_com_spartronics4915_lib_T265Camera
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_spartronics4915_lib_T265Camera
 * Method:    exportRelocalizationMap
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_com_spartronics4915_lib_T265Camera_exportRelocalizationMap(JNIEnv *,
                                                                jobject,
                                                                jstring);

/*
 * Class:     com_spartronics4915_lib_T265Camera
 * Method:    free
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_spartronics4915_lib_T265Camera_free(JNIEnv *,
                                                                    jobject);

/*
 * Class:     com_spartronics4915_lib_T265Camera
 * Method:    setOdometryInfo
 * Signature: (FFFD)V
 */
JNIEXPORT void JNICALL Java_com_spartronics4915_lib_T265Camera_setOdometryInfo(
    JNIEnv *, jobject, jfloat, jfloat, jfloat, jdouble);

/*
 * Class:     com_spartronics4915_lib_T265Camera
 * Method:    sendOdometryRaw
 * Signature: (IFF)V
 */
JNIEXPORT void JNICALL Java_com_spartronics4915_lib_T265Camera_sendOdometryRaw(
    JNIEnv *, jobject, jint, jfloat, jfloat);

/*
 * Class:     com_spartronics4915_lib_T265Camera
 * Method:    newCamera
 * Signature: (Ljava/lang/String;)J
 */
JNIEXPORT jlong JNICALL
Java_com_spartronics4915_lib_T265Camera_newCamera(JNIEnv *, jobject, jstring);

/*
 * Class:     com_spartronics4915_lib_T265Camera
 * Method:    cleanup
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_spartronics4915_lib_T265Camera_cleanup(JNIEnv *,
                                                                       jclass);

#ifdef __cplusplus
}
#endif
#endif

// NOT machine generated like the above :)
// The "native pointer" that the Java code holds points to an instance of this
// class.
class deviceAndSensors {
public:
  deviceAndSensors(rs2::pipeline *pipe, rs2::wheel_odometer *odom,
                   rs2::pose_sensor *pose, jobject globalThis)
      : pipeline(pipe), wheelOdometrySensor(odom), poseSensor(pose),
        globalThis(globalThis) {}

  ~deviceAndSensors() {
    delete pipeline;
    delete poseSensor;
    delete wheelOdometrySensor;
  }

  rs2::pipeline *pipeline;
  rs2::wheel_odometer *wheelOdometrySensor;
  rs2::pose_sensor *poseSensor;
  jobject globalThis;
  bool isRunning = true;

  std::mutex frameNumMutex;
  unsigned long long lastRecvdFrameNum = 0;
};

void importRelocalizationMap(const char *path, rs2::pose_sensor *pose);
deviceAndSensors *getDeviceFromClass(JNIEnv *env, jobject thisObj);
void ensureCache(JNIEnv *env, jobject thisObj);
