 package com.spartronics4915.lib;

 import static org.junit.jupiter.api.Assertions.assertThrows;
 import static org.junit.jupiter.api.Assertions.assertTrue;
 import static org.junit.jupiter.api.Assertions.fail;

 import java.nio.file.Path;
 import java.nio.file.Paths;

 import com.arcrobotics.ftclib.geometry.Pose2d;
 import com.arcrobotics.ftclib.geometry.Rotation2d;
 import com.arcrobotics.ftclib.geometry.Transform2d;
 import com.arcrobotics.ftclib.geometry.Twist2d;
 import com.spartronics4915.lib.T265Camera;

 import org.junit.jupiter.api.Tag;
 import org.junit.jupiter.api.Test;

 public class TestT265Camera
 {

     private boolean mDataRecieved = false;
     private final Object mLock = new Object();

     @Tag("hardwareDependant")
     @Test
     public void testNewCamera() throws InterruptedException
     {
         // This one is a little hard to unit test because we haven't simulated the hardware
         // We mostly just make sure that we can get through this sequence without throwing an exception
         // The rest is just a few sanity checks

         T265Camera cam = null;
         try
         {
             cam = new T265Camera(new Transform2d(), 0);

             // Just make sure this doesn't throw
             cam.sendOdometry(new Twist2d(0, 0, 0));

             cam.start((T265Camera.CameraUpdate update) ->
             {
                 synchronized (mLock)
                 {
                     mDataRecieved = true;
                 }
                 System.out.println("Got pose with confidence " + update.pose);
             });
             System.out.println(
                     "Waiting 5 seconds to recieve data... Move the camera around in the path of the shape of a cross for best results. This will not work unless you get to High confidence.");
             Thread.sleep(5000);
             cam.stop();
             synchronized (mLock)
             {
                 assertTrue(mDataRecieved, "No pose data was recieved after 5 seconds... Try moving the camera?");
             }

             System.out.println("Got pose data, exporting relocalization map to java.io.tmpdir...");
             Path mapPath = Paths.get(System.getProperty("java.io.tmpdir"), "map.bin").toAbsolutePath();
             cam.exportRelocalizationMap(mapPath.toString());

             if (mapPath.toFile().length() <= 0)
                 fail("Relocalization map file length was 0");
             System.out.println("Successfuly saved relocalization map, we will now try to import it");

             cam.free();

             // Try making a new camera and importing the map
             cam = new T265Camera(new Transform2d(), 0f, mapPath.toString());

             System.out.println("Map imported without errors!");
         }
         finally
         {
             if (cam != null)
                 cam.free();
         }
     }

     @Tag("hardwareDependant")
     @Test
     public void testErrorChecking()
     {
         T265Camera cam = null;
         try
         {
             cam = new T265Camera(new Transform2d(), 0);
             cam.start((T265Camera.CameraUpdate unused) -> {});

             final T265Camera camTemp = cam;
             assertThrows(RuntimeException.class, () -> camTemp.start((T265Camera.CameraUpdate unused) -> {}));
         }
         finally
         {
             if (cam != null)
                 cam.free();
         }
     }

 }
