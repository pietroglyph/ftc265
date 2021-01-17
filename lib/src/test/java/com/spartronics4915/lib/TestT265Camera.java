package com.spartronics4915.lib;

import static junit.framework.TestCase.fail;
import static org.junit.Assert.assertTrue;

import android.content.Context;
import androidx.test.core.app.ApplicationProvider;
import com.arcrobotics.ftclib.geometry.Transform2d;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.Ignore;
import org.junit.Test;

public class TestT265Camera {
    private boolean mDataRecieved = false;
    private final Object mLock = new Object();

    private Context context = ApplicationProvider.getApplicationContext();

    @Ignore
    @Test
    public void testNewCamera() throws InterruptedException {
        // This one is a little hard to unit test because we haven't simulated the hardware
        // We mostly just make sure that we can get through this sequence without throwing an
        // exception
        // The rest is just a few sanity checks

        T265Camera cam = null;
        try {
            cam = new T265Camera(new Transform2d(), 0, context);

            // Just make sure this doesn't throw
            cam.sendOdometry(0, 0);

            cam.start(
                    (T265Camera.CameraUpdate update) -> {
                        synchronized (mLock) {
                            mDataRecieved = true;
                        }
                        System.out.println("Got pose with confidence " + update.pose);
                    });
            System.out.println(
                    "Waiting 5 seconds to recieve data... Move the camera around in the path of the shape of a cross for best results. This will not work unless you get to High confidence.");
            Thread.sleep(5000);
            cam.stop();
            synchronized (mLock) {
                assertTrue(
                        "No pose data was recieved after 5 seconds... Try moving the camera?",
                        mDataRecieved);
            }

            System.out.println("Got pose data, exporting relocalization map to java.io.tmpdir...");
            Path mapPath =
                    Paths.get(System.getProperty("java.io.tmpdir"), "map.bin").toAbsolutePath();
            cam.exportRelocalizationMap(mapPath.toString());

            if (mapPath.toFile().length() <= 0) fail("Relocalization map file length was 0");
            System.out.println(
                    "Successfuly saved relocalization map, we will now try to import it");

            cam.free();

            // Try making a new camera and importing the map
            cam = new T265Camera(new Transform2d(), 0f, mapPath.toString(), context);

            System.out.println("Map imported without errors!");
        } finally {
            if (cam != null) cam.free();
        }
    }

    @Ignore
    @Test
    public void testErrorChecking() {
        T265Camera cam = null;
        try {
            cam = new T265Camera(new Transform2d(), 0, context);
            cam.start((T265Camera.CameraUpdate unused) -> {});

            final T265Camera camTemp = cam;
            try {
                camTemp.start((T265Camera.CameraUpdate unused) -> {});
            } catch (RuntimeException e) {
                return;
            }
            fail("Camera didn't throw after double start call");
        } finally {
            if (cam != null) cam.free();
        }
    }
}
