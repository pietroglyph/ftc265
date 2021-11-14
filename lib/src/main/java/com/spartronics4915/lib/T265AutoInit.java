package com.spartronics4915.lib;

import android.content.Context;
import android.util.Log;
import com.qualcomm.ftccommon.FtcEventLoop;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

public class T265AutoInit {
    private static final String kLogTag = "ftc265";

    public static T265Camera slamera;

    /**
     * We have to have a reference to the listener so that it doesn't get garbage collected. The SDK
     * uses a weak reference to the listener, so we need to keep a strong reference to it.
     */
    private static final T265OpModeListener opModeListener = new T265OpModeListener();

    /**
     * Add a listener to initialize the T265Camera when an opmode is created.
     *
     * @param appContext Passed in by the FTC SDK
     * @param eventLoop Passed in by the FTC SDK
     */
    @SuppressWarnings("unused")
    @OnCreateEventLoop
    public static void onCreateEventLoop(Context appContext, FtcEventLoop eventLoop) {
        Log.i(kLogTag, "Attaching op mode listener");
        eventLoop.getOpModeManager().registerListener(opModeListener);
    }
}
