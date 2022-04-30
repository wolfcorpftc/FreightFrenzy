package org.wolfcorp.ff.vision;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.wolfcorp.ff.opmode.util.Match;

public abstract class Detector extends OpenCvPipeline {
    private OpenCvCamera camera;

    public Detector(OpenCvCamera cam) {
        camera = cam;
    }

    public void start() {
        Match.log("Started streaming");
        camera.setPipeline(this);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public void stop() {
        camera.stopStreaming();
    }
}
