package org.wolfcorp.ff.vision;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class Detector extends OpenCvPipeline {
    private OpenCvCamera camera;

    public Detector(OpenCvCamera cam) {
        camera = cam;
    }

    public void start() {
        camera.setPipeline(this);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void stop() {
        camera.stopStreaming();
    }
}
