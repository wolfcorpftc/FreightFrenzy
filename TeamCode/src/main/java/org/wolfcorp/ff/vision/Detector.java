package org.wolfcorp.ff.vision;

import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class Detector extends OpenCvPipeline {
    private OpenCvWebcam webcam;

    public Detector(OpenCvWebcam cam) {
        webcam = cam;
        webcam.setPipeline(this);
    }

    public void start() {
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void stop() {
        webcam.stopStreaming();
    }
}
