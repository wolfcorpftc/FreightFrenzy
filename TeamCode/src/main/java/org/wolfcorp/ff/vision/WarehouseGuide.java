package org.wolfcorp.ff.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class WarehouseGuide extends OpenCvPipeline {
    private OpenCvWebcam webcam;
    private Mat silverMat = null;
    private Mat goldMat = null;
    Mat hierarchy = null;
    ArrayList<MatOfPoint> contours = new ArrayList<>();

    public WarehouseGuide(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam.setPipeline(this);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public Mat processFrame(Mat input) {
        // TODO: *** silver guide ***
        // --- Thresholding input -> silver ---
        Imgproc.cvtColor(input, silverMat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(silverMat, silverMat, Imgproc.COLOR_RGB2HLS);
        double sensitivity = 15;
        Scalar silverLowerBound = new Scalar(0, 216.75, 0);
        Scalar silverUpperBound = new Scalar(179, 255, 255);
        Core.inRange(silverMat, silverLowerBound, silverUpperBound, silverMat);
        // TODO: contour detection for silver
        //silverMat, contours,
        Imgproc.findContours(silverMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        // TODO: find polygons
        // TODO: find center coordinates
        // TODO: find coordinates closest to robot

        // TODO: *** gold guide ***
        // TODO: threshold input -> gold
        // TODO: find polygons
        // TODO: find center coordinates
        // TODO: find coordinates closest to robot
        return input;
    }

    // TODO: implement
    public double getGoldAngle() {
        return 0;
    }

    // TODO: implement
    public double getSilverAngle() {
        return 0;
    }

    public void stop() {
        webcam.stopStreaming();
    }
}
