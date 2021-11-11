package org.wolfcorp.ff.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class TFWarehouseGuide {

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final double INVALID_ANGLE = Double.MIN_VALUE;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Telemetry telemetry;
    private String targetLabel = "Cube";

    @SuppressLint("DefaultLocale")
    public TFWarehouseGuide(VuforiaLocalizer localizer, HardwareMap hardwareMap, Telemetry tl) {
        vuforia = localizer;
        telemetry = tl;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        tfod.setZoom(2.5, 16.0/9.0);
    }

    public TFWarehouseGuide(Freight f,VuforiaLocalizer localizer, HardwareMap hardwareMap, Telemetry tl) {
        this(localizer, hardwareMap, tl);
        setTargetType(f);
    }

    public void setTargetType(Freight f) {
        if (f == Freight.SILVER) {
            targetLabel = "Ball";
        }
        else {
            targetLabel = "Cube";
        }
    }

    public double getTargetAngle(double msTimeout) {
        tfod.activate();
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> lastRecognitions = null;
        List<Recognition> updatedRecognitions;
        ElapsedTime timer = new ElapsedTime();
        do {
            if ((updatedRecognitions = tfod.getUpdatedRecognitions()) != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
                lastRecognitions = updatedRecognitions;
                telemetry.update();
            }
        } while (timer.milliseconds() < msTimeout && lastRecognitions == null);
        tfod.deactivate();

        if (lastRecognitions == null) {
            return 0;
        }

        // *** Find centers of recognitions ***
        ArrayList<Point> centers = new ArrayList<>();
        for (Recognition r : lastRecognitions) {
            if (r.getLabel().equals(targetLabel)) {
                centers.add(new Point(
                        r.getLeft() + r.getWidth() / 2,
                        // TODO: check whether y = 0 starts from the top (otherwise change - to +)
                        r.getBottom() - r.getHeight() / 2
                ));
            }
        }

        // *** Find coordinates closest to robot ***
        double width = lastRecognitions.get(0).getImageWidth();
        double height = lastRecognitions.get(0).getImageHeight();
        Point minPoint = new Point();
        double minDist = Double.MAX_VALUE;
        // TODO: change the point depending on camera placement
        // TODO: check whether y = 0 starts from the top (otherwise change height to 0)
        Point robot = new Point(width / 2.0, height);
        for (Point c : centers) {
            double dist = Math.hypot(c.x - robot.x, c.y - robot.y);
            if (dist < minDist) {
                minPoint = c;
                minDist = dist;
            }
        }

        // assume a 60 deg field of view (value sourced from cam spec)
        return 60 * minPoint.x /  - 30.0;
    }

    public double getTargetAngle() {
        return getTargetAngle(0);
    }
}
