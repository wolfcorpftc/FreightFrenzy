package org.wolfcorp.ff.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

public class TFWarehouseGuide implements Guide {
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

    public static final PolarPoint FALLBACK = new PolarPoint(3, 0);

    private TFObjectDetector tfod;
    private String targetLabel = "Ball";
    private PolarPoint lastNavigation = FALLBACK;

    public TFWarehouseGuide(VuforiaLocalizer localizer, HardwareMap hardwareMap) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, localizer);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        tfod.setZoom(2.5, 16.0/9.0);
    }

    public TFWarehouseGuide(VuforiaLocalizer localizer, HardwareMap hardwareMap, Freight f) {
        this(localizer, hardwareMap);
        setTarget(f);
    }

    /**
     * Sets the desired target freight type.
     * @param f desired target freight type
     */
    public void setTarget(Freight f) {
        if (f == Freight.SILVER) {
            targetLabel = "Ball";
        }
        else {
            targetLabel = "Cube";
        }
    }

    /**
     * @return current target freight type
     */
    public Freight getTarget() {
        switch (targetLabel) {
            default:
            case "Ball":
                return Freight.SILVER;
            case "Cube":
                return Freight.GOLD;
        }
    }

    /**
     * @return last navigation result accessed through {@link TFWarehouseGuide#navigate()}
     */
    @Override
    public PolarPoint getLastNavigation() {
        return lastNavigation;
    }

    /**
     * Activates the TFOD to start scanning for freight.
     */
    public void start() {
        tfod.activate();
    }

    /**
     * Deactivates and shuts down the object detector.
     */
    public void stop() {
        tfod.deactivate();
        tfod.shutdown();
    }

    /**
     * Calculates the relative position of the nearest target freight.
     * @return a {@link PolarPoint} that indicates the nearest desired target object (also see
     * {@link TFWarehouseGuide#setTarget(Freight)})
     */
    public PolarPoint navigate() {
        List<Recognition> recognitions = tfod.getRecognitions();
        if (recognitions.isEmpty()) {
            lastNavigation = FALLBACK;
            return FALLBACK;
        }

        // *** Find centers of recognitions ***
        ArrayList<Point> centers = new ArrayList<>();
        for (Recognition r : recognitions) {
            if (r.getLabel().equals(targetLabel)) {
                centers.add(new Point(
                        r.getLeft() + r.getWidth() / 2,
                        // TODO: check whether y = 0 starts from the top (otherwise change - to +)
                        r.getBottom() - r.getHeight() / 2
                ));
            }
        }
        if (centers.isEmpty()) {
            lastNavigation = FALLBACK;
            return FALLBACK;
        }

        // *** Find coordinates closest to robot ***
        double width = recognitions.get(0).getImageWidth();
        double height = recognitions.get(0).getImageHeight();
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
        PolarPoint navigation = new PolarPoint(WarehouseGuide.DIST_FACTOR * minDist, 60 * minPoint.x /  - 30.0);
        lastNavigation = navigation;
        return navigation;
    }
}
