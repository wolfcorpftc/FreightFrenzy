package org.wolfcorp.ff.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.wolfcorp.ff.opmode.StartingLocation;

public class BarcodeScanner extends Detector {
    private Mat mat;
    private Rect leftROI, midROI, rightROI;
    private Mat leftMat, midMat, rightMat;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Barcode barcode = null;

    public BarcodeScanner(OpenCvWebcam cam, Telemetry t) {
        super(cam);
        telemetry = t;

        // TODO: figure out coordinates of ROIs
        leftROI  = new Rect(new Point(), new Point());
        midROI   = new Rect(new Point(), new Point());
        rightROI = new Rect(new Point(), new Point());
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        // TODO: figure out lower and upper bounds for shipping element
        Scalar lowerBound = new Scalar(0,0,0);
        Scalar upperBound = new Scalar(0,0,0);
        Core.inRange(mat, lowerBound, upperBound, mat);

        leftMat = mat.submat(leftROI);
        midMat = mat.submat(midROI);
        rightMat = mat.submat(rightROI);

        double leftValue = Core.mean(leftMat).val[2];
        double midValue = Core.mean(midMat).val[2];
        double rightValue = Core.mean(rightMat).val[2];
        telemetry.addData("Left", leftValue);
        telemetry.addData("Middle", midValue);
        telemetry.addData("Right", rightValue);

        leftMat.release();
        midMat.release();
        rightMat.release();
        mat.release();

        double max = Math.max(leftValue, Math.max(midValue, rightValue));
        Scalar matchColor = new Scalar(0, 255, 0);
        Scalar mismatchColor = new Scalar(255, 0, 0);
        Scalar leftColor, midColor, rightColor;
        if (max == leftValue) {
            barcode = Barcode.BOT;
            telemetry.addData("Result", "left / bottom");
            leftColor = matchColor;
            midColor = rightColor = mismatchColor;
        }
        else if (max == midValue) {
            barcode = Barcode.MID;
            telemetry.addData("Result", "middle");
            midColor = matchColor;
            leftColor = rightColor = mismatchColor;
        }
        else {
            barcode = Barcode.TOP;
            telemetry.addData("Result", "right / top");
            rightColor = matchColor;
            leftColor = midColor = mismatchColor;
        }
        telemetry.update();

        Imgproc.rectangle(input, leftROI, leftColor);
        Imgproc.rectangle(input, midROI, midColor);
        Imgproc.rectangle(input, rightROI, rightColor);

        return input;
    }

    public Barcode getBarcode() {
        return barcode;
    }
}
