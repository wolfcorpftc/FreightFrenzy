package org.wolfcorp.ff.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.CountDownLatch;

public class BarcodeScanner extends Detector {
    private Mat mat;
    private Rect leftROI, midROI, rightROI;
    private Mat leftMat, midMat, rightMat;
    private Telemetry telemetry;
    private volatile Barcode barcode = null;
    private CountDownLatch latch = new CountDownLatch(1);

    public BarcodeScanner(OpenCvCamera cam, Telemetry t) {
        super(cam);
        telemetry = t;

        // TODO: figure out coordinates of ROIs
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        // TODO: figure out lower and upper bounds for shipping element
        Scalar lowerBound = new Scalar(0,0,0);
        Scalar upperBound = new Scalar(0,0,0);
        Core.inRange(mat, lowerBound, upperBound, mat);

        // TODO: switch to exact ROIs once we have actual camera placement
        if (leftROI == null) {
            // casting int-division to int to suppress linter
            leftROI  = new Rect(
                    new Point(0, 0),
                    new Point((int)(mat.width() / 3), mat.height())
            );
            midROI   = new Rect(
                    new Point((int)(mat.width() / 3) + 1, 0),
                    new Point((int)(2 * mat.width() / 3), mat.height())
            );
            rightROI = new Rect(
                    new Point((int)(2 * mat.width() / 3) + 1, 0),
                    new Point(mat.width(), mat.height())
            );
        }

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
            telemetry.addData("Barcode", "left");
            telemetry.addData("Target Level", "bottom");
            leftColor = matchColor;
            midColor = rightColor = mismatchColor;
        }
        else if (max == midValue) {
            barcode = Barcode.MID;
            telemetry.addData("Barcode", "middle");
            telemetry.addData("Target Level", "middle");
            midColor = matchColor;
            leftColor = rightColor = mismatchColor;
        }
        else {
            barcode = Barcode.TOP;
            telemetry.addData("Barcode", "right");
            telemetry.addData("Result", "top");
            rightColor = matchColor;
            leftColor = midColor = mismatchColor;
        }
        latch.countDown();
        telemetry.update();

        Imgproc.rectangle(input, leftROI, leftColor);
        Imgproc.rectangle(input, midROI, midColor);
        Imgproc.rectangle(input, rightROI, rightColor);

        return input;
    }

    public Barcode getBarcode() throws InterruptedException {
        if (barcode == null) {
            latch.await();
        }
        return barcode;
    }
}
