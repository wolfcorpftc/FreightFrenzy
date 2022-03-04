package org.wolfcorp.ff.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.wolfcorp.ff.opmode.util.Match;

import java.util.concurrent.CountDownLatch;

// TODO: rename to Decoder
public class BarcodeScanner extends Detector {
    protected final Mat mat = new Mat();
    protected Rect leftROI, midROI, rightROI;
    protected volatile Barcode barcode = null;
    protected final CountDownLatch latch = new CountDownLatch(1);

    protected Telemetry.Item leftItem;
    protected Telemetry.Item midItem;
    protected Telemetry.Item rightItem;
    protected Telemetry.Item barcodeItem;
    protected Telemetry.Item targetLevelItem;

    public BarcodeScanner(OpenCvCamera cam) {
        super(cam);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBound = new Scalar(30,100,100);
        Scalar upperBound = new Scalar(80,255,255);
        Core.inRange(mat, lowerBound, upperBound, mat);

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

        Mat leftMat = mat.submat(leftROI);
        Mat midMat = mat.submat(midROI);
        Mat rightMat = mat.submat(rightROI);

        double leftValue = Core.sumElems(leftMat).val[0];
        double midValue = Core.sumElems(midMat).val[0];
        double rightValue = Core.sumElems(rightMat).val[0];

        leftItem.setValue(leftValue);
        midItem.setValue(midValue);
        rightItem.setValue(rightValue);

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
            barcodeItem.setValue("left");
            targetLevelItem.setValue("bottom");
            leftColor = matchColor;
            midColor = rightColor = mismatchColor;
        }
        else if (max == midValue) {
            barcode = Barcode.MID;
            barcodeItem.setValue("middle");
            targetLevelItem.setValue("middle");
            midColor = matchColor;
            leftColor = rightColor = mismatchColor;
        }
        else {
            barcode = Barcode.TOP;
            barcodeItem.setValue("right");
            targetLevelItem.setValue("top");
            rightColor = matchColor;
            leftColor = midColor = mismatchColor;
        }
        latch.countDown();
        Match.update();

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

    public void start() {
        Match.log("Barcode Scanner started");
        leftItem = Match.createLogItem("Barcode - Left Value", 0);
        midItem = Match.createLogItem("Barcode - Mid Value", 0);
        rightItem = Match.createLogItem("Barcode - Right Value", 0);
        barcodeItem = Match.createLogItem("Barcode - Barcode", "awaiting result");
        targetLevelItem = Match.createLogItem("Barcode - Target Level", "awaiting result");
        super.start();
    }
    public void stop() {
        super.stop();
        Match.log("Barcode Scanner stopped");
        Match.removeLogItem(leftItem);
        Match.removeLogItem(midItem);
        Match.removeLogItem(rightItem);
        Match.removeLogItem(barcodeItem);
        Match.removeLogItem(targetLevelItem);
    }
}
