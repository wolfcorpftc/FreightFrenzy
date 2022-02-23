package org.wolfcorp.ff.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.wolfcorp.ff.opmode.util.Match;

/**
 * Scans barcode by looking for obscured barcode tape.
 */
public class BarcodeTapeScanner extends BarcodeScanner {
    public BarcodeTapeScanner(OpenCvCamera cam) {
        super(cam);
    }

    @Override
    public Mat processFrame(Mat input) {
        if (Match.RED) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        } else {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2BGR);
        }

        if (leftROI == null) {
            int third = mat.width() / 3;
            leftROI  = new Rect(new Point(0, 0), new Point(third, mat.height()));
            midROI   = new Rect(new Point(third + 1, 0), new Point(2 * third, mat.height()));
            rightROI = new Rect(new Point(2 * third + 1, 0), new Point(mat.width(), mat.height()));
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

        double min = Math.min(leftValue, Math.min(midValue, rightValue));
        Scalar matchColor = new Scalar(0, 255, 0);
        Scalar mismatchColor = new Scalar(255, 0, 0);
        Scalar leftColor, midColor, rightColor;
        if (min == leftValue) {
            barcode = Barcode.BOT;
            barcodeItem.setValue("left");
            targetLevelItem.setValue("bottom");
            leftColor = matchColor;
            midColor = rightColor = mismatchColor;
        }
        else if (min == midValue) {
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
        Match.telemetry.update();

        Imgproc.rectangle(input, leftROI, leftColor);
        Imgproc.rectangle(input, midROI, midColor);
        Imgproc.rectangle(input, rightROI, rightColor);

        return input;
    }

}
