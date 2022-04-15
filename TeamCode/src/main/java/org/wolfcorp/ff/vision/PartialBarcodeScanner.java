package org.wolfcorp.ff.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.wolfcorp.ff.opmode.util.Match;

public class PartialBarcodeScanner extends BarcodeScanner {
    public PartialBarcodeScanner(OpenCvCamera cam) {
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
                    new Point((int)(mat.width() / 2), mat.height())
            );
            midROI   = new Rect(
                    new Point((int)(mat.width() / 2) + 1, 0),
                    new Point((int)(mat.width()), mat.height())
            );
        }

        Mat leftMat = mat.submat(leftROI);
        Mat midMat = mat.submat(midROI);

        double leftValue = Core.sumElems(leftMat).val[0];
        double midValue = Core.sumElems(midMat).val[0];

        leftItem.setValue(leftValue);
        midItem.setValue(midValue);
        rightItem.setValue(0);

        leftMat.release();
        midMat.release();
        mat.release();

        double max = Math.max(leftValue, midValue);
        Scalar matchColor = new Scalar(0, 255, 0);
        Scalar mismatchColor = new Scalar(255, 0, 0);
        Scalar leftColor = mismatchColor, midColor = mismatchColor, rightColor = mismatchColor;
        if (max < 20000) {
            barcode = Barcode.TOP;
            barcodeItem.setValue("right");
            targetLevelItem.setValue("top");
            rightColor = matchColor;
        } else if (max == leftValue) {
            barcode = Barcode.BOT;
            barcodeItem.setValue("left");
            targetLevelItem.setValue("bottom");
            leftColor = matchColor;
        } else if (max == midValue) {
            barcode = Barcode.MID;
            barcodeItem.setValue("middle");
            targetLevelItem.setValue("middle");
            midColor = matchColor;
        }
        latch.countDown();
        Match.update();

        Imgproc.rectangle(input, leftROI, leftColor);
        Imgproc.rectangle(input, midROI, midColor);

        return input;
    }


}
