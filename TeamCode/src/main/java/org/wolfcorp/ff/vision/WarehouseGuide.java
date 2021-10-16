package org.wolfcorp.ff.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class WarehouseGuide extends Detector {
    private Mat mat = null;
    private Mat hierarchy = null;
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Freight target = Freight.GOLD;
    private double goldAngle = 0;
    private double silverAngle = 0;

    public WarehouseGuide(OpenCvWebcam cam) {
        super(cam);
    }

    public WarehouseGuide(OpenCvWebcam cam, Freight target) {
        super(cam);
        setTargetType(target);
    }

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        if (target == Freight.GOLD) {
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
            // Use blurring to reduce extraneous contours (e.g., holes in silver)
            Imgproc.medianBlur(mat, mat, 20);
            Scalar lowerBound = new Scalar(0, 198, 0);
            Scalar upperBound = new Scalar(180, 255, 255);
            Core.inRange(mat, lowerBound, upperBound, mat);
            goldAngle = findFreightAngle();
        }
        else { // target == Freight.SILVER
            // HSL is better for identifying the color white
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HLS);
            // Use blurring to reduce extraneous contours (e.g., holes in silver)
            Imgproc.medianBlur(mat, mat, 20);
            Scalar lowerBound = new Scalar(0, 216.75, 0);
            Scalar upperBound = new Scalar(179, 255, 255);
            Core.inRange(mat, lowerBound, upperBound, mat);
            silverAngle = findFreightAngle();
        }
        return input;
    }

    // Calculate angle to freight given a threshed image (mat)
    protected double findFreightAngle() {
        // *** Find coutours ***
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        double width = mat.width();
        double height = mat.height();
        mat.release();
        hierarchy.release();

        // *** Find bounding rectangles ***
        MatOfPoint2f contoursPoly = new MatOfPoint2f();
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly, 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly.toArray()));
            contours.get(i).release();
            contoursPoly.release();
        }

        // *** Find center coordinates of rectangles ***
        Point[] centers = new Point[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            centers[i] = new Point(
                    boundRect[i].x + boundRect[i].width / 2.0,
                    boundRect[i].y + boundRect[i].height / 2.0
            );
        }

        // *** Find coordinates closest to robot ***
        Point minPoint = new Point();
        double minDist = Double.MAX_VALUE;
        Point robot = new Point(width / 2.0, height);
        for (Point c : centers) {
            double dist = Math.hypot(c.x - robot.x, c.y - robot.y);
            if (dist < minDist) {
                minPoint = c;
                minDist = dist;
            }
        }

        // assume a 60 deg field of view (value sourced from cam spec)
        return 60 * minPoint.x / width - 30.0;
    }

    public double getGoldAngle() throws InterruptedException {
        while (goldAngle == -9999) Thread.sleep(10);
        return goldAngle;
    }

    public double getSilverAngle() throws InterruptedException {
        while (silverAngle == -9999) Thread.sleep(10);
        return silverAngle;
    }

    public Freight getTargetType() {
        return target;
    }

    public void setTargetType(Freight f) {
        target = f;
        if (f == Freight.GOLD)
            silverAngle = -9999;
        else
            goldAngle = -9999;
    }
}
