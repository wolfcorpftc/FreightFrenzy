package org.wolfcorp.ff.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

public class WarehouseGuide extends Detector {
    private static final double INVALID_ANGLE = Double.MIN_VALUE;

    private Mat mat = new Mat();
    private Mat hierarchy = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Freight target = Freight.GOLD;
    private double targetAngle = INVALID_ANGLE;

    private final Object updateLock = new Object();
    private final ResettableCountDownLatch latch = new ResettableCountDownLatch(1);

    public WarehouseGuide(OpenCvCamera cam) {
        super(cam);
    }

    public WarehouseGuide(OpenCvCamera cam, Freight target) {
        super(cam);
        this.target = target;
    }

    public Mat processFrame(Mat input) {
        synchronized (updateLock) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
            // Use blurring to reduce extraneous contours (e.g., holes in silver)
            Imgproc.medianBlur(mat, mat, 20);
            // TODO: Idea -- perform edge detection on mat here, bitwise-and that matrix with threshed,
            //   and add to the contour matrix; erode, edge-detect, and dilate if necessary

            // *** Thresholding ***
            if (target == Freight.GOLD) {
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
                Scalar lowerBound = new Scalar(0, 198, 0);
                Scalar upperBound = new Scalar(180, 255, 255);
                Core.inRange(mat, lowerBound, upperBound, mat);
            } else { // target == Freight.SILVER
                // HSL is better for identifying the color white
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HLS);
                // TODO: color temperature? normalize on field, make it tunable (text file?)
                Scalar lowerBound = new Scalar(0, 216.75, 0);
                Scalar upperBound = new Scalar(179, 255, 255);
                Core.inRange(mat, lowerBound, upperBound, mat);
            }

            // *** Separate overlapping blobs ***
            Imgproc.erode(mat, mat, new Mat(), new Point(-1, -1), 15);
            Imgproc.dilate(mat, mat, new Mat(), new Point(-1, -1), 15);

            // *** Find contours ***
            Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            double width = mat.width();
            double height = mat.height();
            mat.release();
            hierarchy.release();

            // *** Find bounding rectangles ***
            MatOfPoint2f contoursPoly = new MatOfPoint2f();
            Rect[] boundRect = new Rect[contours.size()];
            Scalar rectColor =
                    target == Freight.GOLD ? new Scalar(255, 171, 23, 1) : new Scalar(192, 192, 192, 1);
            for (int i = 0; i < contours.size(); i++) {
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly, 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly.toArray()));
                contours.get(i).release();
                contoursPoly.release();
                Imgproc.rectangle(input, boundRect[i], rectColor);
            }
            contours.clear();

            // *** Find center coordinates of rectangles ***
            Point[] centers = new Point[boundRect.length];
            for (int i = 0; i < boundRect.length; i++) {
                centers[i] = new Point(
                        boundRect[i].x + boundRect[i].width / 2.0,
                        boundRect[i].y + boundRect[i].height / 2.0
                );
            }

            // *** Find coordinates closest to robot ***
            Point minPoint = new Point();
            double minDist = Double.MAX_VALUE;
            // TODO: change the point depending on camera placement
            Point robot = new Point(width / 2.0, height);
            for (Point c : centers) {
                double dist = Math.hypot(c.x - robot.x, c.y - robot.y);
                if (dist < minDist) {
                    minPoint = c;
                    minDist = dist;
                }
            }
            Imgproc.circle(input, minPoint, 15, new Scalar(0, 255, 0, 1), 5);

            // assume a 60 deg field of view (value sourced from cam spec)
            targetAngle = 60 * minPoint.x / width - 30.0;
            latch.countDown();
        }

        return input;
    }

    public double getTargetAngle() throws InterruptedException {
        if (targetAngle == INVALID_ANGLE) {
            latch.await();
        }
        return targetAngle;
    }

    public Freight getTargetType() {
        return target;
    }

    public void setTargetType(Freight f) {
        synchronized (updateLock) {
            latch.reset();
            target = f;
            targetAngle = INVALID_ANGLE;
        }
    }
}
