package org.wolfcorp.ff.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.OpMode;
public class Kalman {

    //A - state transition matrix
    private RealMatrix A;
    //B - control input matrix
    private RealMatrix B;
    //H - measurement matrix
    private RealMatrix H;
    //Q - process noise covariance matrix (error in the process)
    private RealMatrix Q;
    //R - measurement noise covariance matrix (error in the measurement)
    private RealMatrix R;
    //PO - error covariance matrix
    private RealMatrix PO;
    //x state
    private RealVector x;

    // discrete time interval (30ms) between to steps
    private final double dt = 0.03d;
    // position measurement noise (0.01 cm)
    private final double measurementNoise = 0.01d;
    private KalmanFilter filter;

    public Kalman() {
        this(new Pose2d(0,0,0));
    }

    public Kalman(Pose2d startPos) {
        //A and B describe the physic model of the user moving specified as matrices
        A = new Array2DRowRealMatrix(new double[][] {
                { 1d, 0d, dt, 0d },
                { 0d, 1d, 0d, dt },
                { 0d, 0d, 1d, 0d },
                { 0d, 0d, 0d, 1d }
        });
        B = new Array2DRowRealMatrix(new double[][] {
                { Math.pow(dt, 2d) / 2d , 0d },
                { 0d , Math.pow(dt, 2d) / 2d },
                { dt, 0d},
                { 0d, dt}
        });
        //only observe first 2 values - the position coordinates
        H = new Array2DRowRealMatrix(new double[][] {
                { 1d, 0d, 0d, 0d },
                { 0d, 1d, 0d, 0d },
        });
        Q = new Array2DRowRealMatrix(new double[][] {
                { Math.pow(dt, 4d)/4d, 0d, Math.pow(dt, 3d)/2d, 0d },
                { 0d, Math.pow(dt, 4d)/4d, 0d, Math.pow(dt, 3d)/2d },
                { Math.pow(dt, 3d)/2d, 0d, Math.pow(dt, 2d), 0d },
                { 0d, Math.pow(dt, 3d)/2d, 0d, Math.pow(dt, 2d) }
        });

        R = new Array2DRowRealMatrix(new double[][] {
                { Math.pow(measurementNoise, 2d), 0d },
                { 0d, Math.pow(measurementNoise, 2d) }
        });

        /*PO = new Array2DRowRealMatrix(new double[][] {
                                                        { 1d, 1d, 1d, 1d },
                                                        { 1d, 1d, 1d, 1d },
                                                        { 1d, 1d, 1d, 1d },
                                                        { 1d, 1d, 1d, 1d }
                                                     });*/

        // x = [ 0 0 0 0] state consists of position and velocity[pX, pY, vX, vY]
        //TODO: inititate with map center?
        x = new ArrayRealVector(new double[] { startPos.getX(), startPos.getY(), 0, 0 });

        ProcessModel pm = new DefaultProcessModel(A, B, Q, x, PO);
        MeasurementModel mm = new DefaultMeasurementModel(H, R);
        filter = new KalmanFilter(pm, mm);
    }


    /**
     * Use Kalmanfilter to decrease measurement errors
     * @param position
     * @return
     */
    public Pose2d estimatePosition(Pose2d control, Pose2d position, double heading){

        double[] con = {control.getX(),control.getY()};
        RealVector u = new ArrayRealVector(con);

        double[] pos = {position.getX(),position.getY()};
        RealVector z = new ArrayRealVector(pos);

        // predict the state estimate one time-step ahead
        filter.predict(u);

        // correct the state estimate with the latest measurement
        filter.correct(z);

        //get the corrected state - the position
        double pX = filter.getStateEstimation()[0];
        double pY = filter.getStateEstimation()[1];

        return new Pose2d(pX, pY, heading);
    }
}