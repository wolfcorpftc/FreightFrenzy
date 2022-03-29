package org.wolfcorp.ff.opmode.control;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.Kalman;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequenceRunner;

import java.util.HashMap;

import static java.lang.Math.cos;
import static org.wolfcorp.ff.opmode.util.Match.RED;

public class KalmanDemo extends AutonomousMode {

    protected Pose2d predictedPos;
    protected Pose2d encoderPos = pos(0,0);
    protected Pose2d sensorPos;
    protected Pose2d pastSensorPos;
    protected double time = 0;

    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initHardware();
        drive.setPoseEstimate(pos(0, 0));
        Kalman kalman = new Kalman();
        waitForStart();
        pastSensorPos = localizeWarehouseReturn();
        telemetry.setAutoClear(true);
        while (opModeIsActive()) {
            Match.status("Looping");

            Pose2d testPos = drive.getLocalizer().getPoseEstimate().minus(encoderPos);
            encoderPos = drive.getLocalizer().getPoseEstimate();
            sensorPos = localizeWarehouseReturn();
            if(sensorPos.minus(pastSensorPos).vec().norm() < 5) {
                predictedPos = kalman.estimatePosition(testPos, sensorPos, drive.getExternalHeading());
                drive.setPoseEstimate(predictedPos);
            } else {
                predictedPos = kalman.estimatePosition(testPos, sensorPos, drive.getExternalHeading());
                drive.setPoseEstimate(predictedPos);
            }

            drive.update();
            HashMap packet = TrajectorySequenceRunner.packetData;

            packet.put("INFARED", infaredDistanceSensor.getDistance(DistanceUnit.INCH));
            packet.put("Encoder X", encoderPos.getX());
            packet.put("Encoder Y", encoderPos.getY());
            packet.put("Range X", sensorPos.getX());
            packet.put("Range Y", sensorPos.getY());
            packet.put("Predicted X", predictedPos.getX());
            packet.put("Predicted Y", predictedPos.getY());

            telemetry.addData("Encoder", encoderPos);
            telemetry.addData("Range", sensorPos);
            telemetry.addData("Kalman", predictedPos);
            telemetry.update();
        }
    }


    protected Pose2d localizeWarehouseReturn() {
        // FIXME: replicate all changes to localize* (Hub, Carousel?)
        double heading = drive.getExternalHeading();

        // horizontal distance from wall to sensor
        double wallToXSensor = getCorrectedXReading() * cos(heading);
        // sensor point to robot center point
        Vector2d xSensorToRobot = (RED ? new Vector2d(-5.5, 7.5) : new Vector2d(5.625, -6.916)).rotated(heading);
        // horizontal distance between wall and robot center point
        double xDist;
        if (RED) {
            xDist = -new Vector2d(-wallToXSensor, 0).plus(xSensorToRobot).getX();
        } else {
            xDist = new Vector2d(wallToXSensor, 0).plus(xSensorToRobot).getX();
        }

        // same logic below
        double wallToYSensor = getCorrectedYReading() * cos(heading);
        Vector2d ySensorToRobot = new Vector2d(3.75, -6.75).rotated(heading);
        double yDist = new Vector2d(0, -wallToYSensor).plus(ySensorToRobot).getY();

        Vector2d correctedVec = pos(-72 + xDist, 72 + yDist).vec();
        if (Double.isInfinite(correctedVec.getX()) || Double.isInfinite(correctedVec.getY()) || Double.isNaN(correctedVec.getX()) || Double.isNaN(correctedVec.getY())){
            return drive.getPoseEstimate();
        }
        if (Math.abs(correctedVec.getX()) < 72 && Math.abs(correctedVec.getY()) < 72 && Math.abs(drive.getPoseEstimate().getX()) < Math.abs(correctedVec.getX())) {
            return new Pose2d(correctedVec.getX(), correctedVec.getY(), heading);
        } else {
            return new Pose2d(correctedVec.getX(), correctedVec.getY(), heading);
        }
    }

}
