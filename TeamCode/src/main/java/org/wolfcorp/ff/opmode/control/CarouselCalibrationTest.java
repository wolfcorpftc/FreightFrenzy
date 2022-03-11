package org.wolfcorp.ff.opmode.control;

import static org.wolfcorp.ff.opmode.util.Match.RED;
import static java.lang.Math.cos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.util.InchSensor;

public class CarouselCalibrationTest extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        telemetry.setAutoClear(true);
        initHardware();
        drive.setPoseEstimate(trueCarouselPose);
        Match.status("Waiting for start");
        waitForStart();

        while (opModeIsActive()) {
            drive.update();

            double heading = drive.getExternalHeading();

            InchSensor ySensor = RED ? rightRangeSensor : leftRangeSensor;
            double wallToYSensor = ySensor.get() * cos(heading);
            Vector2d ySensorToRobot = (RED ? new Vector2d(-5.5, 7.5) : new Vector2d(5.25, -7.25)).rotated(heading);
            double yDist = new Vector2d(0, wallToYSensor).plus(ySensorToRobot).getY();

            double wallToXSensor = rangeSensor.get() * cos(heading);
            Vector2d xSensorToRobot = new Vector2d(-2, -6.5).rotated(heading);
            double xDist;
            if (RED) {
                xDist = -new Vector2d(-wallToXSensor, 0).plus(xSensorToRobot).getX();
            } else {
                xDist = new Vector2d(wallToXSensor, 0).plus(xSensorToRobot).getX();
            }

            Vector2d correctedVec = pos(-72 + xDist, -72 + yDist).vec();
            if (Math.abs(correctedVec.getX()) < 72 && Math.abs(correctedVec.getY()) < 72) {
                drive.setPoseEstimate(new Pose2d(correctedVec.getX(), correctedVec.getY(), heading));
            }
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("x sensor", rangeSensor.get());
            telemetry.addData("y sensor", ySensor.get());
            telemetry.addData("x dist", xDist);
            telemetry.addData("y dist", yDist);
            telemetry.addData("Corrected Vector", correctedVec);
            telemetry.update();
        }
    }
}
