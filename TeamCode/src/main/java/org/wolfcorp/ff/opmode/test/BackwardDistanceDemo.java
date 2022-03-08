package org.wolfcorp.ff.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.AutonomousMode;

public class BackwardDistanceDemo extends AutonomousMode {
    @Override
    public void runOpMode() {

        Telemetry graphData = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.setMotorPowers(0.2);
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            graphData.addData("Edge distance", rangeSensor.getDistance(DistanceUnit.INCH));
            graphData.addData("Center distance", rangeSensor.getDistance(DistanceUnit.INCH) + 6.5);
            graphData.update();
            if (rangeSensor.getDistance(DistanceUnit.INCH) + 6.5 > 29){
                drive.setMotorPowers(0);
                break;
            }
        }

    }
}
