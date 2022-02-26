package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.vision.Barcode.EXCESS;
import static org.wolfcorp.ff.vision.Barcode.ZERO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.DumpIndicator;
import org.wolfcorp.ff.vision.Barcode;

@Autonomous(name = "Debug", group = "test")
public class DebugAuto extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        telemetry.setAutoClear(true);
        Match.RED = false;
        Match.BLUE = true;
        initHardware();
        Match.status("Waiting for start");
        waitForStart();
//        Match.status("Measuring distance...");

//        Telemetry.Item rawDist = Match.createLogItem("Raw Distance", 0);
//        Telemetry.Item yEstimate = Match.createLogItem("Y Estimate", 0);

        while (opModeIsActive()) {
            telemetry.addData("LF Position", drive.leftFront.getCurrentPosition());
            telemetry.addData("LB Position", drive.leftBack.getCurrentPosition());
            telemetry.addData("RF Position", drive.rightFront.getCurrentPosition());
            telemetry.addData("RB Position", drive.rightBack.getCurrentPosition());
//            rawDist.setValue(rangeSensor.getDistance(DistanceUnit.INCH));
//            yEstimate.setValue(72 - rangeSensor.getDistance(DistanceUnit.INCH) - (Match.RED ? 6 : 6.5));
            Match.update();
        }
    }
}
