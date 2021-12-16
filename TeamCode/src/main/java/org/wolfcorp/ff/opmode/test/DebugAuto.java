package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.Match;
import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.Intake;

@Autonomous(name = "Debug", group = "test")
public class DebugAuto extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
//        Match.setupTelemetry();
//        Match.status("Initializing");
//        Intake intake = new Intake(hardwareMap);
//        queue(fromHere().strafeLeft(3));
//        queue(() -> intake.in(5));
//        Match.status("Ready for start");
//        waitForStart();
//        runTasks();
        drive = new Drivetrain(hardwareMap);
        drive.setPoseEstimate(initialPose);
        waitForStart();
        queue(fromHere().forward(3));
        runTasks();
    }
}
