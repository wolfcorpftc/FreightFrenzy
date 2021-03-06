package org.wolfcorp.ff.robot.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ff.robot.Drivetrain;

@Autonomous(group = "drive", name = "Spin wheel")
public class SpinWheel extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            drive.leftBack.setPower(0.5);
        }
        drive.leftBack.setPower(0);
    }
}
