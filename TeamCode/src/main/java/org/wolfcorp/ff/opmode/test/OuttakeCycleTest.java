package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.vision.Barcode;

@TeleOp(name = "Outtake Cycle Test", group = "^testing")
public class OuttakeCycleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Outtake outtake = new Outtake(hardwareMap);
        waitForStart();
        outtake.cycleAsync(Barcode.TOP);
        while (opModeIsActive()) {
            telemetry.addData("", outtake.getMotor().getCurrentPosition());
            telemetry.update();
            sleep(50);
        }
    }
}
