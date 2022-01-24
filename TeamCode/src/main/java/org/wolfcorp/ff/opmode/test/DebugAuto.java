package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.robot.DriveConstants.TRACK_WIDTH;
import static org.wolfcorp.ff.robot.DriveConstants.WIDTH;
import static org.wolfcorp.ff.robot.Drivetrain.getAccelerationConstraint;
import static org.wolfcorp.ff.robot.Drivetrain.getVelocityConstraint;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.EMPTY;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.FULL;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.OVERFLOW;
import static org.wolfcorp.ff.vision.Barcode.EXCESS;
import static org.wolfcorp.ff.vision.Barcode.ZERO;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.opmode.util.TimedController;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.DumpIndicator;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.vision.Barcode;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "Debug", group = "test")
public class DebugAuto extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initHardware();
        waitForStart();
        drive.forward(0.5, 12);
    }

    public void runOpMode2() {
        Match.setupTelemetry();
        initHardware();
        Match.log("Position: " + outtake.getServo().getPosition());
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        DumpIndicator.Status lastStatus = null;
        DumpIndicator.Status status;
        INTAKE_LOOP:
        while (isActive()) {
            status = dumpIndicator.update();
            if (lastStatus != status) {
                timer.reset();
            }
            switch (status) {
                case EMPTY:
                    outtake.dumpIn();
                    if (!outtake.willBeAt(ZERO)) {
                        outtake.slideToAsync(ZERO);
                    } else {
                        Match.status("Intaking...");
                        timer.reset();
                        intake.in();
                    }
                    break;
                case FULL:
                    outtake.dumpIn();
                    if (!outtake.willBeAt(ZERO)) {
                        outtake.slideToAsync(ZERO);
                    } else if (timer.seconds() > 0.3) {
                        Match.status("Full!");
                        intake.out();
                    } else if (timer.seconds() > 0.6) {
                        break INTAKE_LOOP;
                    }
                    break;
                case OVERFLOW:
                    if (timer.seconds() > 0.3 && timer.seconds() < 2.5) {
                        Match.status("Ridding excess freight...");
                        intake.out();
                        if (!outtake.willBeAt(EXCESS)) {
                            outtake.slideToAsync(Barcode.EXCESS);
                        }
                        outtake.dumpExcess();
                    } else if (timer.seconds() > 2.5) {
                        intake.out();
                        outtake.dumpIn();
                        if (!outtake.willBeAt(ZERO)) {
                            outtake.slideToAsync(ZERO);
                        }
                    }
                    break;
            }
            lastStatus = status;
        }
    }
}
