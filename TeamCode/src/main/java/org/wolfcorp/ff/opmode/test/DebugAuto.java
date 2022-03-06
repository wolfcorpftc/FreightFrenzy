package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.vision.Barcode.EXCESS;
import static org.wolfcorp.ff.vision.Barcode.ZERO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.DumpIndicator;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.vision.Barcode;

@Autonomous(name = "Debug", group = "test")
public class DebugAuto extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initHardware();
        waitForStart();
        while (opModeIsActive())
            intake.getMotor().setVelocity(0.75 * Intake.IN_SPEED);
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
                    if (!outtake.isApproaching(ZERO)) {
                        outtake.slideToAsync(ZERO);
                    } else {
                        Match.status("Intaking...");
                        timer.reset();
                        intake.in();
                    }
                    break;
                case FULL:
                    outtake.dumpIn();
                    if (!outtake.isApproaching(ZERO)) {
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
                        if (!outtake.isApproaching(EXCESS)) {
                            outtake.slideToAsync(Barcode.EXCESS);
                        }
                        outtake.dumpExcess();
                    } else if (timer.seconds() > 2.5) {
                        intake.out();
                        outtake.dumpIn();
                        if (!outtake.isApproaching(ZERO)) {
                            outtake.slideToAsync(ZERO);
                        }
                    }
                    break;
            }
            lastStatus = status;
        }
    }
}
