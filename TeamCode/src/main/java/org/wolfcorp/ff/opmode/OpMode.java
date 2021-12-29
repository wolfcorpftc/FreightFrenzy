package org.wolfcorp.ff.opmode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.DumpIndicator;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.robot.Outtake;

public abstract class OpMode extends LinearOpMode {
    // public is fine :)
    public static Drivetrain drive = null;
    public static Intake intake = null;
    public static Outtake outtake = null;
    public static CarouselSpinner spinner = null;
    public static DumpIndicator dumpIndicator = null;
    public static DistanceSensor rangeSensor;
    public static RevColorSensorV3 upperDumpDistance = null;
    public static RevColorSensorV3 lowerDumpDistance = null;
    public static DistanceSensor intakeRampDistance = null;

    /**
     * Based on the OpMode name, initialize {@link Match} members appropriately.
     */
    public OpMode() {
        Match.statusItem = null; // allows previous telemetry object to be GC'd (not sure if needed)
        Match.telemetry = telemetry; // don't do any additional config here, see Match#setupTelemetry
        Match.RED = modeNameContains("Red");
    }

    /**
     * Tests whether the current OpMode name contains given substring.
     *
     * @param searchTerm the search term
     * @return whether the OpMode name contains the search term
     */
    public boolean modeNameContains(String searchTerm) {
        return this.getClass().getSimpleName().contains(searchTerm);
    }

    /**
     * Initializes robot hardware. Run after {@link Match#setupTelemetry()}.
     */
    protected void initHardware() {
        Match.status("Initializing robot hardware");

        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        spinner = new CarouselSpinner(hardwareMap, this::sleep);
        dumpIndicator = new DumpIndicator(hardwareMap);
        rangeSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");
        upperDumpDistance = hardwareMap.get(RevColorSensorV3.class, "upperDumpDist");
        lowerDumpDistance = hardwareMap.get(RevColorSensorV3.class, "lowerDumpDist");
        intakeRampDistance = hardwareMap.get(DistanceSensor.class, "intakeDist");

        Match.status("Initialized robot hardware");
    }

    /**
     * Resets robot hardware fields to null to allow garbage collection. Run at the end of
     * {@link OpMode#runOpMode()}.
     */
    protected void resetHardware() {
        Match.status("Resetting robot hardware...");

        drive = null;
        intake = null;
        outtake = null;
        spinner = null;
        dumpIndicator = null;
        rangeSensor = null;
        upperDumpDistance = null;
        lowerDumpDistance = null;
        intakeRampDistance = null;

        Match.status("Done resetting robot hardware");
    }
}
