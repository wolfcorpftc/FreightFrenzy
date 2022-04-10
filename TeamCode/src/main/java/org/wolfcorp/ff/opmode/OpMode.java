package org.wolfcorp.ff.opmode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.DumpIndicator;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.robot.ShippingArm;
import org.wolfcorp.ff.robot.util.InchSensor;

public abstract class OpMode extends LinearOpMode {
    /** Initialized in {@link #OpMode()} and reset in {@link #resetHardware()} */
    private static OpMode instance = null;

    // public is fine :)
    public static Drivetrain drive = null;
    public static Intake intake = null;
    public static Outtake outtake = null;
    public static CarouselSpinner spinner = null;
    public static ShippingArm shippingArm = null;
    public static DumpIndicator dumpIndicator = null;
    public static InchSensor infaredDistanceSensor;
    public static InchSensor rangeSensor;
    public static InchSensor leftRangeSensor;
    public static InchSensor rightRangeSensor;
    public static RevColorSensorV3 upperDumpDistance = null;
    public static RevColorSensorV3 lowerDumpDistance = null;
    public static InchSensor intakeRampDistance = null;

    /**
     * Based on the OpMode name, initialize {@link Match} members appropriately.
     */
    public OpMode() {
        Match.statusItem = null; // allows previous telemetry object to be GC'd (not sure if needed)
        Match.telemetry = telemetry; // don't do any additional config here, see Match#setupTelemetry
        Match.RED = modeNameContains("Red");
        Match.BLUE = !Match.RED;
        instance = this;
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
        shippingArm = new ShippingArm(hardwareMap);
        dumpIndicator = new DumpIndicator(hardwareMap);
        infaredDistanceSensor = new InchSensor(hardwareMap.get(DistanceSensor.class, "frontDist"));
        rangeSensor = new InchSensor(hardwareMap.get(DistanceSensor.class, "sensor_range"));
        if (Match.BLUE)
            leftRangeSensor = new InchSensor(hardwareMap.get(DistanceSensor.class, "leftRangeSensor"));
        else if (Match.RED)
            rightRangeSensor = new InchSensor(hardwareMap.get(DistanceSensor.class, "rightRangeSensor"));
        upperDumpDistance = hardwareMap.get(RevColorSensorV3.class, "upperDumpDist");
        upperDumpDistance.enableLed(false);
        lowerDumpDistance = hardwareMap.get(RevColorSensorV3.class, "lowerDumpDist");
        lowerDumpDistance.enableLed(false);
        intakeRampDistance = new InchSensor(hardwareMap.get(DistanceSensor.class, "intakeDist"));

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
        leftRangeSensor = null;
        rightRangeSensor = null;
        upperDumpDistance = null;
        lowerDumpDistance = null;
        intakeRampDistance = null;

        instance = null;

        Match.status("Done resetting robot hardware");
    }

    public static boolean isActive() {
        return instance != null && instance.opModeIsActive() && !Thread.currentThread().isInterrupted();
    }
}
