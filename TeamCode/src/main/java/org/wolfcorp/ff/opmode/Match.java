package org.wolfcorp.ff.opmode;

import static org.wolfcorp.ff.opmode.AutonomousMode.pos;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.wolfcorp.ff.robot.DriveConstants;

/**
 * Storage for states that should persist during the duration of the match.
 * This allows different classes/OpModes to set and read from a central source of truth.
 */
public final class Match {
    /**
     * Disallows instantiation.
     */
    private Match() {
    }

    // Temporary values to prevent errors if Tele-op is run without Autonomous
    // Will be replaced with actual locations when autonomous runs
    /** Whether the robot is in the red alliance. This member must be declared and initialized first. */
    public static boolean isRed = false;
    /** Where the robot starts in {@link TeleOpMode} according to the pose estimate from {@link AutonomousMode}. */
    public static Pose2d teleOpInitialPose = pos(-72 + DriveConstants.WIDTH / 2, 50, 0);
    /** Where the hub is supposed to be on the field. */
    public static Pose2d hubPose = pos(-49 + DriveConstants.WIDTH / 2, -12, 90);
    /** The telemetry object from the current {@link OpMode} instance. */
    public static Telemetry telemetry;
    /** The item instance that is linked with the OpMode status entry in telemetry. */
    public static Telemetry.Item statusItem;

    /**
     * Configures the telemetry object. This method will be automatically called in other logging
     * methods in {@link Match} if not called at the start of {@link AutonomousMode#runOpMode()}
     * (which is recommended to keep the OpMode status at top at all times).
     */
    static public void setupTelemetry() {
        if (statusItem == null) {
            telemetry.setMsTransmissionInterval(50); // faster telemetry
            telemetry.setAutoClear(false);
            telemetry.update();
            statusItem = createLogItem("OpMode - Status", "set up telemetry");
        }
    }

    /**
     * Update OpMode status through telemetry immediately.
     *
     * @param status current status of the OpMode
     */
    static public void status(String status) {
        setupTelemetry();
        statusItem.setValue(status);
        telemetry.update();
    }

    /**
     * Logs a one line message immediately.
     *
     * @param message the message
     */
    static public void log(String message) {
        setupTelemetry();
        telemetry.log().add(message);
    }

    /**
     * Creates a telemetry item (key-value pair) and gives it an initial value to display immediately.
     *
     * @param key   the key (i.e., caption)
     * @param value the value
     * @param <T>   the type of the value
     * @return an item linked to the specified key-value pair
     */
    static public <T> Telemetry.Item createLogItem(String key, T value) {
        setupTelemetry();
        Telemetry.Item item = telemetry.addData(key, value).setRetained(true);
        telemetry.update();
        return item;
    }

    /**
     * Removes the telemetry item from telemetry immediately.
     *
     * @param item the telemetry item
     */
    static public void removeLogItem(Telemetry.Item item) {
        setupTelemetry();
        telemetry.removeItem(item);
        telemetry.update();
    }
}