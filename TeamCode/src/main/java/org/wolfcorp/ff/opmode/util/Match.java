package org.wolfcorp.ff.opmode.util;

import static org.wolfcorp.ff.opmode.AutonomousMode.pos;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.opmode.TeleOpMode;
import org.wolfcorp.ff.robot.DriveConstants;

/**
 * Storage for states that persist for the duration of the match.
 * This allows different classes/OpModes to set and read from a central source of truth.
 */
public final class Match {
    /**
     * Disallows instantiation.
     */
    private Match() {
    }

    // Initialized with temporary values to prevent errors if TeleOpMode is run without AutonomousMode.
    // Will be replaced with real values when AutonomousMode runs.

    /**
     * Whether we are in the red alliance. Initialized in {@link OpMode#OpMode()}.
     */
    public static boolean RED = false;
    /** Whether we are in the blue alliance. Initialized in {@link OpMode#OpMode()}. */
    public static boolean BLUE = true;
    /**
     * Where the robot starts in {@link TeleOpMode} according to the pose estimate from
     * {@link AutonomousMode}. Initialized in {@link AutonomousMode#runOpMode()}.
     */
    public static Pose2d teleOpInitialPose = pos(-72 + DriveConstants.WIDTH / 2, 50, 0);
    /**
     * Where the hub is supposed to be on the field. Initialized in {@link AutonomousMode#runOpMode()}
     */
    public static Pose2d hubPose = pos(-49 + DriveConstants.WIDTH / 2, -12, 90);
    /**
     * The telemetry object from the current {@link OpMode} instance. Initialized in
     * {@link OpMode#OpMode()}.
     */
    public static Telemetry telemetry = null;
    /** The Item instance that is linked with the OpMode status entry in telemetry. */
    public static Telemetry.Item statusItem = null;

    /**
     * Configures telemetry and sets up status. Call at the start of {@link AutonomousMode#runOpMode()}
     * to keep the OpMode status at top at all times.
     */
    public static void setupTelemetry() {
        if (statusItem == null) {
            telemetry.setMsTransmissionInterval(50); // faster telemetry
            telemetry.setAutoClear(false);
            telemetry.update();
            statusItem = createLogItem("OpMode - Status", "set up telemetry");
        }
    }

    /**
     * Updates OpMode status through telemetry immediately. Call {@link #setupTelemetry()}
     * before use.
     *
     * @param status current status of the OpMode
     * @see #setupTelemetry()
     */
    public static void status(String status) {
        statusItem.setValue(status);
        telemetry.update();
    }

    /**
     * Logs a one-line message immediately. Call {@link #setupTelemetry()} before use.
     *
     * @param message the message
     */
    public static void log(String message) {
        if (telemetry != null) {
            telemetry.log().add(message);
        }
    }

    /**
     * Creates a telemetry item (key-value pair) and gives it an initial value to display
     * immediately. Call {@link #setupTelemetry()} before use.
     *
     * @param key   the key (i.e., caption)
     * @param value the value
     * @param <T>   the type of the value
     * @return an item linked to the specified key-value pair; null if {@link #telemetry} is null
     */
    @Nullable
    public static <T> Telemetry.Item createLogItem(String key, T value) {
        if (telemetry != null) {
            Telemetry.Item item = telemetry.addData(key, value).setRetained(true);
            telemetry.update();
            return item;
        } else {
            return null;
        }
    }

    /**
     * Removes the telemetry item from driver station screen immediately. Call
     * {@link #setupTelemetry()} before use.
     *
     * @param item the telemetry item
     */
    public static void removeLogItem(Telemetry.Item item) {
        if (telemetry != null) {
            telemetry.removeItem(item);
            telemetry.update();
        }
    }

    /**
     * Updates telemetry. Call {@link #setupTelemetry()} before use.
     */
    public static void update() {
        if (telemetry != null) {
            telemetry.update();
        }
    }
}