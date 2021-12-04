package org.wolfcorp.ff.opmode;

import static org.wolfcorp.ff.opmode.AutonomousMode.pos;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.wolfcorp.ff.robot.DriveConstants;

/**
 * Storage for states that should persist during the duration of the match.
 * This allows different classes/opmodes to set and read from a central source of truth.
 */
public final class Match {
    /**
     * Disallow instantiation
     */
    private Match() {
    }

    // Temporary values to prevent errors if Tele-op is run without Autonomous
    // Will be replaced with actual locations when autonomous runs
    public static Pose2d teleOpInitialPose = pos(-72 + DriveConstants.WIDTH / 2, 50, 0);
    public static Pose2d hubPose = pos(-49 + DriveConstants.WIDTH / 2, -12, 90);
    public static boolean isRed = false;
    public static Telemetry telemetry;
    public static Telemetry.Item statusItem;

    static public void setupTelemetry() {
        if (statusItem == null) {
            telemetry.setMsTransmissionInterval(50); // faster telemetry
            telemetry.setAutoClear(false);
            telemetry.update();
            statusItem = createLogItem("OpMode - Status", "set up telemetry");
        }
    }
    /**
     * Send a status message through telemetry. Also sets up telemetry if statusItem is not
     * initialized (assuming first call to status is at the beginning of every OpMode).
     * @param status
     */
    static public void status(String status) {
        statusItem.setValue(status);
        telemetry.update();
    }
    static public void log(String message) {
        telemetry.log().add(message);
    }
    static public <T> Telemetry.Item createLogItem(String message, T value) {
        Telemetry.Item item = telemetry.addData(message, value).setRetained(true);
        telemetry.update();
        return item;
    }
    static public void removeLogItem(Telemetry.Item item) {
        telemetry.removeItem(item);
        telemetry.update();
    }
}