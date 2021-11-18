package org.wolfcorp.ff.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.wolfcorp.ff.robot.DriveConstants;

/**
 * Storage for states that should persist during the duration of the match.
 * This allows different classes/opmodes to set and read from a central source of truth.
 */
public final class Match {
    // Disallow instantiation
    private Match() {}

    // Temporary values to prevent errors if Tele-op is run without Autonomous
    // Will be replaced with actual locations when autonomous runs
    public static Pose2d teleOpInitialPose = new Pose2d(12, 72 - DriveConstants.WIDTH / 2, Math.toRadians(180));
    public static Pose2d hubPose = new Pose2d(-12, 72 - DriveConstants.WIDTH / 2, Math.toRadians(180));
    public static boolean isRed = true;
}