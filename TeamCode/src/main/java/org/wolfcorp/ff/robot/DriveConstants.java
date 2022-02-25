package org.wolfcorp.ff.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * Constants generated by LearnRoadRunner.com/drive-constants
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 *  fields may also be edited through the dashboard (connect to the robot's WiFi network and
 *  navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 *  adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {
    /** Motor spec: ticks per revolution */
    public static final double TICKS_PER_REV = 537.7;
    /** Motor spec: maximum RPM */
    public static final double MAX_RPM = 312;

    /**
     * Set to true to enable built-in hub velocity control using drive encoders.
     * Set to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     */
    public static final boolean RUN_USING_ENCODER = true;
    /**
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from {@link org.wolfcorp.ff.robot.opmode.DriveVelocityPIDTuner}.
     */
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20, 0, 0,
            15);
            // getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    /** Radius of the drive wheels in inches. */
    public static double WHEEL_RADIUS = 1.88976; // in
    /** Ticks per inch of wheel movement on the ground. */
    public static double TICKS_PER_INCH = TICKS_PER_REV / (2 * Math.PI * WHEEL_RADIUS);
    /** output (wheel) speed / input (motor) speed */
    public static double GEAR_RATIO = 1;
    /** Width of the robot from the center of wheel to wheel */
    public static double TRACK_WIDTH = 9.75; // in
    /** Total width of the robot. The distance between the points that contacts the wall. */
    public static double WIDTH = 11.25;
    /** Total length of the robot. The distance between the points that contacts the wall. */
    public static double LENGTH = 15.25;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    /*
     * Note from LearnRoadRunner.com:
     * The velocity and acceleration constraints were calculated based on the following equation:
     * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85
     * Resulting in 52.48180821614297 in/s.
     * This is only 85% of the theoretical maximum velocity of the bot, following the recommendation above.
     * This is capped at 85% because there are a number of variables that will prevent your bot from actually
     * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiencies, etc.
     * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically
     * max velocity. The theoretically maximum velocity is 61.74330378369762 in/s.
     * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
     * affected if it is aiming for a velocity not actually possible.
     *
     * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
     * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
     * to degrade. As of now, it simply mirrors the velocity, resulting in 52.48180821614297 in/s/s
     *
     * Maximum Angular Velocity is calculated as: maximum velocity / trackWidth * (180 / Math.PI) but capped at 360°/s.
     * You are free to raise this on your own if you would like. It is best determined through experimentation.

     */
    /** Maximum velocity */
    public static double MAX_VEL = 40;
    /** Maximum acceleration */
    public static double MAX_ACCEL = 35;
    /** Maximum angular velocity */
    public static double MAX_ANG_VEL = 5.5;
    /** Maximum angular acceleration */
    public static double MAX_ANG_ACCEL = 5.5;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}