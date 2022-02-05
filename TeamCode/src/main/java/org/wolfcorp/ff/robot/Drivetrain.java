package org.wolfcorp.ff.robot;

import static org.wolfcorp.ff.robot.DriveConstants.MAX_ACCEL;
import static org.wolfcorp.ff.robot.DriveConstants.MAX_ANG_ACCEL;
import static org.wolfcorp.ff.robot.DriveConstants.MAX_ANG_VEL;
import static org.wolfcorp.ff.robot.DriveConstants.MAX_RPM;
import static org.wolfcorp.ff.robot.DriveConstants.MAX_VEL;
import static org.wolfcorp.ff.robot.DriveConstants.MOTOR_VELO_PID;
import static org.wolfcorp.ff.robot.DriveConstants.RUN_USING_ENCODER;
import static org.wolfcorp.ff.robot.DriveConstants.TICKS_PER_REV;
import static org.wolfcorp.ff.robot.DriveConstants.TRACK_WIDTH;
import static org.wolfcorp.ff.robot.DriveConstants.encoderTicksToInches;
import static org.wolfcorp.ff.robot.DriveConstants.kA;
import static org.wolfcorp.ff.robot.DriveConstants.kStatic;
import static org.wolfcorp.ff.robot.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequence;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequenceBuilder;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequenceRunner;
import org.wolfcorp.ff.robot.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@SuppressWarnings("unused")
public class Drivetrain extends MecanumDrive {
    public static double SLAM_ERROR = 5;
    public static double SLAM_FORWARD = 18;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(6, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    /** { {@link #leftFront}, {@link #leftBack}, {@link #rightBack}, {@link #rightFront} } */
    private final List<DcMotorEx> motors;

    /** If true, the current trajectory will be aborted. This will be reset after a new one is added */
    private boolean abort = false;

    private final BNO055IMU imu;
    private final VoltageSensor batteryVoltageSensor;

    /** Speed multiplier for {@link #drive(double, double, double, double, boolean)}*/
    public double speedMultiplier = 1;

    public Drivetrain(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        TrajectoryFollower follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder from(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                from(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void turnAsyncDeg(double deg) {
        turnAsync(Math.toRadians(deg));
    }

    public void turnDeg(double deg) {
        turn(Math.toRadians(deg));
    }

    public void turnToDeg(double deg) {
        turnToDegAsync(deg);
        waitForIdle();
    }

    public void turnToDegAsync(double deg) {
        double angle = deg - getExternalHeadingDeg();
        if (Math.abs(angle) < 1) {
            setMotorPowers(0, 0, 0, 0);
            return;
        }
        double power = Math.max(0.05, Math.min(0.5, Math.abs(angle) / 100));
        if (angle > 0) {
            setMotorPowers(-power, -power, power, power);
        } else {
            setMotorPowers(power, power, -power, -power);
        }
    }

    public double getExternalHeadingDeg() {
        return Math.toDegrees(getExternalHeading());
    }

    public void setExternalHeadingDeg(double deg) {
        setExternalHeading(Math.toRadians(deg));
    }

    public void followAsync(Trajectory trajectory) {
        abort = false;
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                from(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void follow(Trajectory trajectory) {
        followAsync(trajectory);
        waitForIdle();
    }

    public void followAsync(TrajectorySequence trajectorySequence) {
        abort = false;
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void follow(TrajectorySequence trajectorySequence) {
        followAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void abort() {
        abort = true;
    }

    public void update() {
        updatePoseEstimate();
        if (abort) {
            trajectorySequenceRunner.followTrajectorySequenceAsync(null);
            return;
        }
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();

        trajectorySequenceRunner.followTrajectorySequenceAsync(null);
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public boolean motorBusy() {
        return leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy();
    }

    public boolean allMotorsBusy() {
        return leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double lf, double lb, double rb, double rf) {
        if (RUN_USING_ENCODER) {
            leftFront.setVelocity(lf * MAX_RPM * TICKS_PER_REV / 60);
            leftBack.setVelocity(lb * MAX_RPM * TICKS_PER_REV / 60);
            rightBack.setVelocity(rb * MAX_RPM * TICKS_PER_REV / 60);
            rightFront.setVelocity(rf * MAX_RPM * TICKS_PER_REV / 60);
        } else {
            leftFront.setPower(lf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);
            rightFront.setPower(rf);
        }
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++) {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }

    public void setMotorPowers(double v) {
        this.setMotorPowers(v, v, v, v);
    }

    public void setMotorVelocity(double v) {
        leftFront.setVelocity(v);
        leftBack.setVelocity(v);
        rightFront.setVelocity(v);
        rightBack.setVelocity(v);
    }

    public void drive(double x, double y, double rotation, double slowModeSpeed, boolean slowModeCondition) {

        double[] wheelSpeeds = new double[4];

        // wheelSpeeds[0] = x * (slowModeCondition ? smX : 1) + y * (slowModeCondition ? smY : 1) + rotation * (slowModeCondition ? smH : 1); // LF
        // wheelSpeeds[1] = x * (slowModeCondition ? smX : 1) - y * (slowModeCondition ? smY : 1) - rotation * (slowModeCondition ? smH : 1); // RF
        // wheelSpeeds[2] = x * (slowModeCondition ? smX : 1) - y * (slowModeCondition ? smY : 1) + rotation * (slowModeCondition ? smH : 1); // LB
        // wheelSpeeds[3] = x * (slowModeCondition ? smX : 1) + y * (slowModeCondition ? smY : 1) - rotation * (slowModeCondition ? smH : 1); // RB

        // FIXME: wheelSpeeds doesn't match setMotorPowers
        wheelSpeeds[0] = x + y + rotation; // LF
        wheelSpeeds[1] = x - y - rotation; // RF
        wheelSpeeds[2] = x - y + rotation; // LB
        wheelSpeeds[3] = x + y - rotation; // RB

        normalize(wheelSpeeds);

        for (int i = 0; i < 4; i++) {
            wheelSpeeds[i] = Math.pow(wheelSpeeds[i], 3);
            if (wheelSpeeds[i] > -0.25 && wheelSpeeds[i] < 0) {
                wheelSpeeds[i] = -0.25;
            } else if (wheelSpeeds[i] < 0.25 && wheelSpeeds[i] > 0) {
                wheelSpeeds[i] = 0.25;
            }
        }
        normalize(wheelSpeeds);

        for (int i = 0; i < 4; i++)
            wheelSpeeds[i] *= speedMultiplier * (slowModeCondition ? slowModeSpeed : 1);

        setMotorPowers(wheelSpeeds[0], wheelSpeeds[2], wheelSpeeds[3], wheelSpeeds[1]);
    }

    public void turnForward(boolean condition) {
        turnToDeg(0);
    }

    public void setDriveTargetPos(int lf, int rf, int lb, int rb) {
        leftFront.setTargetPosition(lf);
        rightFront.setTargetPosition(rf);
        leftBack.setTargetPosition(lb);
        rightBack.setTargetPosition(rb);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position (unless timeout has been reached)
     *  2) Driver stops the opmode running.
     */
    public void drive(double speed, double lf, double rf, double lb, double rb) {
        int lfTarget = leftFront.getCurrentPosition() + (int) (lf * DriveConstants.TICKS_PER_INCH);
        int rfTarget = rightFront.getCurrentPosition() + (int) (rf * DriveConstants.TICKS_PER_INCH);
        int lbTarget = leftBack.getCurrentPosition() + (int) (lb * DriveConstants.TICKS_PER_INCH);
        int rbTarget = rightBack.getCurrentPosition() + (int) (rb * DriveConstants.TICKS_PER_INCH);
        Match.log("1");
        setDriveTargetPos(lfTarget, rfTarget, lbTarget, rbTarget);
        Match.log("2");

        DcMotor.RunMode originalMode = leftFront.getMode();
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Match.log("3");
        setMotorVelocity(Math.abs(speed * MAX_RPM * TICKS_PER_REV / 60));

        while (OpMode.isActive() && allMotorsBusy()) {
            updatePoseEstimate();
        }
        Match.log("4");

        setMotorVelocity(0);
        setMode(originalMode);
    }

    public void forward(double speed, double distance) {
        drive(speed, distance, distance, distance, distance);
    }

    public void backward(double speed, double distance) {
        drive(speed, -distance, -distance, -distance, -distance);
    }

    public void turnLeft(double speed, double degrees) {
        drive(speed, degrees, -degrees, degrees, -degrees);
    }

    public void turnRight(double speed, double degrees) {
        drive(speed, -degrees, degrees, -degrees, degrees);
    }

    public void strafeRight(double speed, double distance) {
        drive(speed, distance, -distance, -distance, distance);
    }

    public void strafeLeft(double speed, double distance) {
        drive(speed, -distance, distance, distance, -distance);
    }

    // speed = 1 versions

    public void forward(double distance) {
        drive(1, distance, distance, distance, distance);
    }

    public void backward(double distance) {
        drive(1, -distance, -distance, -distance, -distance);
    }

    public void turnLeft(double degrees) {
        drive(1, degrees, -degrees, degrees, -degrees);
    }

    public void turnRight(double degrees) {
        drive(1, -degrees, degrees, -degrees, degrees);
    }

    public void strafeRight(double distance) {
        drive(1, distance, -distance, -distance, distance);
    }

    public void strafeLeft(double distance) {
        drive(1, -distance, distance, distance, -distance);
    }

    public void calibrateX(Pose2d calibratedX) {
        Pose2d current = getPoseEstimate();
        setPoseEstimate(new Pose2d(calibratedX.getX(), current.getY(), current.getHeading()));
    }

    public void calibrateY(Pose2d calibratedY) {
        Pose2d current = getPoseEstimate();
        setPoseEstimate(new Pose2d(current.getX(), calibratedY.getY(), current.getHeading()));
    }
}
