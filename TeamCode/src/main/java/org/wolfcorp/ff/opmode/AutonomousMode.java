package org.wolfcorp.ff.opmode;

import static org.wolfcorp.ff.robot.DriveConstants.MAX_ACCEL;
import static org.wolfcorp.ff.robot.DriveConstants.MAX_ANG_VEL;
import static org.wolfcorp.ff.robot.DriveConstants.MAX_VEL;
import static org.wolfcorp.ff.robot.DriveConstants.TRACK_WIDTH;
import static org.wolfcorp.ff.robot.Drivetrain.getAccelerationConstraint;
import static org.wolfcorp.ff.robot.Drivetrain.getVelocityConstraint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.wolfcorp.ff.BuildConfig;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.DriveConstants;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequence;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequenceBuilder;
import org.wolfcorp.ff.vision.Barcode;
import org.wolfcorp.ff.vision.BarcodeScanner;
import org.wolfcorp.ff.vision.Guide;
import org.wolfcorp.ff.vision.TFWarehouseGuide;
import org.wolfcorp.ff.vision.VuforiaNavigator;
import org.wolfcorp.ff.vision.WarehouseGuide;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Supplier;

public abstract class AutonomousMode extends OpMode {
    // region Hardware
    protected Drivetrain drive = null;
    protected CarouselSpinner spinner = null;
    protected OpenCvCamera camera = null;
    protected Intake intake = null;
    protected Outtake outtake = null;
    // endregion

    // region Configuration
    public final boolean VISION = !this.getClass().getSimpleName().contains("NV");
    public final boolean CAROUSEL = this.getClass().getSimpleName().contains("Carousel");
    public final boolean WALL_RUNNER = this.getClass().getSimpleName().contains("WR");

    public static final int SCORING_CYCLES = 0;
    // endregion

    // region Vision Fields
    protected BarcodeScanner scanner;
    protected Guide guide;
    protected VuforiaNavigator navigator;
    protected Barcode barcode;
    // endregion

    // region Poses

    /*
     * Poses are declared in order of appearance in paths.
     * When initializing poses with pos(), assume that the robot starts at blue warehouse.
     * The front of robot is the carousel spinner (0 degree).
    */
    /** where the robot starts */
    protected Pose2d initialPose;

    /** y-coordinate calibration (beyond the wall) */
    protected Pose2d calibratePreCarouselPose;
    /** y-coordinate calibration (real pose) */
    protected Pose2d preCarouselPose;
    /** turn the carousel */
    protected Pose2d carouselPose;

    /** retrieve the left shipping element */
    protected Pose2d elementLeftPose;
    /** retrieve the middle shipping element */
    protected Pose2d elementMidPose;
    /** retrieve the right shipping element */
    protected Pose2d elementRightPose;

    /** between hub and wall; used for heading adjustment coming from carousel w/o hitting SE */
    protected Pose2d preHubPose;
    /** score freight into hub */
    protected Pose2d hubPose;
    /** x-coordinate calibration while cycling (beyond the wall) */
    protected Pose2d calibrateHubWallPose;
    /** x-coordinate calibration while cycling (real pose) */
    protected Pose2d hubWallPose;
    /** change heading after scoring */
    protected Pose2d postHubPose;
    /** used to avoid driving on the barrier / triangles & to start vision */
    protected Pose2d preWhPose;
    /** load freight from warehouse */
    protected Pose2d whPose;
    /** where the robot parks */
    protected Pose2d parkPose;
    // endregion

    // region Task Queue
    private final ArrayList<Object> tasks = new ArrayList<>();
    private final HashMap<String, Object> dynamicTasks = new HashMap<>();
    // endregion

    // region Robot Logic
    public AutonomousMode() {
        if (Match.isRed) {
            // NOTE: Assuming we are by the blue warehouse!
            initialPose = pos(
                    -72 + DriveConstants.WIDTH / 2,
                    24 - DriveConstants.LENGTH / 2 + (CAROUSEL ? 0 : -3),
                    180
            );
            carouselPose = pos(-53.15, -72 + DriveConstants.WIDTH / 2, 90);
        } else {
            initialPose = pos(-72 + DriveConstants.WIDTH / 2, DriveConstants.LENGTH / 2, 0);
            carouselPose = pos(-55, -72 + DriveConstants.WIDTH / 2, 90);
        }
        preCarouselPose = carouselPose.plus(pos(3, 0));
        calibratePreCarouselPose = preCarouselPose.minus(pos(0, 4));

        elementLeftPose = pos(-72 + DriveConstants.LENGTH / 2, 20.4, 180);
        elementMidPose = elementLeftPose.minus(pos(0, 8.4));
        elementRightPose = elementMidPose.minus(pos(0, 8.4));

        if (Match.isRed) {
            hubPose = pos(-51 + DriveConstants.WIDTH / 2, -12 + (CAROUSEL ? 0 : -3), 90);
        } else {
            hubPose = pos(-47 + DriveConstants.WIDTH / 2, -12 + (CAROUSEL ? 0 : -3), 90);
        }
        preHubPose = pos(-48, -12, 0);
        hubWallPose = pos(-72 + DriveConstants.WIDTH / 2, -12, 0);
        calibrateHubWallPose = hubWallPose.minus(pos(6, 0));
        preWhPose = pos(-72 + DriveConstants.WIDTH / 2, 12, 0);
        whPose = pos(-72 + DriveConstants.WIDTH / 2, 42, 0);

        if (WALL_RUNNER) {
            parkPose = pos(-72 + DriveConstants.WIDTH / 2, 38 + DriveConstants.LENGTH / 2, 0);
        } else {
            parkPose = pos(-36, 38 + DriveConstants.LENGTH / 2, 0);
        }

        if (CAROUSEL) {
            initialPose = initialPose.minus(pos(0, 48));

            elementLeftPose = elementLeftPose.minus(pos(0, 48));
            elementMidPose = elementMidPose.minus(pos(0, 48));
            elementRightPose = elementRightPose.minus(pos(0, 48));
        }

        // account for error in carousel scoring
        final double HUB_X_ERROR = CAROUSEL ? (Match.isRed ? 3 : -2) : 5;
        final double HUB_Y_ERROR = Match.isRed ? 5 : 0;
        hubPose = hubPose.plus(pos(HUB_X_ERROR, HUB_Y_ERROR));
        preHubPose = preHubPose.plus(pos(HUB_X_ERROR, HUB_Y_ERROR));
        hubWallPose = hubWallPose.plus(pos(HUB_X_ERROR, HUB_Y_ERROR));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Match.setupTelemetry();
        // *** Initialization ***
        Thread initVisionThread = new Thread(this::initVisionWebcam);
        if (VISION) {
            initVisionThread.start();
        }

        drive = new Drivetrain(hardwareMap);
        drive.setPoseEstimate(initialPose);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        spinner = new CarouselSpinner(hardwareMap, this::sleep);

        Match.status("Robot Initialized, preparing task queue");

        // *** Carousel ***
        if (CAROUSEL) {
            Match.status("Initializing: carousel");
            //queue(fromHere().lineToLinearHeading(preCarouselPose));
            queue(fromHere().lineToLinearHeading(calibratePreCarouselPose));
            queueYCalibration(preCarouselPose);
            queue(fromHere().lineTo(carouselPose.vec(), getVelocityConstraint(15, 5, TRACK_WIDTH), getAccelerationConstraint(15)));
            queue(fromHere().lineTo(carouselPose.minus(pos(0.5, 0)).vec(), getVelocityConstraint(10, 2, TRACK_WIDTH), getAccelerationConstraint(10)));
            queue((RobotRunnable) spinner::spin);
        }

        // *** Pre-loaded cube ***
        Match.status("Initializing: preloaded");
        if (CAROUSEL) {
            queue(fromHere().addTemporalMarker(1, () -> outtake.slideToAsync(barcode)).lineTo(preHubPose.vec()).lineTo(hubPose.vec()));
        } else {
            queue(fromHere().addTemporalMarker(1, () -> outtake.slideToAsync(barcode)).lineTo(preHubPose.vec()).lineToSplineHeading(hubPose));
        }
        queue(() -> {
            outtake.dumpOut();
            sleep(2400);
        });

        boolean isOuttakeReset = false;

        // *** Cycling ***
        for (int i = 1; i <= SCORING_CYCLES; i++) {
            Match.status("Initializing: cycle " + i);
            if (!isOuttakeReset) {
                queue(fromHere()
                        .now(outtake::dumpIn)
                        .now(() -> outtake.slideTo(Barcode.ZERO))
                        .lineToSplineHeading(preHubPose)
                        .lineTo(calibrateHubWallPose.vec()));
                isOuttakeReset = true;
            } else {
                queue(fromHere()
                        .lineToSplineHeading(preHubPose)
                        .lineTo(calibrateHubWallPose.vec()));
            }
            queueXCalibration(hubWallPose);
            queue(fromHere()
                    .lineTo(whPose.vec())
                    .now(intake::in)
                    .forward(3)
                    .waitSeconds(2)
                    .now(intake::out)
                    .back(3)
                    .now(intake::off)
                    .lineTo(hubWallPose.vec())
                    .lineTo(preHubPose.vec())
                    .lineToSplineHeading(hubPose)
            );
            queue(() -> {
                outtake.slideTo(Barcode.BOT);
                outtake.dumpOut();
                sleep(1000);
                outtake.dumpIn();
            });
        }

        // *** Park ***
        Match.status("Initializing: park");
        // TODO: move back
        queue(fromHere()
                .now(outtake::dumpIn)
                .now(() -> outtake.slideTo(Barcode.ZERO))
                .lineToSplineHeading(preHubPose, getVelocityConstraint(30, 5, TRACK_WIDTH), getAccelerationConstraint(15))
                .lineTo(calibrateHubWallPose.vec(), getVelocityConstraint(30, 5, TRACK_WIDTH), getAccelerationConstraint(15))
        );
        if (!isOuttakeReset) {
            isOuttakeReset = true;
        } else {
            queue(fromHere()
                    .lineToLinearHeading(preHubPose)
                    .lineTo(calibrateHubWallPose.vec()));
        }
        queueXCalibration(hubWallPose);
        queue(fromHere().lineTo(parkPose.vec(), getVelocityConstraint(30, 5, TRACK_WIDTH), getAccelerationConstraint(15)));

        // *** Wrapping Up ***
        if (VISION) {
            Match.status("Initializing: vision");
            initVisionThread.join();
            scanner.start();
        }
        Match.status("Task queue ready, waiting for start");

        // *** START ***
        waitForStart();
        // *** Scan Barcode ***
        if (VISION) {
            Match.status("Scanning");
            barcode = scanner.getBarcode();
            scanner.stop();
        }

        Match.status("Start!");

        runTasks();

        sleep(1000);
        Match.teleOpInitialPose = drive.getPoseEstimate();
        Match.hubPose = hubPose;
    }
    // endregion

    // region Vision Initialization
    /**
     * Initializes vision using Vuforia (OpenCV will use a pass-through)
     */
    protected void initVisionVuforia() {
        navigator = new VuforiaNavigator(hardwareMap, telemetry);
        camera = navigator.createOpenCvPassthru();
        guide = new TFWarehouseGuide(navigator.getLocalizer(), hardwareMap);
        scanner = new BarcodeScanner(camera);
    }

    /**
     * Initializes vision using only OpenCV.
     */
    protected void initVisionWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        ((OpenCvWebcam) camera).setMillisecondsPermissionTimeout(2500);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        guide = new WarehouseGuide(camera);
        scanner = new BarcodeScanner(camera);
    }
    // endregion

    // region Helper Methods

    /**
     * Runs all tasks in the task queue in order.
     */
    protected void runTasks() {
        Match.status("Running tasks...");
        try {
            for (Object task : tasks) {
                if (task instanceof String) {
                    if (dynamicTasks.containsKey(task)) {
                        task = dynamicTasks.get(task);
                    } else if (BuildConfig.DEBUG) {
                        throw new IllegalArgumentException("Please initialize the dynamic task `" + task + "`");
                    } else {
                        continue;
                    }
                }

                if (task instanceof TrajectorySequence) {
                    drive.follow((TrajectorySequence) task);
                } else if (task instanceof RobotRunnable) {
                    ((RobotRunnable) task).run();
                } else if (task instanceof Runnable) {
                    ((Runnable) task).run();
                }
            }
        } catch (InterruptedException ignored) {
            Match.status("Interrupted, wrapping up");
        }
        tasks.clear();
    }

    public static double deg(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * Converts a point from Cartesian to Roadrunner by rotating the coordinate plane 90 degrees
     * clockwise (positive y-axis points at the shared hub).
     *
     * @param x x-coordinate of the robot (+x points toward the red alliance station)
     * @param y y-coordinate of the robot (+y points toward the shared hub)
     */
    public static Pose2d pos(double x, double y) {
        return Match.isRed ? new Pose2d(+y, +x) : new Pose2d(+y, -x);
    }

    /**
     * Converts a pose from Cartesian to Roadrunner by rotate the coordinate plane 90 degrees
     * clockwise (positive y-axis points at the shared hub).
     *
     * @param x x-coordinate of the robot (+x points toward the red alliance station)
     * @param y y-coordinate of the robot (+y points toward the shared hub)
     * @param heading heading of the robot (+y / shared hub is zero degrees)
     */
    public static Pose2d pos(double x, double y, double heading) {
        if (Match.isRed) {
            return new Pose2d(+y, +x, -Math.toRadians(heading));
        } else {
            return new Pose2d(+y, -x, Math.toRadians(heading));
        }
    }

    protected void queue(Object o) {
        tasks.add(o);
    }

    protected void queue(TrajectorySequenceBuilder seqBuilder) {
        queue(seqBuilder.build());
    }

    protected void queue(Supplier<TrajectorySequence> seq) {
        queue(seq.get());
    }

    protected void queue(RobotRunnable run) {
        queue((Object) run);
    }

    /**
     *  Sets the last pose manually when robot.turn() is used between trajectory sequences
     */
    protected void queue(Pose2d pose) {
        queue((Object) pose);
    }

    /**
     * @return the pose where the last trajectory (or manual movement) left off
     */
    protected Pose2d getLastPose() {
        for (int i = tasks.size() - 1; i >= 0; i--) {
            if (tasks.get(i) instanceof TrajectorySequence) {
                return ((TrajectorySequence) tasks.get(i)).end();
            } else if (tasks.get(i) instanceof Pose2d) {
                return (Pose2d) tasks.get(i);
            }
        }
        return initialPose;
    }

    protected TrajectorySequenceBuilder fromHere() {
        return drive.from(getLastPose());
    }

    protected TrajectorySequenceBuilder from(Pose2d pose) {
        return drive.from(pose);
    }

    protected void startGuide() {
        if (VISION) {
            guide.start();
        }
    }

    protected void stopGuide() {
        if (VISION) {
            guide.stop();
        }
    }

    protected void queueYCalibration(Pose2d calibratedPose) {
        queue(() -> {
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d correctedPose = new Pose2d(
                    calibratedPose.getX(),
                    currentPose.getY(),
                    calibratedPose.getHeading()
            );
            drive.setPoseEstimate(correctedPose);
        });
        queue(calibratedPose);
    }

    protected void queueXCalibration(Pose2d calibratedPose) {
        queue(() -> {
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d correctedPose = new Pose2d(
                    currentPose.getX(),
                    calibratedPose.getY(),
                    calibratedPose.getHeading()
            );
            drive.setPoseEstimate(correctedPose);
        });
        queue(calibratedPose);
    }
    // endregion
}
