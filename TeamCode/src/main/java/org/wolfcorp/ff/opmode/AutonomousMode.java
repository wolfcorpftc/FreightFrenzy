package org.wolfcorp.ff.opmode;

import static org.wolfcorp.ff.robot.DriveConstants.TRACK_WIDTH;
import static org.wolfcorp.ff.robot.Drivetrain.getAccelerationConstraint;
import static org.wolfcorp.ff.robot.Drivetrain.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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
import java.util.HashMap;

public abstract class AutonomousMode extends OpMode {
    // region Hardware
    protected Drivetrain drive = null;
    protected CarouselSpinner spinner = null;
    protected OpenCvCamera camera = null;
    protected Intake intake = null;
    protected Outtake outtake = null;
    // endregion

    // region Configuration
    public final boolean VISION = !modeNameContains("NV");
    public final boolean CAROUSEL = modeNameContains("Carousel");
    public final boolean WALL_RUNNER = modeNameContains("WR");

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

    /** Where the robot starts on the field. */
    protected Pose2d initialPose;

    /** For y-coordinate calibration while scoring carousel (beyond the wall). */
    protected Pose2d calibratePreCarouselPose;
    /** For y-coordinate calibration while scoring carousel (real pose). */
    protected Pose2d preCarouselPose;
    /** Where the robot turns the carousel. */
    protected Pose2d carouselPose;

    /** Where the robot retrieves the left shipping element. */
    protected Pose2d elementLeftPose;
    /** Where the robot retrieves the middle shipping element. */
    protected Pose2d elementMidPose;
    /** Where the robot retrieves the right shipping element. */
    protected Pose2d elementRightPose;

    /**
     * Robot's pose between hub and wall; for heading adjustment w/o hitting SE or the wall
     * (depending on where the robot came from).
     */
    protected Pose2d preHubPose;
    /** Where the robot scores freight into hub. */
    protected Pose2d hubPose;
    /** For x-coordinate calibration while cycling (beyond the wall). */
    protected Pose2d calibrateHubWallPose;
    /** For x-coordinate calibration while cycling (real pose). */
    protected Pose2d hubWallPose;
    /** Used to avoid driving on the barrier / triangles & to start vision. */
    protected Pose2d preWhPose;
    /** Where the robot loads freight from warehouses. */
    protected Pose2d whPose;
    /** Where the robot parks. */
    protected Pose2d parkPose;
    // endregion

    // region Task Queue
    /** The task queue. */
    private final ArrayList<Object> tasks = new ArrayList<>();
    /**
     * Where the contents of named tasks are stored (only necessary if the task content is random,
     * like picking up the shipping element whose position is determined randomly pre-match).
     * <p>
     * Named tasks can be added by first calling {@link #queue(Object)} with the name of the task to
     * define its position in the queue, then inserting the name-task pair into this HashMap using
     * {@link HashMap#put(Object, Object)} before {@link #runTasks()} is called.
     */
    private final HashMap<String, Object> dynamicTasks = new HashMap<>();
    // endregion

    // region Robot Logic

    /**
     * Initializes waypoints during autonomous paths based on the OpMode's name (which may be
     * changed by inheritance).
     */
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
            // FIXME: avoid touching the hub
            hubPose = pos(-51 + DriveConstants.WIDTH / 2, -12 + (CAROUSEL ? 0 : -3), 90);
        } else {
            hubPose = pos(-47 + DriveConstants.WIDTH / 2, -12 + (CAROUSEL ? 0 : -3), 90);
        }
        preHubPose = pos(-48, -12, 0);
        hubWallPose = pos(-72 + DriveConstants.WIDTH / 2, -12, 0);
        calibrateHubWallPose = hubWallPose.minus(pos(6, 0));
        preWhPose = pos(-72 + DriveConstants.WIDTH / 2, 12, 0);
        whPose = pos(-72 + DriveConstants.WIDTH / 2, 42, 0);

        final double HUB_X_ERROR = CAROUSEL ? (Match.isRed ? 3 : -2) : 5;
        final double HUB_Y_ERROR = Match.isRed ? 5 : 0;
        hubPose = hubPose.plus(pos(HUB_X_ERROR, HUB_Y_ERROR));
        preHubPose = preHubPose.plus(pos(HUB_X_ERROR, HUB_Y_ERROR));
        hubWallPose = hubWallPose.plus(pos(HUB_X_ERROR, HUB_Y_ERROR));

        parkPose = pos(-72 + DriveConstants.WIDTH / 2, 38 + DriveConstants.LENGTH / 2, 0);
        if (!WALL_RUNNER) {
            // Park in the tile to the right
            parkPose = parkPose.plus(pos(24, 0));
        }

        // Carousel path's initial pose is two tiles over from warehouse.
        if (CAROUSEL) {
            initialPose = initialPose.minus(pos(0, 48));

            elementLeftPose = elementLeftPose.minus(pos(0, 48));
            elementMidPose = elementMidPose.minus(pos(0, 48));
            elementRightPose = elementRightPose.minus(pos(0, 48));
        }
    }

    /**
     * Performs the autonomous path based on the OpMode's name (which may be changed by inheritance)
     * and other configuration variables (i.e. {@link #SCORING_CYCLES}).
     */
    @Override
    public void runOpMode() {
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
                sleep(2400);
                outtake.dumpIn();
                outtake.slideTo(Barcode.ZERO);
            });
        }

        // *** Park ***
        Match.status("Initializing: park");
        if (!isOuttakeReset) {
            queue(fromHere()
                    .now(outtake::dumpIn)
                    .now(() -> outtake.slideTo(Barcode.ZERO))
                    .lineToSplineHeading(preHubPose, getVelocityConstraint(30, 5, TRACK_WIDTH), getAccelerationConstraint(15))
                    .lineTo(calibrateHubWallPose.vec(), getVelocityConstraint(30, 5, TRACK_WIDTH), getAccelerationConstraint(15))
            );
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
            try {
                initVisionThread.join();
            } catch (InterruptedException ignored) {
                Match.status("OpMode interrupted");
                return;
            }
            scanner.start();
        }
        Match.status("Task queue ready, waiting for start");

        // *** START ***
        waitForStart();
        // *** Scan Barcode ***
        if (VISION) {
            Match.status("Scanning");
            try {
                barcode = scanner.getBarcode();
            } catch (InterruptedException ignored) {
                Match.status("OpMode interrupted");
                return;
            }
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
     * Initializes vision using Vuforia (OpenCV will use a pass-through).
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
     * Converts a pose from Cartesian to Roadrunner by rotating the coordinate plane 90 degrees
     * clockwise (positive y-axis points at the shared hub).
     *
     * @param x       x-coordinate of the robot (+x points toward the red alliance station)
     * @param y       y-coordinate of the robot (+y points toward the shared hub)
     * @param heading heading of the robot (+y / shared hub is zero degrees)
     */
    public static Pose2d pos(double x, double y, double heading) {
        if (Match.isRed) {
            return new Pose2d(+y, +x, -Math.toRadians(heading));
        } else {
            return new Pose2d(+y, -x, Math.toRadians(heading));
        }
    }

    /**
     * Adds an object into the task queue.
     *
     * @param o the object
     */
    protected void queue(Object o) {
        tasks.add(o);
    }

    /**
     * Builds the {@link TrajectorySequenceBuilder} and adds the built {@link TrajectorySequence} to
     * the task queue.
     *
     * @param seqBuilder the trajectory sequence builder
     */
    protected void queue(TrajectorySequenceBuilder seqBuilder) {
        queue(seqBuilder.build());
    }

    /**
     * Adds a special runnable task to the task queue. A {@link RobotRunnable} object is used
     * instead of {@link Runnable} because the lambda could throw an {@link InterruptedException}
     * due to the nature of autonomous code in robotics.
     *
     * @param runnable the runnable
     */
    protected void queue(RobotRunnable runnable) {
        queue((Object) runnable);
    }

    /**
     * Specify the robot's current pose manually. This will shadow the end pose of the last
     * trajectory in the {@link #tasks} ArrayList.
     *
     * @see #getCurrentPose()
     */
    protected void queue(Pose2d pose) {
        queue((Object) pose);
    }

    /**
     * Retrieves the pose where the last trajectory left off, unless otherwise specified by
     * {@link #queue(Pose2d)}.
     *
     * @return the pose where the robot is at assuming a perfect execution
     */
    protected Pose2d getCurrentPose() {
        for (int i = tasks.size() - 1; i >= 0; i--) {
            if (tasks.get(i) instanceof TrajectorySequence) {
                return ((TrajectorySequence) tasks.get(i)).end();
            } else if (tasks.get(i) instanceof Pose2d) {
                return (Pose2d) tasks.get(i);
            }
        }
        return initialPose;
    }

    /**
     * Retrieves a {@link TrajectorySequenceBuilder} that starts from the pose indicated by
     * {@link #getCurrentPose()}
     *
     * @return a {@link TrajectorySequenceBuilder} that begins at the robot's current pose
     */
    protected TrajectorySequenceBuilder fromHere() {
        return drive.from(getCurrentPose());
    }

    /**
     * Retrieves a {@link TrajectorySequenceBuilder} that starts from the given pose.
     *
     * @param pose the pose from which the trajectory sequence starts
     * @return a {@link TrajectorySequenceBuilder} that begins at the given pose
     */
    protected TrajectorySequenceBuilder from(Pose2d pose) {
        return drive.from(pose);
    }

    /**
     * Activates the {@link #guide}.
     */
    protected void startGuide() {
        if (VISION) {
            guide.start();
        }
    }

    /**
     * Deactivates or shuts down the {@link #guide}.
     */
    protected void stopGuide() {
        if (VISION) {
            guide.stop();
        }
    }

    /**
     * Queues a y-coordinate calibration of the robot's pose estimate using a given pose. The new
     * pose estimate will assume the alliance-agnostic y-coordinate and the heading of the given
     * pose. Future trajectories will automatically begin at the given pose (<em>Note: </em> not the
     * actual new pose estimate, which cannot be known before the autonomous period actually starts).
     *
     * @param calibratedPose the correct pose of the robot
     * @see #pos(double, double, double)
     * @see #queue(Pose2d)
     */
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

    /**
     * Queues a x-coordinate calibration of the robot's pose estimate using a given pose. The new
     * pose estimate will assume the alliance-agnostic x-coordinate and the heading of the given
     * pose. Future trajectories will automatically begin at the given pose (<em>Note: </em> not the
     * actual new pose estimate, which cannot be known before the autonomous period actually starts).
     *
     * @param calibratedPose the correct pose of the robot
     * @see #pos(double, double, double)
     * @see #queue(Pose2d)
     */
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
