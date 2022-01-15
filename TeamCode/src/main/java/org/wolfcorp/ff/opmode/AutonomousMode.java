package org.wolfcorp.ff.opmode;

import static org.wolfcorp.ff.opmode.util.Match.BLUE;
import static org.wolfcorp.ff.opmode.util.Match.RED;
import static org.wolfcorp.ff.robot.DriveConstants.LENGTH;
import static org.wolfcorp.ff.robot.DriveConstants.TRACK_WIDTH;
import static org.wolfcorp.ff.robot.DriveConstants.WIDTH;
import static org.wolfcorp.ff.robot.Drivetrain.getAccelerationConstraint;
import static org.wolfcorp.ff.robot.Drivetrain.getVelocityConstraint;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.EMPTY;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.FULL;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.OVERFLOW;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.wolfcorp.ff.BuildConfig;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.opmode.util.RobotRunnable;
import org.wolfcorp.ff.opmode.util.TimedController;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.DriveConstants;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequence;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequenceBuilder;
import org.wolfcorp.ff.vision.Barcode;
import org.wolfcorp.ff.vision.BarcodeScanner;
import org.wolfcorp.ff.vision.Guide;
import org.wolfcorp.ff.vision.TFWarehouseGuide;
import org.wolfcorp.ff.vision.VuforiaNavigator;
import org.wolfcorp.ff.vision.WarehouseGuide;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.function.Supplier;

public abstract class AutonomousMode extends OpMode {
    // region Configuration
    public final boolean CAROUSEL = modeNameContains("Carousel");
    public final boolean WAREHOUSE = !CAROUSEL;
    public final boolean CYCLE = modeNameContains("Cycle");
    public final boolean PARK = !CYCLE;

    public static int SCORING_CYCLES = 0;
    // endregion

    // region Vision Fields
    protected OpenCvCamera camera = null;
    protected BarcodeScanner scanner;
    protected Guide guide;
    protected VuforiaNavigator navigator;
    protected Barcode barcode;
    protected Thread initVisionThread;
    // endregion

    // region Miscellaneous
    private boolean handleExcess = false;
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
    /** Where the robot scores freight into hub (perfect pose). */
    protected Pose2d trueHubPose;
    /** Where the robot scores freight into hub (takes robot error into account). */
    protected Pose2d hubPose;
    /** Where the robot scores the shipping element. */
    protected Pose2d capPose;
    /** Where the robot scores freight during cyclign **/
    protected Pose2d cycleHubPose;
    /** For x-coordinate calibration while cycling (beyond the wall). */
    protected Pose2d calibrateHubWallPose;
    /** For x-coordinate calibration while cycling (real pose). */
    protected Pose2d hubWallPose;
    /** x-coordinate calibration (beyond the wall) */
    protected Pose2d calibratePreWhPose;
    /** Used to avoid driving on the barrier / triangles & to start vision. */
    protected Pose2d preWhPose;
    /** x-coordinate calibration (beyond the wall) */
    protected Pose2d calibrateWhPose;
    /** Where the robot loads freight from warehouses (perfect pose). */
    protected Pose2d trueWhPose;
    /** Where the robot loads freight from warehouses (takes error into account). */
    protected Pose2d whPose;
    /** Where the robot parks in the warehouse. */
    protected Pose2d parkPose;
    /** x-coordinate calibration */
    protected Pose2d calibratePreDuckPose;
    /** Where the robot moves to prior to picking up duck. Hub wall pose but opposite direction. */
    protected Pose2d preDuckPose;
    /** Where the robot moves to in order to pick up duck */
    protected Pose2d duckPose;
    /** Where the robot parks in the storage unit. */
    protected Pose2d storageUnitParkPose;
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
        // NOTE: All poses are defined assuming we start at blue warehouse!
        if (RED) {
            initialPose = pos(-72 + LENGTH / 2 + 1 /* gap */, WIDTH / 2, 90);
        } else {
            initialPose = pos(-72 + LENGTH / 2 + 1 /* gap */, (CAROUSEL ? 1 : -1) * 24 - WIDTH / 2, 90);
        }
        carouselPose = pos(-53.75, -72 + WIDTH / 2, 90); //  x += RED ? 5 : 0
        preCarouselPose = carouselPose.plus(pos(3, 0));
        calibratePreCarouselPose = preCarouselPose.minus(pos(0, 5));

        if (RED) {
            elementLeftPose = pos(-52, WIDTH / 2, 90); // originally x = -54, y = 20.4
            elementMidPose = pos(-52, 7.25 + 8.25, 90); // y-difference was supposed to be 8.4
            elementRightPose = pos(-52, -1.8 + 8.25, 90);
        } else {
            elementLeftPose = pos(-52, 15, 90); // originally x = -54, y = 20.4
            elementMidPose = pos(-52, 8.25, 90); // y-difference was supposed to be 8.4
            elementRightPose = pos(-52, -1.8, 90);
        }
        trueHubPose = pos(-48.5, -12, 90);
        if (RED && CAROUSEL && CYCLE) {
            hubPose = pos(-45, -3, 90);
            cycleHubPose = pos(-48, -11, 90);
        } if (RED && CAROUSEL && PARK) {
            hubPose = pos(-40.5, -5, 90);
            cycleHubPose = pos(-48, -11, 90);
        } else if (BLUE && CAROUSEL && CYCLE) {
            hubPose = pos(-43.5, -8, 90);
            cycleHubPose = pos(-48, -14, 90);
        } else if (BLUE && CAROUSEL && PARK) {
            hubPose = pos(-42, -8, 90);
            cycleHubPose = pos(-48, -14, 90);
        } else if (RED && WAREHOUSE) {
            hubPose = pos(-42, -12, 90);
            cycleHubPose = pos(-48, -14, 90);
        } else if (BLUE && WAREHOUSE) {
            hubPose = pos(-42, -12, 90);
            cycleHubPose = pos(-48, -14, 90);
        }
        capPose = hubPose.minus(pos(2, WIDTH / 2));
        preHubPose = pos(-48, -12, 0);
        // FIXME: may need to align x-coordinate with hubPose?
        hubWallPose = pos(-72 + WIDTH / 2, -12, 0);
        calibrateHubWallPose = hubWallPose.minus(pos(6, 0));

        preWhPose = pos(-72 + WIDTH / 2, 12, 0);
        calibratePreWhPose = preWhPose.minus(pos(4, 0));
        trueWhPose = pos(-72 + WIDTH / 2, 30, 0);
        whPose = trueWhPose.plus(pos(0, 13));
        calibrateWhPose = whPose.minus(pos(8, 0));

        parkPose = pos(-72 + WIDTH / 2, 40.5 + LENGTH / 2, 0);

        preDuckPose = pos(-72 + WIDTH / 2, -12, 180);
        calibratePreDuckPose = preDuckPose.minus(pos(3, 0));
        duckPose = preDuckPose.minus(pos(0, 48)); // FIXME: fix the y value
        if (RED) {
            storageUnitParkPose = pos(-36 + 3, -72 + WIDTH / 2 - 2.5, 90);
        } else {
            storageUnitParkPose = pos(-36 + 1, -72 + WIDTH / 2 - 2.5, 90);
        }

        // Carousel path's initial pose is two tiles over from warehouse.
        if (CAROUSEL) {
            SCORING_CYCLES = 1;
            Pose2d offset = pos(0, 48);
            initialPose = initialPose.minus(offset);
            elementLeftPose = elementLeftPose.minus(offset);
            elementMidPose = elementMidPose.minus(offset);
            elementRightPose = elementRightPose.minus(offset);
        } else {
            SCORING_CYCLES = 2;
        }
    }

    /**
     * Performs the autonomous path based on the OpMode's name (which may be changed by inheritance)
     * and other configuration variables (i.e. {@link #SCORING_CYCLES}).
     */
    @Override
    public void runOpMode() {
        prologue();

        grabShippingElement();
        spinCarousel(); // CAROUSEL only
        deposit();
        cycle(); // CYCLE only
        // getDuck(); // CAROUSEL PARK only
        park();
        getFreight(); // CYCLE only

        epilogue();
    }

    public void prologue() {
        Match.setupTelemetry();
        // *** Initialization ***
        initHardware();

        Match.status("Starting vision init thread");
        initVisionThread = new Thread(this::initVisionWebcam);
        initVisionThread.start();

        Match.status("Setting pose");
        drive.setPoseEstimate(initialPose);

        Match.status("Robot Initialized, preparing task queue");
    }
    public void grabShippingElement() {
        // FIXME: path-building in real-time
        Match.status("Initializing: shipping element (SE)");
        queue(shippingArm::openClaw);
        queue(shippingArm::armOutermostAsync);
        queue(() -> sleep(1500));
        // step to the side when at warehouse to avoid the barrier
        if (WAREHOUSE) {
            queue(fromHere().lineTo(initialPose.minus(pos(0, 4)).vec()));
        }
        queue(() -> barcode == Barcode.BOT, fromHere().lineTo(elementLeftPose.vec(), getVelocityConstraint(35, 5, TRACK_WIDTH), getAccelerationConstraint(35)));
        queue(() -> barcode == Barcode.MID, fromHere().lineTo(elementMidPose.vec(), getVelocityConstraint(35, 5, TRACK_WIDTH), getAccelerationConstraint(35)));
        queue(() -> barcode == Barcode.TOP, fromHere().lineTo(elementRightPose.vec(), getVelocityConstraint(35, 5, TRACK_WIDTH), getAccelerationConstraint(35)));
        queue(() -> {
            shippingArm.closeClaw();
            sleep(1000);
            if (CAROUSEL) {
                shippingArm.armOutAsync();
            } else {
                shippingArm.armInAsync();
            }
        });
    }
    public void spinCarousel() {
        if (CAROUSEL) {
            Match.status("Initializing: carousel");
//            queue(spinner::on);
            queue(() -> barcode == Barcode.BOT, from(elementLeftPose) .lineTo(calibratePreCarouselPose.vec()));
            queue(() -> barcode == Barcode.MID, from(elementMidPose)  .lineTo(calibratePreCarouselPose.vec()));
            queue(() -> barcode == Barcode.TOP, from(elementRightPose).lineTo(calibratePreCarouselPose.vec()));
            queueYCalibration(preCarouselPose);
            queue(fromHere().lineTo(carouselPose.vec(), getVelocityConstraint(10, 5, TRACK_WIDTH), getAccelerationConstraint(10)));
//            queue(() -> {
//               ElapsedTime spinTimer = new ElapsedTime();
//               while (spinTimer.milliseconds() < CarouselSpinner.WAIT_TIME && OpMode.isActive());
//               spinner.off();
//               shippingArm.armInAsync();
//            });
            queue(spinner::spin);
            queue(shippingArm::armInAsync);
        }
    }
    public void deposit() {
        Match.status("Initializing: deposit (preloaded & SE)");
        // move to carouse
        if (CAROUSEL) {
            // already dealt with end pose branching earlier
            queue(from(carouselPose).now(() -> outtake.slideToAsync(barcode)).lineTo(hubPose.vec(), getVelocityConstraint(35, 5, TRACK_WIDTH), getAccelerationConstraint(25)));
        } else {
            queue(() -> barcode == Barcode.BOT, from(elementLeftPose).now(() -> outtake.slideToAsync(barcode)).lineTo(hubPose.vec(), getVelocityConstraint(35, 5, TRACK_WIDTH), getAccelerationConstraint(25)));
            queue(() -> barcode == Barcode.MID, from(elementMidPose).now(() -> outtake.slideToAsync(barcode)).lineTo(hubPose.vec(), getVelocityConstraint(35, 5, TRACK_WIDTH), getAccelerationConstraint(25)));
            queue(() -> barcode == Barcode.TOP, from(elementRightPose).now(() -> outtake.slideToAsync(barcode)).lineTo(hubPose.vec(), getVelocityConstraint(35, 5, TRACK_WIDTH), getAccelerationConstraint(25)));
        }
        queue(() -> {
            outtake.dumpOut();
            sleep(1200);
            // slide and dump will be reset in cycle()
        });
        // TODO: move to a better pose to cap, do armOut during the path
    }
    public void cycle() {
        if (PARK)
            return;
        for (int i = 1; i <= SCORING_CYCLES; i++) {
            Match.status("Initializing: cycle " + i);
            if (i == 1) {
                queue(outtake::dumpIn);
                queue(() -> outtake.slideToAsync(Barcode.ZERO));
            }


            // *** Hub to warehouse ***
            queue(from(hubPose)
                    .lineToLinearHeading(calibrateHubWallPose)
                    .now(() -> {
                        drive.setPoseEstimate(hubWallPose);
                        intake.in();
                    })
                    .lineTo(calibrateWhPose.minus(pos(0, 5)).vec()));


            // *** Intake ***
            queue(() -> {
                ElapsedTime timer = new ElapsedTime();
                TimedController controller = new TimedController(30, 480, 800);
                drive.setMotorPowers(0.3);
                while (dumpIndicator.update() == EMPTY) {
                    drive.updatePoseEstimate();
                    intake.setVelocityRPM(controller.update());
                }
                drive.setMotorPowers(0);
            });
            // TEST!
//            queue(() -> drive.sidestepRight(0.75, 5 * (RED ? 1 : -1)));
            queue(() -> {
                if (dumpIndicator.update() != EMPTY) {
                    intake.out();
                } else {
                    intake.getMotor().setVelocity(1.75 * Intake.IN_SPEED);
                }
            });
            queueWarehouseSensorCalibration(pos(-72 + DriveConstants.WIDTH / 2, 46, 0));
            queue(() -> Match.status(drive.getPoseEstimate() + "; sensor = " + rangeSensor.getDistance(DistanceUnit.INCH)));


            // *** Warehouse to hub ***
            queue(fromHere()
                    // get out
                    .lineTo(preWhPose.minus(pos(4, 10)).vec())
                    .now(() -> {
                        switch (dumpIndicator.update()) {
                            case EMPTY:
                                intake.in();
                                break;
                            case FULL:
                                intake.out();
                            case OVERFLOW:
//                                outtake.slideToAsync(Barcode.EXCESS);
                                // outtake.dumpExcess();
                                intake.out();
                                handleExcess = true;
                                break;
                        }
                    }));
            queue(fromHere().addTemporalMarker(0.5, () -> {
                if (dumpIndicator.update() == FULL) {
                    outtake.slideToAsync(Barcode.TOP);
                }
            }).lineToSplineHeading(cycleHubPose));
            queueHubSensorCalibration(trueHubPose);


            // *** Score ***
            queue(() -> {
                if (handleExcess) {
                    outtake.resetSlide();
                    handleExcess = false;
                }
                intake.off();

//                outtake.slideTo(Barcode.TOP); // since it gets counted in TeleOp period scoring
                outtake.slideTo(Barcode.TOP);
                outtake.dumpOut();
                sleep(1200);
                outtake.dumpIn();
                outtake.slideToAsync(Barcode.ZERO);
            });
        }
    }
    public void getDuck() {
        if (CAROUSEL && PARK) {
            queue(from(hubPose)
                    .lineToLinearHeading(calibrateHubWallPose)
                    .now(() -> {
                        drive.setPoseEstimate(hubWallPose);
                        intake.in();
                    })
                    .lineTo(calibrateWhPose.minus(pos(0, 5)).vec()));

        }
    }
    public void park() {
        Match.status("Initializing: park");
        // park in storage unit
        if (!CYCLE && CAROUSEL) {
            queue(from(hubPose)
                    .now(outtake::dumpIn)
                    .now(() -> outtake.slideTo(Barcode.ZERO))
                    .lineTo(storageUnitParkPose.vec()));
            return;
        }
        // park in warehouse
        if (SCORING_CYCLES == 0) {
            queue(from(hubPose)
                    .now(outtake::dumpIn)
                    .now(() -> outtake.slideTo(Barcode.ZERO))
                    .lineToSplineHeading(preHubPose.minus(pos(10, -12)))
                    .splineToLinearHeading(preWhPose.minus(pos(4, 0)), 0)
                    .lineTo(parkPose.minus(pos(10, 0)).vec(), getVelocityConstraint(30, 5, TRACK_WIDTH), getAccelerationConstraint(30)));
        } else {
            queue(fromHere()
                    .lineToLinearHeading(calibrateHubWallPose)
                    .now(() -> {
                        drive.setPoseEstimate(hubWallPose);
                    })
                    .lineTo(calibrateWhPose.minus(pos(0, 5)).vec())
                    .lineTo(calibrateWhPose.vec(), getVelocityConstraint(40, 5, TRACK_WIDTH), getAccelerationConstraint(40)));
        }
        queueWarehouseSensorCalibration(parkPose);
        queue(shippingArm::resetArm);
    }
    public void getFreight() {
        if (!CYCLE && CAROUSEL)
            return;
        queue(() -> {
            intake.in();
            while (dumpIndicator.update() == EMPTY && opModeIsActive()) {
            }
            if (dumpIndicator.update() == OVERFLOW) {
//                outtake.dumpExcess();
                intake.out();
            } else {
                intake.off();
            }
        });
    }
    public void startScanner() {
        Match.status("Initializing: vision");
        try {
            initVisionThread.join();
        } catch (InterruptedException ignored) {
            Match.status("OpMode interrupted");
            Match.log("OpMode interrupted");
            return;
        }
        scanner.start();
    }
    public void scanBarcode() {
        Match.status("Scanning");
        try {
            barcode = scanner.getBarcode();
        } catch (InterruptedException ignored) {
            Match.status("OpMode interrupted");
            return;
        }
        scanner.stop();
    }
    public void epilogue() {
        startScanner();
        Match.status("Task queue ready, waiting for start");
        waitForStart();
        Match.status("Start!");

        scanBarcode();
        runTasks();

        sleep(1000);
        Match.teleOpInitialPose = drive.getPoseEstimate();
        Match.hubPose = hubPose;
        resetHardware();
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

    // region Helper

    public class ConditionalTask {
        private final Supplier<Boolean> condition;
        private final Object task;

        public ConditionalTask(Supplier<Boolean> condition, Object task) {
            this.condition = condition;
            this.task = task;
        }

        public boolean satisfied() {
            return condition.get();
        }

        public Object getTask() {
            return task;
        }
    }

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
                } else if (task instanceof ConditionalTask) {
                    if (((ConditionalTask) task).satisfied()) {
                        task = ((ConditionalTask) task).getTask();
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
        return RED ? new Pose2d(+y, +x) : new Pose2d(+y, -x);
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
        if (RED) {
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

    protected void queue(Supplier<Boolean> condition, TrajectorySequenceBuilder seqBuilder) {
        queue(new ConditionalTask(condition, seqBuilder.build()));
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

    protected void queue(Supplier<Boolean> condition, RobotRunnable runnable) {
        queue(new ConditionalTask(condition, runnable));
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
        guide.start();
    }

    /**
     * Deactivates or shuts down the {@link #guide}.
     */
    protected void stopGuide() {
        guide.stop();
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

    /**
     * Queues a xy-coordinates calibration of the robot's pose estimate using a given pose. The new
     * pose estimate will assume the alliance-agnostic xy-coordinates and the heading of the given
     * pose. Future trajectories will automatically begin at the given pose.
     *
     * @param calibratedPose the correct pose of the robot
     * @see #pos(double, double, double)
     * @see #queue(Pose2d)
     */
    protected void queueCalibration(Pose2d calibratedPose) {
        queue(() -> drive.setPoseEstimate(calibratedPose));
        queue(calibratedPose);
    }

    /**
     * Queues a x-coordinates calibration of the robot's pose estimate at the hub. The new pose
     * estimate will calibrate the alliance-agnostic x-coordinates based on the distance sensor
     * reading. Future trajectories will begin at a predicted pose.
     *
     * @param predictedPose the correct pose of the robot
     * @see #pos(double, double, double)
     * @see #queue(Pose2d)
     */
    protected void queueHubSensorCalibration(Pose2d predictedPose) {
        queue(() -> {
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d correctedPose = new Pose2d(
                    currentPose.getX(),
                    pos(-72 + rangeSensor.getDistance(DistanceUnit.INCH) + 6.5,0).getY(),
                    currentPose.getHeading()
            );
            drive.setPoseEstimate(correctedPose);
        });
        queue(predictedPose);
    }

    /**
     * Queues a y-coordinates calibration of the robot's pose estimate at the warehouse. The new
     * pose estimate will calibrate the alliance-agnostic y-coordinates based on the distance sensor
     * reading. Future trajectories will begin at a predicted pose.
     *
     * @param predictedPose the correct pose of the robot
     * @see #pos(double, double, double)
     * @see #queue(Pose2d)
     */
    protected void queueWarehouseSensorCalibration(Pose2d predictedPose) {
        queue(() -> {
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d correctedPose = new Pose2d(
                    pos(0,72 - rangeSensor.getDistance(DistanceUnit.INCH) - 6.5).getX(),
                    predictedPose.getY(),
                    currentPose.getHeading()
            );
            drive.setPoseEstimate(correctedPose);
        });
        queue(predictedPose);
    }
    // endregion
}
