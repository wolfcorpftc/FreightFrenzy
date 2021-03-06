package org.wolfcorp.ff.opmode;

import static org.wolfcorp.ff.opmode.util.Match.BLUE;
import static org.wolfcorp.ff.opmode.util.Match.RED;
import static org.wolfcorp.ff.robot.CarouselSpinner.SPIN_TIME;
import static org.wolfcorp.ff.robot.CarouselSpinner.WAIT_TIME;
import static org.wolfcorp.ff.robot.DriveConstants.LENGTH;
import static org.wolfcorp.ff.robot.DriveConstants.TRACK_WIDTH;
import static org.wolfcorp.ff.robot.DriveConstants.WIDTH;
import static org.wolfcorp.ff.robot.Drivetrain.getAccelerationConstraint;
import static org.wolfcorp.ff.robot.Drivetrain.getVelocityConstraint;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.EMPTY;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.FULL;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.OVERFLOW;
import static org.wolfcorp.ff.vision.Barcode.EXCESS;
import static org.wolfcorp.ff.vision.Barcode.SUPERTOP;
import static org.wolfcorp.ff.vision.Barcode.ZERO;
import static java.lang.Math.cos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.wolfcorp.ff.BuildConfig;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.opmode.util.RobotRunnable;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequence;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequenceBuilder;
import org.wolfcorp.ff.robot.util.InchSensor;
import org.wolfcorp.ff.vision.Barcode;
import org.wolfcorp.ff.vision.BarcodeScanner;
import org.wolfcorp.ff.vision.Guide;
import org.wolfcorp.ff.vision.PartialBarcodeScanner;
import org.wolfcorp.ff.vision.TFWarehouseGuide;
import org.wolfcorp.ff.vision.VuforiaNavigator;
import org.wolfcorp.ff.vision.WarehouseGuide;

import java.util.ArrayList;
import java.util.function.Supplier;

public abstract class KalmanAutonomousMode extends OpMode {
    // region Configuration
    public final boolean CAROUSEL = modeNameContains("Carousel");
    public final boolean WAREHOUSE = !CAROUSEL;
    public final boolean CYCLE = modeNameContains("Cycle");
    public final boolean PARK = !CYCLE;
    public final boolean SAFETY = modeNameContains("Safety");

    public final int SCORING_CYCLES = (WAREHOUSE && !SAFETY) ? 5 : 2; // varies based on path
    // endregion

    // region Vision Fields
    protected OpenCvCamera camera = null;
    protected BarcodeScanner scanner;
    protected Guide guide;
    protected VuforiaNavigator navigator;
    protected Barcode barcode;
    protected Thread initVisionThread;
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
    /** Actual pose where the obot turn the carousel (for localization purposes)." */
    protected Pose2d trueCarouselPose;

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
    protected Pose2d carouselHubPose;
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
    protected Pose2d storageUnitPose;
    protected Pose2d trueStorageUnitPose;
    /** Where the robot parks on the shared alliance wall */
    protected Pose2d sharedParkPose;
    // endregion

    // region Task Queue
    /**
     * The task queue.
     *
     * @see ConditionalTask
     */
    private final ArrayList<Object> tasks = new ArrayList<>();
    private final ElapsedTime runtime = new ElapsedTime();
    // endregion

    // region Robot Logic

    /**
     * Initializes waypoints during autonomous paths based on the OpMode's name (which may be
     * changed by inheritance).
     */
    public KalmanAutonomousMode() {

        // NOTE: All poses are defined assuming we start at blue warehouse!
        if (RED) {
            initialPose = pos(-72 + LENGTH / 2 + 1 /* gap */, WIDTH / 2, 90);
        } else if (BLUE) {
            initialPose = pos(-72 + LENGTH / 2 + 1 /* gap */, 24 - WIDTH / 2, 90);
        }
        if (BLUE && WAREHOUSE) {
            initialPose = initialPose.minus(pos(0, 2));
        }

        if (RED && CAROUSEL) {
            elementLeftPose = pos(-52, WIDTH / 2, 90); // originally x = -54, y = 20.4
            elementMidPose = pos(-52, 7.25 + 8.25, 90); // y-difference was supposed to be 8.4
            elementRightPose = pos(-52, 11 + 15, 90);
        } else if (BLUE && CAROUSEL) {
            elementLeftPose = pos(-52, 17, 90); // originally x = -54, y = 20.4
            elementMidPose = pos(-52, 8.25, 90); // y-difference was supposed to be 8.4
            elementRightPose = pos(-52, -1.8, 90);
        } else if (RED && WAREHOUSE) {
            elementLeftPose = pos(-52, WIDTH / 2, 90); // originally x = -54, y = 20.4
            elementMidPose = pos(-52, 7.25 + 8.25, 90); // y-difference was supposed to be 8.4
            elementRightPose = pos(-52, 11 + 15, 90);
        } else if (BLUE && WAREHOUSE) {
            // everything is the same except right / top
            elementLeftPose = pos(-52, 17, 90); // originally x = -54, y = 20.4
            elementMidPose = pos(-52, 8.25, 90); // y-difference was supposed to be 8.4
            elementRightPose = pos(-52, -1.8, 90); // FIXME: fix pose; make sure no collision w/ wall
        }

        trueCarouselPose = pos(-48 + 1 - LENGTH / 2, -72 + WIDTH / 2, 90);
        if (RED) {
            carouselPose = pos(-54, -72 + WIDTH / 2, 90);
        } else {
            carouselPose = pos(-54, -72 + WIDTH / 2, 90);
        }
        preCarouselPose = carouselPose.plus(pos(4, 0));
        calibratePreCarouselPose = preCarouselPose.minus(pos(0, 8));

        trueHubPose = pos(-48.5, -12, 90);
        carouselHubPose = pos(-24, -33, 180);
        if (RED && CAROUSEL && CYCLE) {
            hubPose = pos(-45, 0, 90);
            cycleHubPose = pos(-48, -8, 90);
        } else if (RED && CAROUSEL && PARK) {
            carouselHubPose = pos(-28, -34, 180);
            hubPose = pos(-45, 0, 90);
            cycleHubPose = pos(-48, -8, 90);
        } else if (BLUE && CAROUSEL && CYCLE) {
            hubPose = pos(-43.5, -8, 90);
            cycleHubPose = pos(-48, -11, 90);
        } else if (BLUE && CAROUSEL && PARK) {
            carouselHubPose = pos(-32, -38, 180);
            hubPose = pos(-45, 0, 90);
            cycleHubPose = pos(-48, -8, 90);
        } else if (RED && WAREHOUSE) {
            hubPose = pos(-44, -14, 90);
            cycleHubPose = pos(-47, -3, 90);
        } else if (BLUE && WAREHOUSE) {
            hubPose = pos(-44, -16, 90);
            cycleHubPose = pos(-48, -6, 90);
        }

        if (SAFETY) {
            cycleHubPose.plus(pos(0,-15));
        }

        capPose = hubPose.minus(pos(2, WIDTH / 2));
        preHubPose = pos(-48, -12, 0);
        // FIXME: may need to align x-coordinate with hubPose?
        hubWallPose = pos(-72 + WIDTH / 2, -12, 0);
        calibrateHubWallPose = hubWallPose.minus(pos(6, 0));

        preWhPose = pos(-72 + WIDTH / 2, 12, 0);
        calibratePreWhPose = preWhPose.minus(pos(4, 0));
        trueWhPose = pos(-72 + WIDTH / 2, 30, 0);
        if (RED && CAROUSEL && CYCLE) {
            whPose = trueWhPose.plus(pos(0, 15));
        } else if (RED && CAROUSEL && PARK) {
            whPose = trueWhPose.plus(pos(0, 15));
        } else if (BLUE && CAROUSEL && CYCLE) {
            whPose = trueWhPose.plus(pos(0, 15));
        } else if (BLUE && CAROUSEL && PARK) {
            whPose = trueWhPose.plus(pos(0, 15));
        } else if (RED && WAREHOUSE) {
            whPose = trueWhPose.plus(pos(0, 9));
        } else if (BLUE && WAREHOUSE) {
            whPose = trueWhPose.plus(pos(-1, 11));
        }

        calibrateWhPose = whPose.minus(pos(8, 0));
        sharedParkPose = pos(-36, 72 - WIDTH / 2, 90);

        parkPose = pos(-72 + WIDTH / 2, 40.5 + LENGTH / 2, 0);

        preDuckPose = pos(-72 + WIDTH / 2, -12, 180);
        calibratePreDuckPose = preDuckPose.minus(pos(3, 0));
        duckPose = preDuckPose.minus(pos(0, 48)); // FIXME: fix the y value
        storageUnitPose = pos(-36, -72 + WIDTH / 2, 90);
        if (RED) {
            storageUnitPose = pos(-34, -72 + WIDTH / 2 - 2.5, 90);
        } else {
            storageUnitPose = pos(-34, -72 + WIDTH / 2 - 6, 90);
        }

        // Carousel path's initial pose is two tiles over from warehouse.
        if (CAROUSEL) {
            Pose2d offset = pos(0, 48);
            initialPose = initialPose.minus(offset);
            elementLeftPose = elementLeftPose.minus(offset);
            elementMidPose = elementMidPose.minus(offset);
            elementRightPose = elementRightPose.minus(offset);
        }
    }

    /**
     * Performs the autonomous path based on the OpMode's name (which may be changed by inheritance)
     * and other configuration variables (i.e. {@link #SCORING_CYCLES}).
     */
    @Override
    public void runOpMode() {
        prologue();

        spinCarousel(); // CAROUSEL only
        deposit();
        cycle(); // CYCLE only
        park();
        getFreight();
//        getFreight(); // CYCLE only
//        parkShared(); // CYCLE ONLY

        epilogue();
    }

    public void prologue() {
        Match.setupTelemetry();
        // *** Initialization ***
        initHardware();
        if (SAFETY) {
            drive.enableSlowDrive();
        }

        outtake.dumpIn();
        sleep(750);
        outtake.dumpExcess();
        sleep(750);
        outtake.dumpIn();

        Match.status("Starting vision init thread");
        initVisionThread = new Thread(this::initVisionWebcam);
        initVisionThread.start();

        Match.status("Setting pose");
        drive.setPoseEstimate(initialPose);
        drive.initKalman(preWhPose);

        Match.status("Robot Initialized, preparing task queue");
    }

    public void spinCarousel() {
        if (CAROUSEL) {
            Match.status("Initializing: carousel");
            queue(shippingArm::armOutAsync); // necessary to prevent the arm from blocking the spinner
            queue(fromHere()
                    .lineTo(calibratePreCarouselPose.vec())
                    .lineTo(carouselPose.minus(pos(0, 10)).vec(), getVelocityConstraint(30, 5, TRACK_WIDTH), getAccelerationConstraint(20)));
            queue(() -> {
                Thread spin = spinner.spinAsync(1, 2 * SPIN_TIME, WAIT_TIME);
                drive.setMotorPowers(0.05);
                Pose2d currentPose = drive.getPoseEstimate();
                Pose2d correctedPose = new Pose2d(
                        trueCarouselPose.getX(),
                        currentPose.getY(),
                        trueCarouselPose.getHeading()
                );
                drive.setPoseEstimate(correctedPose);
                spin.join();
                shippingArm.armInAsync(0.7);
            });
            queue(trueCarouselPose);
        }
    }

    public void deposit() {
        Match.status("Initializing: deposit (preloaded & SE)");
        queue(() -> outtake.slideToAsync(barcode));
        if (CAROUSEL) {
            queue(fromHere()
                    .lineTo(storageUnitPose.plus(pos(3,-2)).vec()));
            queueCalibration(storageUnitPose);
            queue(fromHere()
                    .lineToLinearHeading(carouselHubPose.plus(pos(0, 4)), getVelocityConstraint(25, 5, TRACK_WIDTH), getAccelerationConstraint(25))
                    .addTemporalMarker(2.75, outtake::dumpOut)
            .waitSeconds(0.5));
        } else if (CYCLE) {
            queue(fromHere()
                    .addTemporalMarker(1, outtake::dumpOut)
                    .lineTo(hubPose.vec()));
            queueLocalizeHub(trueHubPose);
        }
    }

    public void cycle() {
        if (PARK)
            return;
        for (int i = 1; i <= SCORING_CYCLES; i++) {
            Match.status("Initializing: cycle " + i);

            // *** To warehouse ***
            queue(intake::in);
            // TODO: angle offset?
            Pose2d moddedWhPose = whPose.plus(pos(0, i == 1 ? 0 : 2 + (RED ? 2 : 0) + i * 2));
            queue(from(new Pose2d(trueHubPose.getX(), trueHubPose.getY(), drive.getExternalHeading()))
                    .addTemporalMarker(0.2, async(() -> {
                        outtake.dumpIn();
                    }))
                    .addTemporalMarker(0.4, async(() -> {
                        outtake.slideToAsync(ZERO);
                    }))
                    .addSpatialMarker(preWhPose.vec(), async(() -> drive.kalmon()))
//                    .splineToSplineHeading(preWhPose.plus(pos(-4, -11)), deg(RED ? -20 : 20))
                    .splineToSplineHeading(preWhPose.plus(pos(-8, -10)), deg(RED ? -24 : 24))
                    .splineToSplineHeading(moddedWhPose.plus(pos(-14, 0)), deg(RED ? 10 : -10)));// from 3.5
            queueWarehouseSensorCalibration(moddedWhPose);


            // *** Intake ***
            intake(i, moddedWhPose);
            // queue(this::localizeWarehouse);
//            queueWarehouseSensorCalibration(pos(-72 + DriveConstants.WIDTH / 2, 42, 0));
            queue(() -> {
                if (30 - runtime.seconds() <= 5) {
                    throw new InterruptedException();
                }
            });

            // *** To hub ***
            double angleOffset = RED ? -5 : 5;
//            double angleOffset = 0;
            queue(from(moddedWhPose.plus(pos(0, 0, angleOffset))) // FIXME: TELEPORTS
                    .lineToLinearHeading(preWhPose.plus(pos(-4, -4, angleOffset)))
                    .addSpatialMarker(preWhPose.vec(), async(() -> drive.kalmoff()))
                    .addTemporalMarker(1.15, async(() -> {
                        // last-minute check & fix for intake
                        if (dumpIndicator.update() == FULL) {
                            outtake.slideToAsync(SUPERTOP);
                        } else if (dumpIndicator.update() == EMPTY) {
                            intake.in();
                            outtake.slideToAsync(SUPERTOP);
                        } else {
                            intake.out();
                            outtake.slideToAsync(EXCESS);
                            sleep(100);
//                            outtake.dumpExcess();
                        }
                    }))
                    .addTemporalMarker(1.4, async(() -> {
                        if (dumpIndicator.update() == FULL) {
                            outtake.dumpIn();
                            if (!outtake.isApproaching(SUPERTOP))
                                outtake.slideToAsync(SUPERTOP);
                        }
                    }))
                    .addTemporalMarker(1.0, -0.4, outtake::dumpOut)
                    .splineToSplineHeading(cycleHubPose.plus(pos(-3,0)), deg((BLUE ? -1 : 1) * 90)));
            //
             //queueLocalizeHub(trueHubPose);
            queue(() -> drive.setPoseEstimate(new Pose2d(trueHubPose.getX(), trueHubPose.getY(), drive.getExternalHeading())));
            queue(new Pose2d(trueHubPose.getX(), trueHubPose.getY(), drive.getExternalHeading()));


            // *** Score ***
            queue(() -> {
                intake.off();
                if (!outtake.isApproaching(SUPERTOP)) {
                    outtake.slideTo(SUPERTOP);
                }
            });
        }
    }

    public void intake(int iteration, Pose2d startPos) {
//        regularIntake();
        queue(() -> {
            ElapsedTime timer = new ElapsedTime();
            ElapsedTime cloggedTimer = new ElapsedTime();
            boolean clog = false;
            boolean clogFlag = false;
            drive.setEncoderPos(startPos);
            while (dumpIndicator.update() != FULL && timer.milliseconds() < 400) {
                drive.update();
                if (dumpIndicator.update() == EMPTY) {
                    timer.reset();
                    if (!clog) {
                        if ((Math.abs(intake.getMotor().getVelocity()) > 0.1 * Intake.MAX_SPEED)) {
                            cloggedTimer.reset();
                        }
                        intake.getMotor().setVelocity(clogFlag ? -0.6 * Intake.MAX_SPEED : Intake.IN_SPEED);
                        drive.setMotorPowers(0.2);
                    } else {
                        intake.getMotor().setVelocity(Intake.MAX_SPEED);
                        drive.setMotorPowers(-0.3);
                    }
                    if (cloggedTimer.milliseconds() > 1000) {
                        cloggedTimer.reset();
                        clog = false;
                    } else if (cloggedTimer.milliseconds() > 750) {
                        clog = true;
                        clogFlag = true;
                    }
                    outtake.dumpIn();
                } else if (dumpIndicator.update() == OVERFLOW) {
                    drive.setMotorPowers(0); // -0.05
                    intake.getMotor().setVelocity(0.75 * Intake.OUT_SPEED);
                    if (!outtake.isApproaching(EXCESS)) {
                        outtake.slideToAsync(EXCESS);
                    }
//                    outtake.dumpExcess();
                } else {
                    drive.setMotorPowers(0); // -0.05
                    intake.out();
                }
            }
            drive.setMotorPowers(0);
            intake.out();
            if (!outtake.isApproaching(EXCESS)) {
                outtake.slideToAsync(EXCESS);
            }
            outtake.dumpExcess();
//            drive.follow(from(drive.getPoseEstimate()).lineTo(whPose.vec()).build());
//            if (iteration == 2) {
//                if (RED) {
//                    drive.strafeRight(1, 10);
//                } else {
//                    drive.strafeLeft(1, 10);
//                }
//            }
        });
    }

    public void park() {
        Match.status("Initializing: park");
        // park in storage unit
        if (CAROUSEL && PARK) {
            queue(from(carouselHubPose)
                    .addTemporalMarker(1, async(() -> {
                        outtake.dumpIn();
                        outtake.slideToAsync(ZERO);
                    }))
                    .lineToLinearHeading(storageUnitPose.minus(pos(8, 0))));
            return;
        }
        if (CYCLE) {
            // park in warehouse
            queue(from(new Pose2d(trueHubPose.getX(), trueHubPose.getY(), drive.getExternalHeading()))
                    .addTemporalMarker(0.5, async(() -> {
                        intake.in();
                        outtake.dumpIn();
                        outtake.slideToAsync(ZERO);
                    }))
                    .splineToSplineHeading(preWhPose.plus(pos(-3.5, 4)), deg(RED ? 10 : -10))
                    .lineTo(whPose.plus(pos(-3.5, 9)).vec()));
            queue(this::localizeWarehouse);
        }
        queue(shippingArm::resetArmAsync);
    }

    public void getFreight() {
        if (!CYCLE)
            return;
        queue(() -> {
            Match.status("getting additional freight");
            while (opModeIsActive()) {
                if (dumpIndicator.update() == EMPTY) {
                    drive.setMotorPowers(0.2);
                    intake.in();
                } else {
                    drive.setMotorPowers(0);
                    intake.out();
                }
            }
            drive.setMotorPowers(0);
            intake.off();
        });
    }

    /**
     * Park facing the shared hub with one freight in the dump. Use after {@link #park()}.
     */
    public void parkSharedBrainDead() {
        if (!CYCLE)
            return;
        Pose2d transition = pos(-36, 36, 45);
        Pose2d intakePose = pos(-48 + LENGTH / 2, 72 - WIDTH / 2, 90);
        queue(fromHere().lineToLinearHeading(transition).lineToLinearHeading(intakePose));
        queue(() -> {
            drive.setMotorPowers(0.2);
            intake.in();
            while (intakeRampDistance.getDistance(DistanceUnit.INCH) > 3 && dumpIndicator.update() == EMPTY && OpMode.isActive());
            drive.setMotorPowers(0);
            intake.off();
        });
    }

    public void parkSharedSpline() {
        if (!CYCLE)
            return;
        Pose2d transition = pos(-36, 36, 45);
        Pose2d intakePose = pos(-48 + LENGTH / 2, 72 - WIDTH / 2, 90);
        queue(fromHere().lineToLinearHeading(transition).lineToLinearHeading(intakePose));
        queue(() -> {
            drive.setMotorPowers(0.2);
            intake.in();
            while (intakeRampDistance.getDistance(DistanceUnit.INCH) > 3 && dumpIndicator.update() == EMPTY && OpMode.isActive());
            drive.setMotorPowers(0);
            intake.off();
        });
    }

    public void parkShared() {
        Match.status("Initializing: park");
        // park in storage unit
        if (CAROUSEL && PARK) {
            queue(from(trueHubPose)
                    .addTemporalMarker(0.5, async(() -> {
                        outtake.dumpIn();
                        outtake.slideToAsync(ZERO);
                    }))
                    .splineToSplineHeading(storageUnitPose));
            return;
        }
        if (CYCLE) {
            // park in warehouse
            queue(from(trueHubPose)
                    .addTemporalMarker(0.5, async(() -> {
                        intake.in();
                        outtake.dumpIn();
                        outtake.slideToAsync(ZERO);
                    }))
                    .splineToSplineHeading(preWhPose)
                    .lineTo(whPose.minus(pos(3.5, 5)).vec())
                    .now(shippingArm::resetArmAsync)
                    .lineTo(parkPose.vec())
                    .splineToSplineHeading(pos(-42, 44, 90))
                    .splineToSplineHeading(pos(-48 + LENGTH / 2, 72 - WIDTH / 2, 90))
            );
        }
        queueWarehouseSensorCalibration(parkPose);
        queue(shippingArm::resetArmAsync);
    }

    public void startScanner() {
        Match.status("Initializing: vision");
        try {
            initVisionThread.join();
            scanner.start();
        } catch (InterruptedException ignored) {
            Match.status("OpMode interrupted");
        }
    }

    public void scanBarcode() {
        Match.status("Scanning");
        try {
            barcode = scanner.getBarcode();
        } catch (InterruptedException ignored) {
            Match.status("OpMode interrupted");
            Thread.currentThread().interrupt();
        } finally {
            scanner.stop();
        }
    }

    public void epilogue() {
        startScanner();
        Match.status("Task queue ready, waiting for start; Voltage: " + drive.batteryVoltageSensor.getVoltage());
        Match.telemetry.addData("Voltage", drive.batteryVoltageSensor.getVoltage());
        System.out.println("Voltage: " + drive.batteryVoltageSensor.getVoltage());

        waitForStart();
        runtime.reset();
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
        scanner = new PartialBarcodeScanner(camera);
    }
    // endregion

    // region Helper

    public static class ConditionalTask {
        private final Supplier<Boolean> condition;
        private final Object task;

        public ConditionalTask(Supplier<Boolean> condition, Object task) {
            this.condition = condition;
            this.task = task;
        }

        public boolean runnable() {
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
        Match.log("Scoring Cycles = " + SCORING_CYCLES);
        try {
            for (Object task : tasks) {
                // Convert object to actual task, if appropriate
                if (task instanceof String) {
                    if (BuildConfig.DEBUG) {
                        throw new IllegalArgumentException("Please initialize the dynamic task `" + task + "`");
                    } else {
                        continue;
                    }
                } else if (task instanceof ConditionalTask) {
                    if (((ConditionalTask) task).runnable()) {
                        task = ((ConditionalTask) task).getTask();
                    }
                }

                // Execute task based on class
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
    protected void queueLocalizeHub(Pose2d predictedPose) {
        queue(() -> {
            double dist = rangeSensor.get();
            Match.log("hub dist = " + dist);
            if (dist > 30) {
                drive.setPoseEstimate(predictedPose);
                return;
            }
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d correctedPose = new Pose2d(
                    currentPose.getX(),
                    pos(-72 + dist + 6.75, 0).getY(),
                    currentPose.getHeading()
            );
            drive.setPoseEstimate(correctedPose);
        });
        queue(predictedPose);
    }

    protected double getCorrectedXReading() {
        InchSensor xSensor = RED ? rightRangeSensor : leftRangeSensor;
        return 1.11113 * xSensor.get() - 0.241264;
    }

    protected double getCorrectedUltrasonicYReading() {
        return 1.07863 * rangeSensor.get() - 0.318066; // adjust for inaccuracies
    }

    /**
     * Calibrate robot pose at a wall corner using side and forward distance sensors and IMU.
     * @see #queueLocalizeWarehouse(Pose2d)
     */
    protected void localizeWarehouse() {
        // FIXME: replicate all changes to localize* (Hub, Carousel?)
        double heading = drive.getExternalHeading();

        // horizontal distance from wall to sensor
        double wallToXSensor = getCorrectedXReading() * cos(heading);
        // sensor point to robot center point
        Vector2d xSensorToRobot = (RED ? new Vector2d(-5.5, 7.5) : new Vector2d(5.625, -6.916)).rotated(heading);
        // horizontal distance between wall and robot center point
        double xDist;
        if (RED) {
            xDist = -new Vector2d(-wallToXSensor, 0).plus(xSensorToRobot).getX();
        } else {
            xDist = new Vector2d(wallToXSensor, 0).plus(xSensorToRobot).getX();
        }

        // same logic below
        double wallToYUltrasonicSensor = getCorrectedUltrasonicYReading() * cos(heading);
        // TODO: Change
        double wallToDistanceSensor = altRangeSensor.getDistance(DistanceUnit.INCH) * cos(heading);
        Vector2d yUltrasonicSensorToRobot = new Vector2d(3.75, -6.75).rotated(heading);
        Vector2d yDistanceSensorToRobot = new Vector2d(-2, -6).rotated(heading);
        double yUltrasonicDist = new Vector2d(0, -wallToYUltrasonicSensor).plus(yUltrasonicSensorToRobot).getY();
        double yDistanceDist = new Vector2d(0, -wallToDistanceSensor).plus(yDistanceSensorToRobot).getY();
        double yDist = 16;
        // picking the correct distance input
        // when both are giving good readings
        if (yDistanceDist < 48 && yUltrasonicDist < 48) {
            // when ultrasonic sees something on the ground
            if (yDistanceDist - 4 > yUltrasonicDist) {
                yDist = yDistanceDist;
            } else {
                yDist = yUltrasonicDist;
            }
        // when the ultrasonic is giving a bad reading
        } else if (yDistanceDist < 48) {
            yDist = yDistanceDist;
        // when the infared is giving a bad reading
        } else if (yUltrasonicDist < 48) {
            yDist = yUltrasonicDist;
        }

        if (Math.abs(yUltrasonicDist) < 24){
            yUltrasonicDist += ((24-Math.abs(yUltrasonicDist))*0.4);
        }
//        Vector2d correctedVec = pos(-72 + xDist, 72 + yDist).vec();
        Vector2d correctedVec = pos(-72 + xDist, 72 + yUltrasonicDist).vec();
        if (!Double.isFinite(correctedVec.getX()) || !Double.isFinite(correctedVec.getY())){
            return;
        }
//        if (Math.abs(correctedVec.getX()) < 72 && Math.abs(correctedVec.getY()) < 72 && Math.abs(drive.getPoseEstimate().getX()) < Math.abs(correctedVec.getX())) {
        drive.setPoseEstimate(new Pose2d(correctedVec.getX(), correctedVec.getY(), heading));
        System.out.println("oogabooga " + correctedVec.getX());
    }

    protected Pose2d localizeWarehouseReturn() {
        // FIXME: replicate all changes to localize* (Hub, Carousel?)
        double heading = drive.getExternalHeading();

        // horizontal distance from wall to sensor
        double wallToXSensor = getCorrectedXReading() * cos(heading);
        // sensor point to robot center point
        Vector2d xSensorToRobot = (RED ? new Vector2d(-5.5, 7.5) : new Vector2d(5.625, -6.916)).rotated(heading);
        // horizontal distance between wall and robot center point
        double xDist;
        if (RED) {
            xDist = -new Vector2d(-wallToXSensor, 0).plus(xSensorToRobot).getX();
        } else {
            xDist = new Vector2d(wallToXSensor, 0).plus(xSensorToRobot).getX();
        }

        // same logic below
        double wallToYSensor = getCorrectedUltrasonicYReading() * cos(heading);
        Vector2d ySensorToRobot = new Vector2d(3.75, -6.75).rotated(heading);
        double yDist = new Vector2d(0, -wallToYSensor).plus(ySensorToRobot).getY();

        Vector2d correctedVec = pos(-72 + xDist, 72 + yDist).vec();
        if (Double.isInfinite(correctedVec.getX()) || Double.isInfinite(correctedVec.getY()) || Double.isNaN(correctedVec.getX()) || Double.isNaN(correctedVec.getY())){
            return drive.getPoseEstimate();
        }
        if (Math.abs(correctedVec.getX()) < 72 && Math.abs(correctedVec.getY()) < 72 && Math.abs(drive.getPoseEstimate().getX()) < Math.abs(correctedVec.getX())) {
            return new Pose2d(correctedVec.getX(), correctedVec.getY(), heading);
        } else {
            return new Pose2d(correctedVec.getX(), correctedVec.getY(), heading);
        }
    }

    protected void localizeCarousel() {
        double heading = drive.getExternalHeading();

        InchSensor ySensor = RED ? rightRangeSensor : leftRangeSensor;
        double wallToYSensor = ySensor.get() * cos(heading);
        Vector2d ySensorToRobot = (RED ? new Vector2d(-5.5, 7.5) : new Vector2d(5.25, -7.25)).rotated(heading);
        double yDist = new Vector2d(0, wallToYSensor).plus(ySensorToRobot).getY();

        double wallToXSensor = rangeSensor.get() * cos(heading);
        Vector2d xSensorToRobot = new Vector2d(-2, -6.5).rotated(heading);
        double xDist;
        if (RED) {
            xDist = -new Vector2d(-wallToXSensor, 0).plus(xSensorToRobot).getX();
        } else {
            xDist = new Vector2d(wallToXSensor, 0).plus(xSensorToRobot).getX();
        }

        Vector2d correctedVec = pos(-72 + xDist, -72 + yDist).vec();
        if (Math.abs(correctedVec.getX()) < 72 && Math.abs(correctedVec.getY()) < 72 && Math.abs(drive.getPoseEstimate().getX()) < Math.abs(correctedVec.getX())) {
            drive.setPoseEstimate(new Pose2d(correctedVec.getX(), drive.getPoseEstimate().getY(), heading));
        } else {
            drive.setPoseEstimate(new Pose2d(correctedVec.getX(), drive.getPoseEstimate().getY(), heading));
        }
    }


    /**
     * Queues a y-coordinates calibration of the robot's pose estimate at the warehouse. The new
     * pose estimate will calibrate the alliance-agnostic y-coordinates based on the distance sensor
     * reading. Future trajectories will begin at a predicted pose.
     *
     * @param predictedPose the correct pose of the robot
     * @see #localizeWarehouse()
     * @see #pos(double, double, double)
     * @see #queue(Pose2d)
     */
    protected void queueLocalizeWarehouse(Pose2d predictedPose) {
        queue(this::localizeWarehouse);
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
            double dist = rangeSensor.get();
            Match.log("warehouse dist = " + dist);
            if (dist > 30) {
                drive.setPoseEstimate(predictedPose);
                return;
            }
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d correctedPose = new Pose2d(
                    pos(0, 72 - dist - 6.5).getX(),
                    predictedPose.getY(),
                    currentPose.getHeading()
            );
            drive.setPoseEstimate(correctedPose);
        });
        queue(predictedPose);
    }

    /**
     * Queues a x-coordinates calibration of the robot's pose estimate at the shared alliance park. The new pose
     * estimate will calibrate the alliance-agnostic x-coordinates based on the distance sensor
     * reading. Future trajectories will begin at a predicted pose.
     *
     * @param predictedPose the correct pose of the robot
     * @see #pos(double, double, double)
     * @see #queue(Pose2d)
     */
    protected void queueAllianceHubLocalization(Pose2d predictedPose) {
        queue(() -> {
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d correctedPose = new Pose2d(
                    72 - WIDTH / 2,
                    pos(-72 + rangeSensor.getDistance(DistanceUnit.INCH) + 6.5, 0).getY(),
                    currentPose.getHeading()
            );
            drive.setPoseEstimate(correctedPose);
        });
        queue(predictedPose);
    }

    public static MarkerCallback async(Runnable task) {
        return () -> {
            new Thread(task).start();
        };
    }

    public static Thread doAsync(Runnable task) {
        Thread t = new Thread(task);
        t.start();
        return t;
    }

    public void quit() {
        queue(() -> { throw new InterruptedException();});
    }
    // endregion
}