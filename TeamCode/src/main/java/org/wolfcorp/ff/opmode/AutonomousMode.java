package org.wolfcorp.ff.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.wolfcorp.ff.BuildConfig;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.DriveConstants;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequence;
import org.wolfcorp.ff.robot.trajectorysequence.TrajectorySequenceBuilder;
import org.wolfcorp.ff.vision.Barcode;
import org.wolfcorp.ff.vision.BarcodeScanner;
import org.wolfcorp.ff.vision.VuforiaNavigator;
import org.wolfcorp.ff.vision.WarehouseGuide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

public abstract class AutonomousMode extends LinearOpMode {
    // region Hardware
    protected Drivetrain drive = null;
    protected CarouselSpinner spinner = null;
    protected OpenCvCamera camera = null;
    // endregion

    // region Configuration
    public final boolean USE_VISION = !this.getClass().getSimpleName().contains("NV");
    public final boolean CAROUSEL = this.getClass().getSimpleName().contains("Carousel");
    public final boolean WALL_RUNNER = this.getClass().getSimpleName().contains("WR");

    public static final int SCORING_CYCLES = 4;
    // endregion

    // region Vision Fields
    protected BarcodeScanner scanner;
    protected WarehouseGuide guide;
    protected VuforiaNavigator navigator;
    protected Barcode barcode;
    // endregion

    // region Poses
    // Poses are declared in order of appearance in paths.
    // When initializing poses with pos(), assume that the robot starts at blue warehouse.
    protected Pose2d initialPose; // where the robot starts

    protected Pose2d fakePreCarouselPose; // bang against the wall for calibration of y
    protected Pose2d preCarouselPose; // actual pose before driving toward carousel
    protected Pose2d carouselPose; // turn the carousel
    protected Pose2d postCarouselPose; // move away from the carousel to allow for turns
    protected Pose2d fakeInitialPose; // bang against the other wall for calibration of x

    protected Pose2d elementLeftPose; // retrieve the left shipping element
    protected Pose2d elementMidPose; // retrieve the middle shipping element
    protected Pose2d elementRightPose; // retrieve the right shipping element

    protected Pose2d hubPose; // score freight into hub
    protected Pose2d whPose; // load freight from warehouse
    protected Pose2d parkPose; // where the robot parks
    // endregion

    // region Task Queue
    private final ArrayList<Object> tasks = new ArrayList<>();
    private final HashMap<String, Object> dynamicTasks = new HashMap<>();
    // endregion

    // region Robot Logic
    public AutonomousMode() {
        Match.isRed = this.getClass().getSimpleName().contains("Red");

        // Faster telemetry
        telemetry.setMsTransmissionInterval(50);

        initialPose = pos(-72 + DriveConstants.WIDTH / 2, 12, 180);
        fakeInitialPose = initialPose.minus(pos(3, 0));

        carouselPose = pos(-50.5, -72 + DriveConstants.WIDTH / 2, 90);
        preCarouselPose = carouselPose.plus(pos(1.5, 0));
        fakePreCarouselPose = preCarouselPose.minus(pos(0, 3));
        postCarouselPose = carouselPose.plus(pos(0, 5));

        elementLeftPose = pos(-72 + DriveConstants.LENGTH / 2, 20.4, 180);
        elementMidPose = elementLeftPose.minus(pos(0, 8.4));
        elementRightPose = elementMidPose.minus(pos(0, 8.4));

        hubPose = pos(-72 + DriveConstants.WIDTH / 2, -12, 180);
        whPose = pos(-72 + DriveConstants.WIDTH / 2, 46, 180);

        if (WALL_RUNNER) {
            parkPose = pos(-72 + DriveConstants.WIDTH / 2, 37, 180);
        }
        else {
            parkPose = pos(-36, 37, 180);
        }

        if (CAROUSEL) {
            initialPose = initialPose.minus(pos(0, 48));
            fakeInitialPose = fakeInitialPose.minus(pos(0, 48));
            elementLeftPose = elementLeftPose.minus(pos(0, 48));
            elementMidPose = elementMidPose.minus(pos(0, 48));
            elementRightPose = elementRightPose.minus(pos(0, 48));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        log("AutonomousMode runOpMode; initializing robot");
        // *** Initialization ***
        Thread initVisionThread = new Thread(this::initVision);
        if (USE_VISION) {
            initVisionThread.start();
        }

        drive = new Drivetrain(hardwareMap);
        drive.setPoseEstimate(initialPose);

        spinner = new CarouselSpinner(hardwareMap, this::sleep);

        log("Robot Initialized, preparing task queue");

        // *** Carousel ***
        if (CAROUSEL) {
            log("Initializing: carousel");
            queue(fromHere().lineToLinearHeading(fakePreCarouselPose));
            queue(() -> drive.setPoseEstimate(preCarouselPose));
            queue(from(preCarouselPose).lineTo(carouselPose.vec()));
            queue((Runnable) spinner::spin);
            queue(fromHere().lineTo(postCarouselPose.vec()));
            queue(fromHere().lineToLinearHeading(fakeInitialPose));
            queue(() -> drive.setPoseEstimate(initialPose));
            queue(initialPose);
        }

        // *** Barcode & Pre-loaded cube ***
        queue("elementSeq");
        TrajectorySequence elementLeftSeq = fromHere().lineToLinearHeading(elementLeftPose).build();
        TrajectorySequence elementMidSeq = fromHere().lineToLinearHeading(elementMidPose).build();
        TrajectorySequence elementRightSeq = fromHere().lineToLinearHeading(elementRightPose).build();
        queue(() -> {
            // TODO: pick up shipping element
        });

        queue("hubSeq");
        TrajectorySequence hubLeftSeq = from(elementLeftPose).lineToLinearHeading(hubPose).build();
        TrajectorySequence hubMidSeq = from(elementMidPose).lineToLinearHeading(hubPose).build();
        TrajectorySequence hubRightSeq = from(elementRightPose).lineToLinearHeading(hubPose).build();
        queue(() -> {
            // TODO: score preloaded freight
        });

        // *** Cycling ***
        TrajectorySequence goToWh = fromHere().now(this::startGuide).lineTo(whPose.vec()).build();
        TrajectorySequence goToHub = fromHere().now(this::stopGuide).lineTo(hubPose.vec()).build();
        for (int i = SCORING_CYCLES; i >= 1; i--) {
            queue(goToWh);
            queue(() -> {
                // TODO: pick up freight
            });
            queue(goToHub);
            queue(() -> {
                // TODO: score freight
            });
        }

        // *** Park ***
        queue(fromHere().lineTo(whPose.vec()).lineTo(parkPose.vec()));

        // *** Wrapping Up ***
        if (USE_VISION) {
            initVisionThread.join();
            scanner.start();
            log("BarcodeScanner started");
        }

        log("Task queue ready, waiting for start");

        waitForStart();

        log("Start!");

        // *** Scan Barcode ***
        if (USE_VISION) {
            barcode = scanner.getBarcode();
            scanner.stop();
            switch (barcode) {
                case TOP: // left
                    dynamicTasks.put("elementSeq", elementLeftSeq);
                    dynamicTasks.put("hubSeq", hubLeftSeq);
                    break;
                case MID: // mid
                    dynamicTasks.put("elementSeq", elementMidSeq);
                    dynamicTasks.put("hubSeq", hubMidSeq);
                    break;
                case BOT: // right
                    dynamicTasks.put("elementSeq", elementRightSeq);
                    dynamicTasks.put("hubSeq", hubRightSeq);
                    break;
            }
        }
        else {
            dynamicTasks.put("elementSeq", elementLeftSeq);
            dynamicTasks.put("hubSeq", hubLeftSeq);
        }

        runTasks();

        sleep(1000);
        Match.teleOpInitialPose = drive.getPoseEstimate();
        Match.hubPose = hubPose;
    }
    // endregion

    // region Vision Initialization
    protected void initVision() {
        initVisionPassthru();
        scanner = new BarcodeScanner(camera, telemetry);
        // TODO: compare TFWarehouseGuide and WarehouseGuide performance
        guide = new WarehouseGuide(camera);
    }

    private void initVisionPassthru() {
        navigator = new VuforiaNavigator(hardwareMap, telemetry);
        camera = navigator.createOpenCvPassthru();
    }

    private void initVisionWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        ((OpenCvWebcam) camera).setMillisecondsPermissionTimeout(2500);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {}

            @Override
            public void onError(int errorCode) {}
        });
    }
    // endregion

    // region Helper Methods
    protected void runTasks() {
        log("Running tasks...");
        for (Object task : tasks) {
            if (task instanceof String) {
                if (dynamicTasks.containsKey(task)) {
                    task = dynamicTasks.get(task);
                }
                else if (BuildConfig.DEBUG) {
                    throw new IllegalArgumentException("Please initialize the dynamic task `" + task + "`");
                }
                else {
                    continue;
                }
            }

            if (task instanceof TrajectorySequence) {
                drive.follow((TrajectorySequence) task);
            }
            else if (task instanceof Runnable){
                ((Runnable) task).run();
            }
        }
        tasks.clear();
    }

    // Rotate the coordinate plane 90 degrees clockwise (positive y-axis points at the shared hub)
    // Basically converts a point from Cartesian to Roadrunner
    public Pose2d pos(double x, double y) {
        return Match.isRed ? new Pose2d(+y, +x) : new Pose2d(+y, -x);
    }

    // Rotate the coordinate plane 90 degrees clockwise (positive y-axis points at the shared hub)
    // Basically converts a point from Cartesian to Roadrunner
    // The positive y-axis represents a heading of 0 degree
    public Pose2d pos(double x, double y, double heading) {
        if (Match.isRed) {
            return new Pose2d(+y, +x, Math.toRadians(heading + 180));
        }
        else {
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

    protected void queue(Runnable run) {
        queue((Object) run);
    }

    // Set the last pose manually when robot.turn() is used between trajectory sequences
    protected void queue(Pose2d pose) {
        queue((Object) pose);
    }

    protected Barcode getBarcode() {
        return barcode;
    }

    protected Pose2d getLastPose() {
        for (int i = tasks.size() - 1; i >= 0; i--) {
            if (tasks.get(i) instanceof TrajectorySequence) {
                return ((TrajectorySequence) tasks.get(i)).end();
            }
            else if (tasks.get(i) instanceof Pose2d) {
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
        if (USE_VISION) {
            guide.start();
        }
    }

    protected void stopGuide() {
        if (USE_VISION) {
            guide.stop();
        }
    }

    protected void log(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }
    // endregion
}
