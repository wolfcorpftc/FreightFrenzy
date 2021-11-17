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
    static protected AutonomousMode instance = null;

    // region Hardware
    protected Drivetrain drive = null;
    protected CarouselSpinner spinner = null;
    protected OpenCvCamera camera = null;
    // endregion

    // region Configuration
    public final boolean USE_QUEUE = !this.getClass().getSimpleName().contains("NQ");
    public final boolean USE_VISION = !this.getClass().getSimpleName().contains("NV");
    public final boolean CAROUSEL = this.getClass().getSimpleName().contains("Carousel");
    public final boolean WALL_RUNNER = this.getClass().getSimpleName().contains("WR");
    public final boolean RED = this.getClass().getSimpleName().contains("Red");

    public static final int SCORING_CYCLES = 4;
    // endregion

    // region Vision Fields
    protected BarcodeScanner scanner;
    protected WarehouseGuide guide;
    protected VuforiaNavigator navigator;
    protected Barcode barcode;
    // endregion

    // region Poses
    // All of the following poses assume that the robot starts at blue warehouse
    protected Pose2d initialPose;
    protected Pose2d carouselPose;
    protected Pose2d elementLeftPose;
    protected Pose2d elementMidPose;
    protected Pose2d elementRightPose;
    protected Pose2d hubPose;
    protected Pose2d whPose;
    protected Pose2d parkPose;
    // endregion

    // region Task Queue
    private final ArrayList<Object> tasks = new ArrayList<>();
    private final HashMap<String, Object> dynamicTasks = new HashMap<>();
    // endregion

    // region Robot Logic
    public AutonomousMode() {
        instance = this;

        initialPose = pos(-72 + DriveConstants.WIDTH / 2, 12, 180);
        carouselPose = pos(-55, -74 + DriveConstants.WIDTH / 2, 90);
        elementLeftPose = pos(-72 + DriveConstants.LENGTH / 2, 20.4, 180);
        elementMidPose = pos(-72 + DriveConstants.LENGTH / 2, 12, 180);
        elementRightPose = pos(-72 + DriveConstants.LENGTH / 2, 3.6, 180);
        hubPose = pos(-72 + DriveConstants.WIDTH / 2, -12, 180);
        whPose = pos(-72 + DriveConstants.WIDTH / 2, 46, 180);

        if (WALL_RUNNER)
            parkPose = pos(-60, 36, 180);
        else
            parkPose = pos(-36, 36, 180);

        if (CAROUSEL) {
            initialPose = initialPose.plus(pos(0, -48));
            elementLeftPose = elementLeftPose.plus(pos(0, -48));
            elementMidPose = elementMidPose.plus(pos(0, -48));
            elementRightPose = elementRightPose.plus(pos(0, -48));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
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
            queue(fromHere().lineToLinearHeading(carouselPose));
            queue(() -> {
                drive.setPoseEstimate(pos(-54, -72 + DriveConstants.WIDTH / 2, 90));
                spinner.spin();
            });
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
        PoseStorage.currentPose = drive.getPoseEstimate();
        PoseStorage.hubPose = hubPose;

        // allow the current object to be GC'd
        instance = null;
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

    public static AutonomousMode getInstance() {
        return instance;
    }

    // Rotate the coordinate plane 90 degrees clockwise (positive y-axis points at the shared hub)
    // Basically converts a point from Cartesian to Roadrunner
    public Pose2d pos(double x, double y) {
        return RED ? new Pose2d(+y, +x) : new Pose2d(+y, -x);
    }

    // Rotate the coordinate plane 90 degrees clockwise (positive y-axis points at the shared hub)
    // Basically converts a point from Cartesian to Roadrunner
    // The positive y-axis represents a heading of 0 degree
    public Pose2d pos(double x, double y, double heading) {
        if (RED) {
            return new Pose2d(+y, +x, Math.toRadians(heading + 180));
        }
        else {
            return new Pose2d(+y, -x, Math.toRadians(heading));
        }
    }

    protected void queue(Object o) {
        if (USE_QUEUE) {
            tasks.add(o);
        }
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
