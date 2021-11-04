package org.wolfcorp.ff.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.wolfcorp.ff.BuildConfig;
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
    protected OpenCvCamera camera = null;
    // endregion

    // region Configuration
    protected boolean invert = isRed();
    protected final Boolean DISABLE_QUEUE;
    protected final Boolean USE_VISION;
    // endregion

    // region Vision Fields
    protected BarcodeScanner scanner;
    protected WarehouseGuide guide;
    protected VuforiaNavigator navigator;
    // endregion

    // region Poses
    // All of the following poses assume that the robot starts at blue warehouse
    protected Pose2d initialPose;
    protected Pose2d carouselPose;
    protected Pose2d elementLeftPose;
    protected Pose2d elementMidPose;
    protected Pose2d elementRightPose;
    protected Pose2d hubPose;
    protected Pose2d preWhPose;
    protected Pose2d whPose;
    protected Pose2d parkPose;
    // endregion

    // region Task Queue
    ArrayList<Object> tasks = new ArrayList<>();
    HashMap<String, Object> dynamicTasks = new HashMap<>();
    // endregion

    // region Robot Logic
    public AutonomousMode() {
        USE_VISION = true;
        DISABLE_QUEUE = false;
        initPoses();
    }

    public AutonomousMode(boolean useVision, boolean disableQueue) {
        USE_VISION = useVision;
        DISABLE_QUEUE = disableQueue;
        initPoses();
    }

    private void initPoses() {
        initialPose = pos(-72 + DriveConstants.LENGTH / 2, 12, -90);
        carouselPose = pos(-60, -60, 180);
        elementLeftPose = pos(-72 + DriveConstants.LENGTH / 2, 20.4, -90);
        elementMidPose = pos(-72 + DriveConstants.LENGTH / 2, 12, -90);
        elementRightPose = pos(-72 + DriveConstants.LENGTH / 2, 3.6, -90);
        hubPose = pos(-72 + DriveConstants.LENGTH / 2, -12, -90);
        preWhPose = pos(-72 + DriveConstants.WIDTH / 2, 24 - DriveConstants.LENGTH / 2);
        whPose = pos(-72 + DriveConstants.WIDTH / 2, 36);

        if (isWallRunner())
            parkPose = pos(-60, 36);
        else
            parkPose = pos(-36, 36);

        if (isNearCarousel()) {
            initialPose = initialPose.plus(pos(0, -48));
            elementLeftPose = elementLeftPose.plus(pos(0, -48));
            elementMidPose = elementMidPose.plus(pos(0, -48));
            elementRightPose = elementRightPose.plus(pos(0, -48));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Barcode barcode;
        Thread initVisionThread = new Thread(this::initVision);
        if (USE_VISION)
            initVisionThread.start();

        drive = new Drivetrain(hardwareMap);
        drive.setPoseEstimate(initialPose);

        // *** Carousel ***
        if (isNearCarousel()) {
            queue(fromHere().splineToSplineHeading(carouselPose));
        }

        // *** Barcode & Pre-loaded cube ***
        queue("elementSeq");
        Pose2d preElement = getLastPose();
        TrajectorySequence elementLeftSeq = from(preElement).lineToLinearHeading(elementLeftPose).build();
        TrajectorySequence elementMidSeq = from(preElement).lineToLinearHeading(elementMidPose).build();
        TrajectorySequence elementRightSeq = from(preElement).lineToLinearHeading(elementRightPose).build();
        queue(() -> {
            // TODO: pick up shipping element
        });

        queue("hubSeq");
        TrajectorySequence hubLeftSeq = from(elementLeftPose).strafeTo(hubPose.vec()).build();
        TrajectorySequence hubMidSeq = from(elementMidPose).strafeTo(hubPose.vec()).build();
        TrajectorySequence hubRightSeq = from(elementRightPose).strafeTo(hubPose.vec()).build();
        queue(() -> {
            // TODO: score preloaded freight
        });

        // *** Cycling ***
        Supplier<TrajectorySequence> goToWh =
                () -> fromHere().splineToSplineHeading(preWhPose)
                        .addDisplacementMarker(this::startGuide).lineTo(whPose.vec()).build();
        Supplier<TrajectorySequence> goToHub =
                () -> fromHere().addDisplacementMarker(this::stopGuide)
                        .lineTo(preWhPose.vec()).splineToSplineHeading(hubPose).build();
        queue(goToWh);
        // TODO: pick up freight
        queue(goToHub);
        // TODO: score freight
        // TODO: put above in a loop

        // *** Park ***
        queue(fromHere().splineToSplineHeading(preWhPose).lineTo(whPose.vec()).lineTo(parkPose.vec()));

        // *** Wrapping Up ***
        if (USE_VISION) {
            initVisionThread.join();
            scanner.start();
            telemetry.addLine("BarcodeScanner started");
            telemetry.addLine("Waiting for start");
            telemetry.update();
        }
        waitForStart();

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
            dynamicTasks.put("elementSeq", elementMidSeq);
            dynamicTasks.put("hubSeq", hubMidSeq);
        }

        runTasks();
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
    private void runTasks() {
        for (Object task : tasks) {
            if (task instanceof String) {
                if (dynamicTasks.containsKey(task))
                    task = dynamicTasks.get(task);
                else if (BuildConfig.DEBUG) {
                    throw new IllegalArgumentException("Please initialize the dynamic task `" + task + "`");
                }
                else
                    continue;
            }

            if (task instanceof TrajectorySequence) {
                drive.follow((TrajectorySequence) task);
            }
            else if (task instanceof Runnable){
                ((Runnable) tasks).run();
            }
        }
        tasks.clear();
    }

    // Rotate the coordinate plane 90 degrees clockwise (positive y-axis points at the shared hub)
    // Basically converts a point from Cartesian to Roadrunner
    public Pose2d pos(double x, double y) {
        return invert ? new Pose2d(+y, +x) : new Pose2d(+y, -x);
    }

    // Rotate the coordinate plane 90 degrees clockwise (positive y-axis points at the shared hub)
    // Basically converts a point from Cartesian to Roadrunner
    // The positive y-axis represents a heading of 0 degree
    public Pose2d pos(double x, double y, double heading) {
        if (invert)
            return new Pose2d(+y, +x, Math.toRadians(-heading));
        else
            return new Pose2d(+y, -x, Math.toRadians(heading));
    }

    public boolean isRed() {
        return this.getClass().getSimpleName().contains("Red");
    }

    public boolean isNearCarousel() {
        return this.getClass().getSimpleName().contains("Carousel");
    }

    public boolean isWallRunner() {
        return this.getClass().getSimpleName().contains("WR");
    }

    protected void queue(Object o) {
        if (!DISABLE_QUEUE)
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
    protected double queue(Pose2d pose) {
        if (!DISABLE_QUEUE)
            tasks.add(pose);
        return tasks.size() - 1;
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
        if (USE_VISION)
            guide.start();
    }

    protected void stopGuide() {
        if (USE_VISION)
            guide.stop();
    }
    // endregion
}
