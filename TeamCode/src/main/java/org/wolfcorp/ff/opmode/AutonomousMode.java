package org.wolfcorp.ff.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.trajectorysequence.TrajectorySequence;
import org.wolfcorp.ff.trajectorysequence.TrajectorySequenceBuilder;
import org.wolfcorp.ff.vision.Barcode;
import org.wolfcorp.ff.vision.BarcodeScanner;
import org.wolfcorp.ff.vision.WarehouseGuide;

import java.util.ArrayList;
import java.util.function.Supplier;

public abstract class AutonomousMode extends LinearOpMode {
    // Hardware
    protected Drivetrain drive = null;
    protected OpenCvWebcam webcam = null;

    // Configuration
    protected boolean invert = isRed();
    protected boolean testVision = false;

    // All of the following poses assume that the robot starts at blue warehouse
    protected Pose2d initialPose;
    protected Pose2d carouselPose;
    protected Pose2d elementPose;
    protected Pose2d hubPose;
    protected Pose2d preWhPose;
    protected Pose2d whPose;
    protected Pose2d parkPose;

    ArrayList<Object> tasks = new ArrayList<>();

    public AutonomousMode() {
        // TODO: add heading
        // TODO: take robot width into account
        initialPose = pos(-72, 12);
        carouselPose = pos(-60, -60);
        elementPose = pos(-72, -72);
        hubPose = pos(-72, 12);
        preWhPose = pos(-72, 72);
        whPose = pos(12, 72);
        if (isWallrunner())
            parkPose = pos(-60, 36);
        else
            parkPose = pos(-48, 36);

        Pose2d[] poses = {initialPose, elementPose, hubPose, whPose};
        if (isNearCarousel())
            for (Pose2d pose : poses)
                initialPose.plus(pos(0, -48));
    }

    // *** Core logic ***

    @Override
    public void runOpMode() throws InterruptedException {
        initCam();

        BarcodeScanner scanner = new BarcodeScanner(webcam, telemetry);
        Barcode barcode = null;
        WarehouseGuide guide = new WarehouseGuide(webcam);

        drive = new Drivetrain(hardwareMap);
        drive.setPoseEstimate(initialPose);

        // *** Carousel ***
        if (isNearCarousel()) {
            queue(fromHere().splineToSplineHeading(carouselPose));
        }
        // *** Barcode ***
        queue(fromHere().splineToSplineHeading(hubPose));
        queue(() -> {
            // TODO: score preloaded freight
        });

        // *** Cycling ***
        Supplier<TrajectorySequence> goToWh =
                () -> fromHere().splineToSplineHeading(preWhPose)
                        .addDisplacementMarker(guide::start).lineTo(whPose.vec()).build();
        Supplier<TrajectorySequence> goToHub =
                () -> fromHere().addDisplacementMarker(guide::stop)
                        .lineTo(preWhPose.vec()).splineToSplineHeading(hubPose).build();
        queue(goToWh);
        // TODO: pick up freight
        queue(goToHub);
        // TODO: score freight
        // TODO: put above in a loop

        // *** Park ***
        queue(fromHere().splineToSplineHeading(preWhPose).lineTo(whPose.vec()).lineTo(parkPose.vec()));

        scanner.start();
        waitForStart();

        barcode = scanner.getBarcode();
        scanner.stop();

        // *** Empty queue (run tasks) ***
        runTasks();
    }

    private void initCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {}

            @Override
            public void onError(int errorCode) {}
        });
    }

    private void runTasks() {
        for (Object task : tasks) {
            if (task instanceof TrajectorySequence) {
                drive.follow((TrajectorySequence) task);
            }
            else if (task instanceof Runnable){
                ((Runnable) tasks).run();
            }
        }
        tasks.clear();
    }

    // *** Helper methods ***

    // Rotate the coordinate plane 90 degrees clockwise (positive y-axis points at the shared hub)
    public Pose2d pos(double x, double y) {
        return invert ? new Pose2d(-y, -x) : new Pose2d(+y, -x);
    }

    // Rotate the coordinate plane 90 degrees clockwise (positive y-axis points at the shared hub)
    // The positive y-axis represents a heading of 0 degree
    public Pose2d pos(double x, double y, double heading) {
        // Roadrunner shouldn't care but we do it to be safe.
        if (invert)
            return new Pose2d(-y, -x, Math.toRadians(-heading));
        else
            return new Pose2d(+y, -x, Math.toRadians(heading));
    }

    public boolean isRed() { return this.getClass().getSimpleName().contains("RED"); }

    public boolean isBlue() { return !isRed(); }

    public boolean isNearCarousel() { return this.getClass().getSimpleName().contains("CA"); }

    public boolean isNearWarehouse() { return !isNearCarousel(); }

    public boolean isWallrunner() { return this.getClass().getSimpleName().contains("WR"); }

    protected void queue(TrajectorySequence seq) {
        tasks.add(seq);
    }

    protected void queue(TrajectorySequenceBuilder seqBuilder) {
        if (!testVision)
            tasks.add(seqBuilder.build());
    }

    protected void queue(Supplier<TrajectorySequence> seq) {
        tasks.add(seq.get());
    }

    protected void queue(Runnable run) {
        tasks.add(run);
    }

    // Set the last pose manually when robot.turn() is used between trajectory sequences
    protected void queue(Pose2d pose) { tasks.add(pose); }

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
}
