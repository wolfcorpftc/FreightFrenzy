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
    protected StartingLocation location;
    protected boolean wallrunner;
    protected boolean invert;

    // All of the following poses assume that the robot starts at blue warehouse
    protected Pose2d initialPose;
    protected Pose2d carouselPose;
    protected Pose2d elementPose;
    protected Pose2d hubPose;
    protected Pose2d preWhPose;
    protected Pose2d whPose;
    protected Pose2d parkPose;

    ArrayList<Object> tasks = new ArrayList<>();

    public AutonomousMode(StartingLocation loc, boolean wr) {
        location = loc;
        wallrunner = wr;
        invert = isRed();

        // TODO: add heading
        // TODO: take robot width into account
        initialPose = pos(-72, 12);
        carouselPose = pos(-60, -60);
        elementPose = pos(-72, -72);
        hubPose = pos(-72, 12);
        preWhPose = pos(-72, 72);
        whPose = pos(12, 72);
        if (wallrunner)
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
        Barcode barcode = null;

        BarcodeScanner scanner = new BarcodeScanner(webcam, telemetry);
        WarehouseGuide guide = new WarehouseGuide(webcam);

        drive = new Drivetrain(hardwareMap);
        drive.setPoseEstimate(initialPose);

        // *** Carousel ***
        if (isNearCarousel()) {
            queue(fromHere().splineToSplineHeading(carouselPose).build());
        }
        // *** Barcode ***
        queue(fromHere().splineToSplineHeading(hubPose).build());
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
        queue(fromHere().splineToSplineHeading(preWhPose).lineTo(whPose.vec()).lineTo(parkPose.vec()).build());

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

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
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
            else {
                ((Runnable) tasks).run();
            }
        }
        tasks.clear();
    }

    // *** Helper methods ***

    public Pose2d pos(double x, double y) {
        return invert ? new Pose2d(-y, -x) : new Pose2d(+y, -x);
    }

    public Pose2d pos(double x, double y, double angle) {
        // Roadrunner shouldn't care but we do it to be safe.
        angle = ((angle - 90) % 360 + 360) % 360;
        if (invert)
            return new Pose2d(-y, -x, Math.toRadians(-angle));
        else
            return new Pose2d(+y, -x, Math.toRadians(angle));
    }

    public boolean isRed() { return location.toString().startsWith("RED"); }

    public boolean isBlue() { return !isRed(); }

    public boolean isNearCarousel() { return location.toString().endsWith("CA"); }

    public boolean isNearWarehouse() { return !isNearCarousel(); }

    protected void queue(TrajectorySequence seq) {
        tasks.add(seq);
    }

    protected void queue(Supplier<TrajectorySequence> seq) {
        tasks.add(seq.get());
    }

    protected void queue(Runnable run) {
        tasks.add(run);
    }

    protected Pose2d getLastPose() {
        for (int i = tasks.size() - 1; i >= 0; i--) {
            if (tasks.get(i) instanceof TrajectorySequence) {
                return ((TrajectorySequence) tasks.get(i)).end();
            }
        }
        return initialPose;
    }

    protected TrajectorySequenceBuilder fromHere() {
        return drive.from(getLastPose());
    }
}
