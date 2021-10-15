package org.wolfcorp.ff.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.trajectorysequence.TrajectorySequence;
import org.wolfcorp.ff.trajectorysequence.TrajectorySequenceBuilder;
import org.wolfcorp.ff.vision.Barcode;
import org.wolfcorp.ff.vision.BarcodeScanner;

import java.util.ArrayList;
import java.util.concurrent.Callable;
import java.util.function.Supplier;

public abstract class AutonomousMode extends LinearOpMode {
    protected StartingLocation location;
    protected boolean wallrunner;
    protected boolean invert = false;

    // blue side warehouse
    protected Pose2d initialPose;
    protected Pose2d carouselPose;
    protected Pose2d elementPose;
    protected Pose2d hubPose;
    protected Pose2d preWhPose;
    protected Pose2d whPose;
    protected Pose2d parkPose;

    protected Drivetrain drive = null;

    ArrayList<Object> tasks = new ArrayList<>();

    public AutonomousMode(StartingLocation loc, boolean wr) {
        location = loc;
        wallrunner = wr;

        if (isRed())
            invert = true;

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

    @Override
    public void runOpMode() throws InterruptedException {
        Barcode barcode = null;
        BarcodeScanner scanner = new BarcodeScanner(telemetry, hardwareMap);
        drive = new Drivetrain(hardwareMap);
        drive.setPoseEstimate(initialPose);

        // *** Carousel ***
        if (isNearCarousel()) {
            addTask(fromHere().splineToSplineHeading(carouselPose).build());
        }
        // *** Barcode ***
        addTask(fromHere().splineToSplineHeading(hubPose).build());
        addTask(() -> {
            // TODO: score preloaded freight
        });

        // *** Cycling ***
        Supplier<TrajectorySequence> goToWh =
                () -> fromHere().splineToSplineHeading(preWhPose).lineTo(whPose.vec()).build();
        Supplier<TrajectorySequence> goToHub =
                () -> fromHere().lineTo(preWhPose.vec()).splineToSplineHeading(hubPose).build();
        addTask(goToWh);
        // TODO: pick up freight
        addTask(goToHub);
        // TODO: score freight
        // TODO: put above in a loop

        // *** Park ***
        addTask(fromHere().splineToSplineHeading(preWhPose).lineTo(whPose.vec()).lineTo(parkPose.vec()).build());
        waitForStart();

        barcode = scanner.getBarcode();
        scanner.stop();
        for (Object task : tasks) {
            if (task instanceof TrajectorySequence) {
                drive.follow((TrajectorySequence) task);
            }
            else {
                ((Runnable) tasks).run();
            }
        }
    }

    public Pose2d pos(double x, double y) {
        if (invert)
            return new Pose2d(-y, -x);
        return new Pose2d(+y, -x);
    }

    public Pose2d pos(double x, double y, double angle) {
        // Roadrunner shouldn't care but we do it to be safe.
        angle = ((angle - 90) % 360 + 360) % 360;
        if (invert)
            return new Pose2d(-y, -x, Math.toRadians(-angle));
        return new Pose2d(+y, -x, Math.toRadians(angle));
    }

    public boolean isRed() { return location.toString().startsWith("RED"); }
    public boolean isBlue() { return !isRed(); }
    public boolean isNearCarousel() { return location.toString().endsWith("CA"); }
    public boolean isNearWarehouse() { return !isNearCarousel(); }
    protected Pose2d getLastPose() {
        for (int i = tasks.size() - 1; i >= 0; i--) {
            if (tasks.get(i) instanceof TrajectorySequence) {
                return ((TrajectorySequence) tasks.get(i)).end();
            }
        }
        return initialPose;
    }
    // Helper methods (to enforce type)
    protected void addTask(TrajectorySequence seq) {
        tasks.add(seq);
    }

    protected void addTask(Supplier<TrajectorySequence> seq) {
        tasks.add(seq.get());
    }

    protected void addTask(Runnable run) {
        tasks.add(run);
    }

    protected TrajectorySequenceBuilder fromHere() {
        return drive.from(getLastPose());
    }
}
