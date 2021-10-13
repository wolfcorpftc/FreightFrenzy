package org.wolfcorp.ff.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.trajectorysequence.TrajectorySequence;
import org.wolfcorp.ff.vision.Barcode;
import org.wolfcorp.ff.vision.BarcodeScanner;

public abstract class AutonomousMode extends LinearOpMode {
    protected StartingLocation location;
    protected boolean wallrunner = true;
    protected boolean invert = false;

    // blue side warehouse
    protected Pose2d initialPose;
    protected Pose2d elementPose;
    protected Pose2d hubPose;
    protected Pose2d whPose;
    protected Pose2d parkPose;

    protected Drivetrain drive = null;

    public AutonomousMode(StartingLocation loc, boolean wr) {
        location = loc;
        wallrunner = wr;

        if (isRed())
            invert = true;

        // TODO: add heading
        // TODO: take robot width into account
        initialPose = pos(-72, 12);
        elementPose = pos(-72, -72);
        hubPose = pos(-72, 12);
        whPose = pos(-72, 72);
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
        // TODO: set up traj seq
        waitForStart();
        barcode = scanner.getBarcode();
        scanner.stop();
        drive.setPoseEstimate(initialPose);
//        TrajectorySequence t1 = drive
//                .from(initialPose)
//                .build();
        if (isNearCarousel()) {
            // TODO: carousel
        }
        // TODO: barcode
        if (wallrunner) {
            // TODO: cycle WR
        } else {
            // TODO: cycle over barrier
        }
        // TODO: park
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

}
