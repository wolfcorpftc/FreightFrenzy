package org.wolfcorp.ff.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.trajectorysequence.TrajectorySequence;
import org.wolfcorp.ff.vision.Barcode;
import org.wolfcorp.ff.vision.BarcodeScanner;

import java.util.ArrayList;

public class AutoMode extends LinearOpMode {
    protected boolean invert = false;
    protected StartingLocation location = StartingLocation.BLU_WH;

    // blue side warehouse
    protected Pose2d initialPose;
    protected Pose2d elementPose;
    protected Pose2d hubPose;
    protected Pose2d whPose;

    public AutoMode() {
        if (isRed())
            invert = true;

        initialPose = pos(-72, 12);
        elementPose = pos(-72, -72);
        hubPose = pos(-72, 12);
        whPose = pos(-72, 72);

        Pose2d[] poses = {initialPose, elementPose, hubPose, whPose};
        if (isCarousel())
            for (Pose2d pose : poses)
                initialPose.plus(pos(0, -48));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Barcode barcode = null;
        BarcodeScanner scanner = new BarcodeScanner(telemetry, hardwareMap);
        Drivetrain drive = new Drivetrain(hardwareMap);
        // TODO: set up traj seq
        waitForStart();
        barcode = scanner.getBarcode();
        scanner.stop();
        drive.setPoseEstimate(initialPose);
        TrajectorySequence t1 = drive
                .from(initialPose)
                .build();
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
    public boolean isBlue() { return location.toString().startsWith("BLU"); }
    public boolean isCarousel() { return location.toString().endsWith("CA"); }
    public boolean isWarehouse() { return location.toString().endsWith("WH"); }

}
