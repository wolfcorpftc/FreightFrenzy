package org.wolfcorp.ff.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.trajectorysequence.TrajectorySequence;

public class AutoMode extends LinearOpMode {
    public Pose2d pos(double x, double y) {
        return new Pose2d(+y, -x);
    }

    public Pose2d pos(double x, double y, double angle) {
        // Roadrunner shouldn't care but we do it to be safe.
        angle = ((angle - 90) % 360 + 360) % 360;
        return new Pose2d(+y, -x, Math.toRadians(angle));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap);
        Pose2d initialPose = pos(-72, -36);
        drive.setPoseEstimate(initialPose);
        TrajectorySequence t1 = drive
                .from(initialPose)
                .build();
    }
}
