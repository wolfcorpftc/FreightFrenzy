package org.wolfcorp.ff.sim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

import java.util.Arrays;

public class AutoTest {
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
    protected Pose2d cycleHubPose;
    protected Pose2d whPose; // load freight from warehouse
    protected Pose2d parkPose; // where the robot parks
    protected Pose2d preHubPose;
    protected Pose2d preWhPose;

    protected double length = 15;
    protected double width = 13;

    protected boolean isRed = false;
    protected boolean isWallRunner = true;
    protected boolean isNearCarousel = false;

    protected Pose2d sharedParkPose;

    public void initPoses() {
        if (isRed) {
            initialPose = pos(-72 + length / 2 + 1 /* gap */, width / 2, 90);
        } else {
            initialPose = pos(-72 + length / 2 + 1 /* gap */, 24 - width / 2, 90);
        }
        fakeInitialPose = initialPose.minus(pos(3, 0));

        carouselPose = pos(-50.5, -72 + width / 2, 90);
        preCarouselPose = carouselPose.plus(pos(1.5, 0));
        fakePreCarouselPose = preCarouselPose.minus(pos(0, 3));
        postCarouselPose = carouselPose.plus(pos(0, 5));

        elementLeftPose = pos(-72 + length / 2, 20.4, 180);
        elementMidPose = elementLeftPose.minus(pos(0, 8.4));
        elementRightPose = elementMidPose.minus(pos(0, 8.4));

        whPose = pos(-72 + width / 2, 42, 180);
        hubPose = pos(-48.5 + width / 2, -12, 90);
        cycleHubPose = pos(-48, -11.5, 90);
        preHubPose = pos(-48, -12, 0);
        preWhPose = pos(-72 + width / 2, 12, 0);

        sharedParkPose = pos(-36,72-width/2,90);

        if (isWallRunner) {
            parkPose = pos(-72 + width / 2, 37, 180);
        }
        else {
            parkPose = pos(-36, 37, 180);
        }

        if (isNearCarousel) {
            initialPose = initialPose.minus(pos(0, 48));
            fakeInitialPose = fakeInitialPose.minus(pos(0, 48));
            elementLeftPose = elementLeftPose.minus(pos(0, 48));
            elementMidPose = elementMidPose.minus(pos(0, 48));
            elementRightPose = elementRightPose.minus(pos(0, 48));
        }

    }


    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }


    public void start() {
        initPoses();
        initialPose = whPose;

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(600)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setBotDimensions(width,length)
                .followTrajectorySequence(drive -> drive
                        .trajectorySequenceBuilder(initialPose)
                        .splineToConstantHeading(preWhPose.minus(pos(2, 10)).vec(), deg(180))
                        .splineToSplineHeading(cycleHubPose, deg((isRed ? 1 : -1) * 90))
                        .build()
                )
                .start();

    }

    public Pose2d pos(double x, double y) {
        return isRed ? new Pose2d(+y, +x) : new Pose2d(+y, -x);
    }

    public Pose2d pos(double x, double y, double heading) {
        if (isRed)
            return new Pose2d(+y, +x, -Math.toRadians(heading));
        else
            return new Pose2d(+y, -x, Math.toRadians(heading));
    }

    public static double deg(double degrees) {
        return Math.toRadians(degrees);
    }
}
