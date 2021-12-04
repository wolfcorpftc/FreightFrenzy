package org.wolfcorp.ff.sim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

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
    protected Pose2d whPose; // load freight from warehouse
    protected Pose2d parkPose; // where the robot parks

    protected double length = 15;
    protected double width = 13;

    protected boolean isRed = true;
    protected boolean isWallRunner = true;
    protected boolean isNearCarousel = false;

    public void initPoses() {
        initialPose = pos(-72 + width / 2, 12, 180);
        fakeInitialPose = initialPose.minus(pos(3, 0));

        carouselPose = pos(-50.5, -72 + width / 2, 90);
        preCarouselPose = carouselPose.plus(pos(1.5, 0));
        fakePreCarouselPose = preCarouselPose.minus(pos(0, 3));
        postCarouselPose = carouselPose.plus(pos(0, 5));

        elementLeftPose = pos(-72 + length / 2, 20.4, 180);
        elementMidPose = elementLeftPose.minus(pos(0, 8.4));
        elementRightPose = elementMidPose.minus(pos(0, 8.4));

        hubPose = pos(-72 + width / 2, -12, 180);
        whPose = pos(-72 + width / 2, 42, 180);

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

    public void start() {
        initPoses();

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
                        .lineToLinearHeading(preCarouselPose)
                        .lineTo(carouselPose.vec())
                        .waitSeconds(1)
                        .lineTo(postCarouselPose.vec())
                        .lineToLinearHeading(initialPose)
                        .lineTo(elementLeftPose.vec())
                        .lineTo(hubPose.vec())
                        .waitSeconds(1)
                        .lineTo(whPose.vec())
                        .waitSeconds(1)
                        .lineTo(hubPose.vec())
                        .waitSeconds(1)
                        .lineTo(whPose.vec())
                        .lineTo(parkPose.vec())
                        .build()
                )
                .start();
    }

    public Pose2d pos(double x, double y) {
        return isRed ? new Pose2d(+y, +x) : new Pose2d(+y, -x);
    }

    // Rotate the coordinate plane 90 degrees clockwise (positive y-axis points at the shared hub)
    // Basically converts a point from Cartesian to Roadrunner
    // The positive y-axis represents a heading of 0 degree
    public Pose2d pos(double x, double y, double heading) {
        if (isRed)
            return new Pose2d(+y, +x, Math.toRadians(180+heading));
        else
            return new Pose2d(+y, -x, Math.toRadians(heading));
    }

    public static double deg(double degrees) {
        return Math.toRadians(degrees);
    }
}