package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.entity.BotEntity;
import com.noahbres.meepmeep.core.entity.Entity;

import org.jetbrains.annotations.NotNull;

import java.awt.Graphics2D;
import java.util.function.Supplier;

public class MeepMeepTesting {

    protected Pose2d initialPose;
    protected Pose2d carouselPose;
    protected Pose2d elementLeftPose;
    protected Pose2d elementMidPose;
    protected Pose2d elementRightPose;
    protected Pose2d hubPose;
    protected Pose2d preWhPose;
    protected Pose2d whPose;
    protected Pose2d parkPose;

    double length = 12.25;
    double width = 12.25;

    boolean invert = true;
    boolean isWallRunner = true;
    boolean isNearCarousel = true;

    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
        // System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels

        MeepMeepTesting bot = new MeepMeepTesting();

        bot.setUpPos();

        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setBotDimensions(bot.width,bot.length)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(bot.initialPose)
                                .lineToLinearHeading(bot.carouselPose)
                                .waitSeconds(1)
                                .lineToLinearHeading(bot.elementLeftPose)
                                .lineToLinearHeading(bot.elementLeftPose.plus(bot.pos(13,0)))
                                .lineToLinearHeading(bot.hubPose)
                                .waitSeconds(1)
                                .lineToLinearHeading(bot.preWhPose)
                                .lineTo(bot.whPose.vec())
                                .lineTo(bot.preWhPose.vec())
                                .lineToLinearHeading(bot.hubPose)
                                .waitSeconds(1)
                                .lineToLinearHeading(bot.preWhPose)
                                .lineTo(bot.whPose.vec())
                                .lineTo(bot.parkPose.vec())
                                .build()
                )
                .start();
    }

    public void setUpPos() {

        /*
        initialPose = pos(-72 + length / 2, 12, -90);
        carouselPose = pos(-60, -60, 180);
        elementLeftPose = pos(-72 + length / 2, 20.4, -90);
        elementMidPose = pos(-72 + length / 2, 12, -90);
        elementRightPose = pos(-72 + length / 2, 3.6, -90);
        hubPose = pos(-72 + length / 2, -12, -90);
        preWhPose = pos(-72 + width / 2, 24 - length / 2);
        whPose = pos(-72 + width / 2, 36);
        */

        initialPose = pos(-72 + length / 2, 12, 180);
        carouselPose = pos(-50, -60, 180);
        elementLeftPose = pos(-60 + length / 2, 20.4, 90);
        elementMidPose = pos(-60 + length / 2, 12, 90);
        elementRightPose = pos(-60 + length / 2, 3.6, 90);
        hubPose = pos(-72 + length / 2, -12, 180);
        preWhPose = pos(-72 + width / 2, 24 - length / 2, 180);
        whPose = pos(-72 + width / 2, 46, 180);

        if (isWallRunner)
            parkPose = pos(-72 + width / 2, 42, 180);
        else
            parkPose = pos(-36, 46, 180);

        if (isNearCarousel){
            initialPose = initialPose.plus(pos(0, -48));
            elementLeftPose = elementLeftPose.plus(pos(0, -48));
            elementMidPose = elementMidPose.plus(pos(0, -48));
            elementRightPose = elementRightPose.plus(pos(0, -48));
        }
    }

    public void setUpPaths(){
        // TODO: set up
    }

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
}