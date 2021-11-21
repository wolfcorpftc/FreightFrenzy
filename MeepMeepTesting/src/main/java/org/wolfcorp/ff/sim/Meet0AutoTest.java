package org.wolfcorp.ff.sim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class Meet0AutoTest extends AutoTest {
    protected Pose2d preWhPose;

    public Meet0AutoTest() {
        isRed = true;
        isWallRunner = true;
        isNearCarousel = true;
    }

    @Override
    public void initPoses() {
        super.initPoses();
        hubPose = pos(-48 + length / 2, -12, 90);
        preWhPose = pos(-72 + width / 2, 12, 180);
        parkPose = whPose;
    }

    @Override
    public void start() {
        initPoses();

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
                .followTrajectorySequence(drive -> {
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(initialPose);

                    // *** Carousel ***
                    if (isNearCarousel) {
                        builder.lineToLinearHeading(preCarouselPose)
                                .lineTo(carouselPose.vec())
                                .waitSeconds(1);
                    }

                    // *** Score preloaded cube ***
                    builder.lineToLinearHeading(hubPose)
                            .waitSeconds(1);

                    // *** Park ***
                    builder.splineToSplineHeading(preWhPose, deg(0))
                            .lineTo(parkPose.vec())
                    ;
                    return builder.build();
                })
                .start();
    }
}
