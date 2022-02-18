package org.wolfcorp.ff.sim;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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
    protected Pose2d preHubPose;
    protected Pose2d preWhPose;

    protected double length = 15;
    protected double width = 13;

    protected boolean isRed = true;
    protected boolean isWallRunner = true;
    protected boolean isNearCarousel = false;

    protected MeepMeep mm;

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

        whPose = pos(-72 + width / 2, 42, 180);
        hubPose = pos(-48.5 + width / 2, -12, 90);
        preHubPose = pos(-48, -12, 0);
        preWhPose = pos(-72 + width / 2, 12, 0);

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

        mm = new MeepMeep(600);
        /*RoadRunnerBotEntity bot1 = new DefaultBotBuilder(mm)
                .setColorScheme(new ColorSchemeRedDark())
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(width,length)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(hubPose)
                        .lineToSplineHeading(preHubPose.minus(pos(10,-12)))
                        //.splineToSplineHeading(preHubPose.minus(pos(15,-15)),0)
                        .splineToLinearHeading(preWhPose,0)
                        .lineTo(whPose.vec())
                        .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(bot1)
                .start();
         */
        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f);

        RoadRunnerBotEntity allianceBot = otherBot(4);
        mm.addEntity(allianceBot);

        // Create array of RoadRunnerBotEntity objects
        RoadRunnerBotEntity[] bots = new RoadRunnerBotEntity[30];
        for(double i=0; i<30; i++){
            bots[(int)i] = newBot(i/5);
            mm.addEntity(bots[(int)i]);
            int finalI = (int)i;
            new Thread(() -> {
                try {
                    sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                while(!bots[finalI].getTrajectoryPaused()){
                    Pose2d p = bots[finalI].getPose();
                    Pose2d p2 = allianceBot.getPose();
                    if (Math.abs(p.getX()-p2.getX()) < 15 && Math.abs(p.getY()-p2.getY()) < 15){
                        bots[finalI].pause();
                        bots[finalI].setListenToSwitchThemeRequest(true);
                        bots[finalI].switchScheme(new ColorSchemeBlueDark());
                    }
                    try {
                        sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }).start();

        }
        mm.start();
/*
        while(true) {
            // delete all bots
            for (int i = 0; i < bots.length; i += 2) {
                Pose2d p = bots[i].getPose();
                if (p.getX() > 20) {
                    bots[i].pause();
                    bots[i].setListenToSwitchThemeRequest(true);
                    bots[i].switchScheme(new ColorSchemeBlueDark());
                }
            }
        }

 */
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

    public RoadRunnerBotEntity newBot(double delay){
        return new DefaultBotBuilder(mm)
                .setColorScheme(new ColorSchemeRedDark())
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(width,length)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(hubPose)
                        .waitSeconds(delay)
                        .lineToSplineHeading(preHubPose.minus(pos(10,-12)))
                        //.splineToSplineHeading(preHubPose.minus(pos(15,-15)),0)
                        .splineToLinearHeading(preWhPose,0)
                        .lineTo(whPose.vec())
                        .build()
                );
    }
    public RoadRunnerBotEntity otherBot(double delay){
        return new DefaultBotBuilder(mm)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(width,length)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(pos(0,20))
                        .waitSeconds(delay)
                        .lineTo(pos(-100,20).vec())
                        .build()
                );
    }
}
