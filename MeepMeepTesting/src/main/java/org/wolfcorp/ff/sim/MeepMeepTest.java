package org.wolfcorp.ff.sim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTest extends AutoTest {
    public static void main(String[] args) {
        // Enable this flag to improve performance when necessary
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeepTest test = new MeepMeepTest();
        test.start();
    }
}