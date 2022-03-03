package org.wolfcorp.ff.sim;

public class MeepMeepTest extends AutoTest {
    public static void main(String[] args) {
        // Enable this flag to improve performance when necessary
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeepTest test = new MeepMeepTest();
        test.start();
        test.original();
    }
}