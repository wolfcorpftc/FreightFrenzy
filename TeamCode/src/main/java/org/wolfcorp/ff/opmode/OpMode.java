package org.wolfcorp.ff.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OpMode extends LinearOpMode {
    /**
     * Based on the OpMode name, initialize {@link Match} members appropriately.
     */
    public OpMode() {
        Match.statusItem = null; // allows previous telemetry object to be GC'd (not sure if needed)
        Match.telemetry = telemetry; // don't do any additional config here, see Match#setupTelemetry
        Match.isRed = modeNameContains("Red");
    }

    /**
     * Tests whether the current OpMode name contains given substring.
     *
     * @param searchTerm the search term
     * @return whether the OpMode name contains the search term
     */
    public boolean modeNameContains(String searchTerm) {
        return this.getClass().getSimpleName().contains(searchTerm);
    }
}
