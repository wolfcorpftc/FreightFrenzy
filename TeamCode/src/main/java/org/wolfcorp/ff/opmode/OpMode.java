package org.wolfcorp.ff.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class OpMode extends LinearOpMode {
    public OpMode() {
        Match.statusItem = null;
        Match.telemetry = telemetry;
        Match.isRed = this.getClass().getSimpleName().contains("Red");
    }
    public boolean modeNameContains(String searchTerm) {
        return this.getClass().getSimpleName().contains(searchTerm);
    }
}
