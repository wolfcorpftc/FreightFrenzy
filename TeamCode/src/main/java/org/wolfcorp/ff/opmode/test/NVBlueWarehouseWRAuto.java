package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ff.opmode.AutonomousMode;

@Autonomous(name = "No Vision Blue Warehouse Wall-runner", group = "test")
public class NVBlueWarehouseWRAuto extends AutonomousMode {
    public NVBlueWarehouseWRAuto() {
        super(false, true);
    }
}
