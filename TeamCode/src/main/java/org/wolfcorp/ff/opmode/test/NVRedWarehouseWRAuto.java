package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ff.opmode.AutonomousMode;

@Autonomous(name = "No Vision Red Warehouse Wall-runner", group = "test")
public class NVRedWarehouseWRAuto extends AutonomousMode {
    public NVRedWarehouseWRAuto() {
        super(false, true);
    }
}
