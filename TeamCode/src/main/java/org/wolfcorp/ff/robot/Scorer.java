package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Scorer {
    public DcMotorEx shovel;
    public static final int SHOVEL_MOE = 10; // margin of error
    public static final int SHOVEL_MOA = 3; // margin of acceptance
    public static final int SHOVEL_DRIFT_TIME_DELAY = 1000;
    private int shovelTargetPos = 0;

    private ElapsedTime shovelDriftDelay = new ElapsedTime();

    public Scorer(HardwareMap hwMap) {
        shovel = hwMap.get(DcMotorEx.class, "shovel");

        shovel.setPower(0);

        shovel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shovel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setShovelTargetPos(int pos) {
        shovelTargetPos = pos;
        shovelDriftDelay.reset();
    }

    public int getShovelTargetPos() {
        return shovelTargetPos;
    }

    public void checkShovelPosDrift() {
        if (Math.abs(shovel.getCurrentPosition() - shovelTargetPos) > SHOVEL_MOE &&
                shovelDriftDelay.milliseconds() > SHOVEL_DRIFT_TIME_DELAY) {
            shovel.setTargetPosition(shovelTargetPos);
            shovel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shovel.setPower(0.2);
            while (Math.abs(shovel.getCurrentPosition() - shovelTargetPos) > SHOVEL_MOA) {
            }
            shovel.setPower(0);
            shovel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
