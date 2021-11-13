package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.wolfcorp.ff.opmode.AutonomousMode;

public class IO {
    private final Boolean RED = AutonomousMode.getInstance().RED;

    public DcMotorEx intake;
    public DcMotorEx redSlide;
    public DcMotorEx blueSlide;

    public CRServo redCarouselTurner;
    public CRServo blueCarouselTurner;

    public IO(HardwareMap hardwareMap) {
    }
}
