package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CVConfig {

    HardwareMap hwMap;

    public DcMotorImplEx backLeft;
    public DcMotorImplEx backRight;
    public DcMotorImplEx frontLeft;
    public DcMotorImplEx frontRight;
    public DcMotorImplEx motor;

    public HardwareMap Configure(HardwareMap ahwMap) {

        hwMap = ahwMap;

        backLeft = hwMap.get(DcMotorImplEx.class, "back_left_motor");
        backRight = hwMap.get(DcMotorImplEx.class, "back_right_motor");
        frontLeft = hwMap.get(DcMotorImplEx.class, "front_left_motor");
        frontRight = hwMap.get(DcMotorImplEx.class, "front_right_motor");
        motor = hwMap.get(DcMotorImplEx.class, "motor");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        return hwMap;
    }


}
