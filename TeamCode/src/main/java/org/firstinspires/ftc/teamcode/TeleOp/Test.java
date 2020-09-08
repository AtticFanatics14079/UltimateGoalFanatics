package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.CVConfig;

@TeleOp(name= "TEST")
public class Test extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        CVConfig config = new CVConfig();
        config.Configure(hardwareMap);
        waitForStart();
        config.backRight.setPower(0.2);
        while(!isStopRequested()) {
            telemetry.addLine(config.backRight.getVelocity() + " and " + config.backRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
