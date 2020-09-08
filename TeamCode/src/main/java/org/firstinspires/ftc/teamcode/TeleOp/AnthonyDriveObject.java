package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.*;

@TeleOp(name= "AnthonySketch")
public class AnthonyDriveObject extends LinearOpMode {

    SampleConfiguration config = new SampleConfiguration();
    ValueStorage vals = new ValueStorage();
    HardwareThread hardware = new HardwareThread(hardwareMap, vals, config);
    private boolean xPressed = false;

    Sequence BIGSPICE1 = new Sequence(() -> config.motor.setForTime(2000,3), null);
    Sequence BIGSPICE2 = new Sequence(() -> {
        config.servo.set(0);
        return null;
    }, BIGSPICE1);
    Sequence BIGSPICE3 = new Sequence(() -> config.backLeft.groupSetPosition(4000,0.5,100, config.backLeft,config.backRight,config.frontLeft,config.frontRight), BIGSPICE2);
    Sequence BIGSPICE4 = new Sequence(() -> config.motor.setForTime(-2000,3),BIGSPICE3);

    public void runOpMode() throws InterruptedException {
        try{

            waitForStart();
            ElapsedTime time = new ElapsedTime();
            hardware.start();

            while(!isStopRequested()) {
                vals.waitForCycle();
                getInput();
            }
        } catch(Exception e){
            System.out.println(e);
        } finally {
            hardware.Stop();
        }

    }

    private void getInput() {

        setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if(gamepad1.x && !xPressed){
            xPressed = true;
            Thread bigSpice = new Thread(BIGSPICE4);
            bigSpice.start();
        }
        else if (!gamepad1.x) xPressed = false;
    }

    private void setPower(double px, double py, double pa){
        double p1 = -px + py + pa;
        double p2 = px + py + pa;
        double p3 = -px + py - pa;
        double p4 = px + py - pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        config.backLeft.setPower(p1);
        config.frontLeft.setPower(p2);
        config.frontRight.setPower(p3);
        config.backRight.setPower(p4);
    }
}
