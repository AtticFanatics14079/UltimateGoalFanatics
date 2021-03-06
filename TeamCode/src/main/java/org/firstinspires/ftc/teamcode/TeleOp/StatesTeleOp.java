package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareConfigs.StatesConfigure;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "States TeleOp", group = "Sensor")
public class StatesTeleOp extends LinearOpMode {

    private StatesTeleOpMecanum Drive = new StatesTeleOpMecanum();

    @Override
    public void runOpMode() {
        Drive.Configure(hardwareMap);
        Drive.setBulkCachingManual();
        //Running scissors down to keep starting position consistent
        ElapsedTime time = new ElapsedTime();
        telemetry.setMsTransmissionInterval(40);
        double prevTime = time.seconds();
        while(time.seconds() - prevTime < 1) {
            Drive.ScissorLeft.setPower(-0.2);
            Drive.ScissorRight.setPower(-0.2);
        }
        while(time.seconds() - prevTime < 2) {
            Drive.ScissorLeft.setPower(0);
            Drive.ScissorRight.setPower(0);
        }
        Drive.ScissorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive.ScissorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive.ScissorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive.ScissorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive.ExtendGripper.setTargetPosition(50);
        Drive.ExtendGripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!isStopRequested() && Drive.ExtendGripper.isBusy() && !isStarted()){}
        Drive.ExtendGripper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive.ExtendGripper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*while(!isStopRequested()) {
            telemetry.addData("Pressed: ", Drive.BlockSense.isPressed());
            telemetry.update();
        }
         */
        waitForStart();
        Drive.ingesterStates = StatesConfigure.Ingester.STOPPEDIN;
        //Drive.ingester.setPower(0.65);
        Drive.Capstone.setPosition(Drive.CAPSTONE_CLOSED);
        Drive.startTime();
        while(!isStopRequested()) {
            Drive.Move(hardwareMap, gamepad1, gamepad2);
            telemetry.addData("Block Level: ", Drive.level - 2);
            telemetry.addData("NextStack: ", Drive.nextStack);
            telemetry.addData("Macro State: ", Drive.Macro);
            telemetry.addData("Capping Mode: ", Drive.stack);
            telemetry.addData("Robot Role: ", Drive.status);
            telemetry.addData("Extend Pos: ", Drive.ExtendGripper.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addData("Ingester Current: ", Drive.ingester.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("ScissorLeft Current: ", Drive.ScissorLeft.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("ScissorRight Current: ", Drive.ScissorRight.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("ExtendGripper Current: ", Drive.ExtendGripper.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }
}