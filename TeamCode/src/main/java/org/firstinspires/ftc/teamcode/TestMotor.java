package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TestMotor")
public class TestMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DcMotorEx testMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "test");
        while (opModeIsActive()) {
            testMotor.setPower(gamepad1.left_stick_x);
        }
    }
}
