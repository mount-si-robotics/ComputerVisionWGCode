package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class EndOfYear extends LinearOpMode {

    EndOfYearChassis chassis = new EndOfYearChassis(this);
    @Override
    public void runOpMode() throws InterruptedException {
        chassis.waitForStart(this);
        while (opModeIsActive()) {
            chassis.tick();

        }
    }

    public void findAndGrabBalls() {
        while (opModeIsActive()) {
            chassis.tick();
        }
    }

}
