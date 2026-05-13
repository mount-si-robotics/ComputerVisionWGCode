package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class EndOfYear extends LinearOpMode {

    EndOfYearChassis chassis = new EndOfYearChassis(this);
    @Override
    public void runOpMode() throws InterruptedException {
        chassis.waitForStart(this);
        while (opModeIsActive()) {
            chassis.tick();


        }
    }
}
