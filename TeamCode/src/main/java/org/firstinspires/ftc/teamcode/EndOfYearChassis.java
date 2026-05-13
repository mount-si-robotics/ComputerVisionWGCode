package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.atlas.AtlasChassis;
import org.firstinspires.ftc.teamcode.atlas.ChassisConfig;

import java.util.function.Consumer;

public class EndOfYearChassis extends AtlasChassis {
    public static final int GREEN_PIPELINE = 2;
    public static final int PURPLE_PIPELINE = 3;
    Limelight3A limelight;
    Servo servo1, servo2;
    public EndOfYearChassis(OpMode opMode) {
        super(opMode);
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        ChassisConfig config = new ChassisConfig();
        config.imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        config.frontLeftName = "frontLeft";
        config.frontRightName = "frontRight";
        config.backLeftName = "rearLeft";
        config.backRightName = "rearRight";

        servo1 = opMode.hardwareMap.get(Servo.class, "servoLeft");
        servo2 = opMode.hardwareMap.get(Servo.class, "servoRight");
        init(config);
    }

    @Override
    public void tick() {

    }

    public void waitForSwitch(int pipeline) {
        if (opMode instanceof LinearOpMode) {
            LinearOpMode linear = (LinearOpMode) opMode;
            limelight.pipelineSwitch(pipeline);
            while (true) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.getPipelineIndex() == pipeline) return;
                linear.sleep(10);
            }
        } else {
            throw new RuntimeException("You can only wait in a linear opmode");
        }
    }

    public void getLimelightResult(Consumer<LLResult> callback) {
        LLResult result = limelight.getLatestResult();
        if (result != null) callback.accept(result);
    }

    public void openGate() {
        servo2.setPosition(1);
        servo1.setPosition(0);
    }

    public void closeGate() {
        servo2.setPosition(0.4);
        servo1.setPosition(0.45);
    }

    @Override
    public void initLoop(OpMode opMode) {

    }
}
