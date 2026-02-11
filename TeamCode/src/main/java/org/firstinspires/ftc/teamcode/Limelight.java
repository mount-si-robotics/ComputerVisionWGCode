package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "Limelight")
public class Limelight extends OpMode {
    IMU imu;
    Limelight3A limelight;
    double tx;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(2);

        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    private LLResult _latestResult;
    private LLResult _greenResult;
    private LLResult _purpleResult;
    private int targetPipeline;

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        _latestResult = limelight.getLatestResult();
        if (_latestResult != null && _latestResult.getPipelineIndex() == targetPipeline) {
            if (_latestResult.getPipelineIndex() == 2) _purpleResult = _latestResult;
            else _greenResult = _latestResult;
            targetPipeline = _latestResult.getPipelineIndex() == 2 ? 3 : 2;
            limelight.pipelineSwitch(targetPipeline);
            _latestResult = null;
        }

        if (_greenResult == null || _purpleResult == null) { // this is always true for some reason
            telemetry.addLine("No Results");
            telemetry.update();
            return;
        }

        if (_greenResult.isValid()) {
            List<LLResultTypes.ColorResult> greenResults = _greenResult.getColorResults();
            int greenAmount = greenResults.size();
            telemetry.addData("Green Amount", greenAmount);
        } else telemetry.addData("Green Amount", "INVALID");

        if (_purpleResult.isValid()) {
            List<LLResultTypes.ColorResult> purpleResults = _purpleResult.getColorResults();
            int purpleAmount = purpleResults.size();
            telemetry.addData("Purple Amount", purpleAmount);
        } else telemetry.addData("Purple Amount", "INVALID");

//            tx = result.getTx();
//                telemetry.addData("tx", result.getTx());
//                telemetry.addData("ty", result.getTy());
//                telemetry.addData("Purple Amount", purpleAmount);

        telemetry.update();
    }
}
