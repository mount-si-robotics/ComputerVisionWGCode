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

    private LLResult _latestResult;
    private LLResult _greenResult;
    private LLResult _purpleResult;
    public void findAndGrabBalls() {
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();
            chassis.limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Robot Yaw", orientation.getYaw());

            _latestResult = limelight.getLatestResult();

            if (_latestResult != null && _latestResult.getPipelineIndex() == targetArtifactPipeline.pipeline) {
                if (_latestResult.getPipelineIndex() == 2) _purpleResult = _latestResult;
                else _greenResult = _latestResult;
                targetArtifactPipeline = _latestResult.getPipelineIndex() == 2 ? REOYPLFD.Artifact.GREEN : REOYPLFD.Artifact.PURPLE;
                limelight.pipelineSwitch(targetArtifactPipeline.pipeline);
                _latestResult = null;
            }

            if (_greenResult == null || _purpleResult == null) {
                telemetry.addLine("No Results");
                telemetry.update();
                return;
            }

            if(gamepad1.aWasPressed()) _targetArtifact = _targetArtifact == REOYPLFD.Artifact.PURPLE ? REOYPLFD.Artifact.GREEN : REOYPLFD.Artifact.PURPLE;
            telemetry.addData("Target Artifact", _targetArtifact);

            if (_greenResult.isValid()) {
                List<LLResultTypes.ColorResult> greenResults = _greenResult.getColorResults();
                int greenAmount = greenResults.size();
                telemetry.addData("Green Amount", greenAmount);

                if(_targetArtifact == REOYPLFD.Artifact.GREEN) {
                    LLResultTypes.ColorResult target = _greenResult.getColorResults().get(_targetArtifactIndex);
                    moveRobot(target);
                }
            } else {
                telemetry.addData("Green Amount", "INVALID");
                if(_targetArtifact == REOYPLFD.Artifact.GREEN) {

                }
            }

            if (_purpleResult.isValid()) {
                List<LLResultTypes.ColorResult> purpleResults = _purpleResult.getColorResults();
                int purpleAmount = purpleResults.size();
                telemetry.addData("Purple Amount", purpleAmount);

                if(_targetArtifact == REOYPLFD.Artifact.PURPLE) {
                    LLResultTypes.ColorResult target = _purpleResult.getColorResults().get(_targetArtifactIndex);
                    moveRobot(target);
                }
            } else {
                telemetry.addData("Purple Amount", "INVALID");
                if (_targetArtifact == REOYPLFD.Artifact.PURPLE) {
                    frontLeft.setVelocity(0);
                    frontRight.setVelocity(0);
                    backRight.setVelocity(0);
                    backLeft.setVelocity(0);
                }
            }

            telemetry.update();
        }
    }

}
