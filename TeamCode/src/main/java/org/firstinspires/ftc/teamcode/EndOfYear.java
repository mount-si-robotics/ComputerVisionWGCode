package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.atlas.ChassisConfig;

import java.util.List;

@TeleOp(name = "EOY Party")
public class EndOfYear extends LinearOpMode {
    EndOfYearChassis chassis = new EndOfYearChassis(this);
    @Override
    public void runOpMode() {
        chassis.init(new ChassisConfig());
        chassis.limelight.pipelineSwitch(2);
        chassis.limelight.start();
        chassis.waitForStart(this);
        while (opModeIsActive()) {
            chassis.tick();
            findAndGrabArtifacts();
        }
    }
    
    // ARTIFACT HUNTING CODE STARTS HERE
    private final double VELOCITY_CONSTANT = 750;
    private final double TARGET_AREA = 0.095;
    private final double TURNING_DAMPING = 25;
    private final double EDGE = 18;

    enum Artifact {
        PURPLE(2),
        GREEN(3);

        private final int pipeline;
        Artifact(int pipeline) {
            this.pipeline = pipeline;
        }

        public int getPipeline() {
            return pipeline;
        }
    }

    private LLResult _latestResult;
    private LLResult _greenResult;
    private LLResult _purpleResult;
    private Artifact targetArtifactPipeline = Artifact.PURPLE;
    private Artifact _targetArtifact = Artifact.PURPLE;
    private int _targetArtifactIndex = 0;
    private boolean artifactCaptured = false;

    public void findAndGrabArtifacts() {
        telemetry.addLine("Running");
        if (artifactCaptured) {
            return;
        }

        chassis.limelight.updateRobotOrientation(chassis.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        _latestResult = chassis.limelight.getLatestResult();
        if (_latestResult != null && _latestResult.getPipelineIndex() == targetArtifactPipeline.pipeline) {
            if (_latestResult.getPipelineIndex() == 2) _purpleResult = _latestResult;
            else _greenResult = _latestResult;
            targetArtifactPipeline = _latestResult.getPipelineIndex() == 2 ? Artifact.GREEN : Artifact.PURPLE;
            chassis.limelight.pipelineSwitch(targetArtifactPipeline.pipeline);
            _latestResult = null;
        }

        if (_greenResult == null || _purpleResult == null) {
            telemetry.addLine("No Results");
            telemetry.update();
            return;
        }

        if(gamepad1.aWasPressed()) _targetArtifact = _targetArtifact == Artifact.PURPLE ? Artifact.GREEN : Artifact.PURPLE;
        telemetry.addData("Target Artifact", _targetArtifact);

        if (_greenResult.isValid()) {
            List<LLResultTypes.ColorResult> greenResults = _greenResult.getColorResults();
            int greenAmount = greenResults.size();
            telemetry.addData("Green Amount", greenAmount);

            if(_targetArtifact == Artifact.GREEN) {
                LLResultTypes.ColorResult target = _greenResult.getColorResults().get(_targetArtifactIndex);
                seekArtifacts(target);
            }
        } else {
            telemetry.addData("Green Amount", "INVALID");
            if(_targetArtifact == Artifact.GREEN) {
                chassis.frontLeft.setVelocity(0);
                chassis.frontRight.setVelocity(0);
                chassis.backRight.setVelocity(0);
                chassis.backLeft.setVelocity(0);
            }
        }

        if (_purpleResult.isValid()) {
            List<LLResultTypes.ColorResult> purpleResults = _purpleResult.getColorResults();
            int purpleAmount = purpleResults.size();
            telemetry.addData("Purple Amount", purpleAmount);

            if(_targetArtifact == Artifact.PURPLE) {
                LLResultTypes.ColorResult target = _purpleResult.getColorResults().get(_targetArtifactIndex);
                seekArtifacts(target);
            }
        } else {
            telemetry.addData("Purple Amount", "INVALID");
            if (_targetArtifact == Artifact.PURPLE) {
                chassis.frontLeft.setVelocity(0);
                chassis.frontRight.setVelocity(0);
                chassis.backRight.setVelocity(0);
                chassis.backLeft.setVelocity(0);
            }
        }

        telemetry.update();
    }

    private void seekArtifacts(LLResultTypes.ColorResult target) {
        double targetOffset = target.getTargetXDegrees();
        double rotationalPower = getRotationalPower(targetOffset);
        double inverseArea = 2 / ((target.getTargetArea()) * 100);

        telemetry.addData("Target Offset", targetOffset);

        if (target.getTargetArea() >= TARGET_AREA || Math.abs(targetOffset) >= EDGE) inverseArea = 0;

        if (inverseArea == 0 && rotationalPower == 0) {
            captureArtifact();
            return;
        }

        telemetry.addData("Target Area", target.getTargetArea());
        telemetry.addData("Inverse Area", inverseArea);

        telemetry.addData("Rotational Power", rotationalPower);
        telemetry.addData("Total Power (FL)", getSeekingVelocity(inverseArea, rotationalPower));
        if (!gamepad1.b) {
            chassis.frontLeft.setVelocity(getSeekingVelocity(inverseArea, rotationalPower));
            chassis.frontRight.setVelocity(getSeekingVelocity(inverseArea, -rotationalPower));
            chassis.backLeft.setVelocity(getSeekingVelocity(inverseArea, rotationalPower));
            chassis.backRight.setVelocity(getSeekingVelocity(inverseArea, -rotationalPower));
        } else {
            chassis.frontLeft.setVelocity(0);
            chassis.frontRight.setVelocity(0);
            chassis.backLeft.setVelocity(0);
            chassis.backRight.setVelocity(0);
        }
    }

    private void captureArtifact() {
        chassis.frontLeft.setVelocity(0);
        chassis.frontRight.setVelocity(0);
        chassis.backRight.setVelocity(0);
        chassis.backLeft.setVelocity(0);
        chassis.closeGate();
        artifactCaptured = true;
    }

    private double getSeekingVelocity(double inverseArea, double rotationalPower) {
        return (inverseArea / Math.max((Math.abs(rotationalPower) * 20), 1) + rotationalPower) * VELOCITY_CONSTANT;
    }

    private final double _maxRotationalError = 1;

    private double getRotationalPower(double targetAngle){
        double rotationPower = 0.0;
        double angleError = getNormalizedAngle(targetAngle);
        if (Math.abs(angleError) > _maxRotationalError) {
            rotationPower = Math.max(-1.0, Math.min(-angleError / TURNING_DAMPING, 1.0));
        }

        return rotationPower;
    }

    private double getNormalizedAngle(double rawError) {
        return (rawError + 180) % 360 - 180;
    }
}
