package org.firstinspires.ftc.teamcode;

import android.media.SoundPool;
import android.speech.tts.TextToSpeech;

import com.qualcomm.ftccommon.SoundPlayer;
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
    public enum Artifact {
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
    EndOfYearChassis chassis = new EndOfYearChassis(this);
    @Override
    public void runOpMode() {
        chassis.init(new ChassisConfig());
        chassis.limelight.pipelineSwitch(2);
        chassis.limelight.start();
        chassis.waitForStart(this);
        while (opModeIsActive()) {
            System.out.println("Finding and grabbing artifacts");
            findAndGrabArtifacts();
            if (isStopRequested() || stopOpMode) return;
            System.out.println("Sorting artifacts");
            sortBall(_targetArtifact);
            if (isStopRequested()) return;
        }
    }

    public void findAndGrabArtifacts() {
        artifactCaptured = false;
        _purpleResult = null;
        _greenResult = null;
        chassis.waitForSwitch(targetArtifactPipeline.pipeline);
        timeSeenBall = System.currentTimeMillis();
        while (opModeIsActive()) {
            chassis.update();
            findAndGrabArtifactsLoop();
            if (isStopRequested() || stopOpMode) return;
            if (artifactCaptured) {
                System.out.println("Artifact captured!");
                break;
            }
        }
    }

    // ARTIFACT HUNTING CODE STARTS HERE
    private final double VELOCITY_CONSTANT = 750;
    private final double TARGET_AREA = 0.095;
    private final double TURNING_DAMPING = 25;
    private final double EDGE = 18;

    private LLResult _latestResult;
    private LLResult _greenResult;
    private LLResult _purpleResult;
    private Artifact targetArtifactPipeline = Artifact.PURPLE;
    private Artifact _targetArtifact = Artifact.GREEN;
    private boolean artifactCaptured = false;
    private boolean stopOpMode = false;

    private long timeSeenBall = System.currentTimeMillis();
    public void findAndGrabArtifactsLoop() {
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

        if (_greenResult == null && _purpleResult == null) {
            telemetry.addLine("No Results");
            telemetry.update();
            return;
        }

        LLResultTypes.ColorResult target = null;
        double targetArea = 0;
        double tdeg = Double.MAX_VALUE;
        Artifact targetBall = null;
        long currentTime = System.currentTimeMillis();
        if (_greenResult != null && _greenResult.isValid() && (_targetArtifact == Artifact.GREEN || currentTime - timeSeenBall > 500)) {
            List<LLResultTypes.ColorResult> greenResults = _greenResult.getColorResults();
            int greenAmount = greenResults.size();
            telemetry.addData("Green Amount", greenAmount);
            for (LLResultTypes.ColorResult result : greenResults) {
                if (currentTime - timeSeenBall < 500) {
                    double degrees = Math.abs(result.getTargetXDegrees());
                    if (tdeg > degrees) {
                        tdeg = degrees;
                        target = result;
                        targetBall = Artifact.GREEN;
                    }
                } else if (result.getTargetArea() > targetArea) {
                    targetArea = result.getTargetArea();
                    target = result;
                    targetBall = Artifact.GREEN;
                }
            }
        }

        if (_purpleResult != null && _purpleResult.isValid() && (_targetArtifact == Artifact.PURPLE || currentTime - timeSeenBall > 500)) {
            List<LLResultTypes.ColorResult> purpleResults = _purpleResult.getColorResults();
            int purpleAmount = purpleResults.size();
            telemetry.addData("Purple Amount", purpleAmount);
            for (LLResultTypes.ColorResult result : purpleResults) {
                if (currentTime - timeSeenBall < 500) {
                    double degrees = Math.abs(result.getTargetXDegrees());
                    if (tdeg > degrees) {
                        tdeg = degrees;
                        target = result;
                        targetBall = Artifact.PURPLE;
                    }
                } else if (result.getTargetArea() > targetArea) {
                    targetArea = result.getTargetArea();
                    target = result;
                    targetBall = Artifact.PURPLE;
                }
            }
        }

        if (target == null) {
            telemetry.addLine("Can't see target 3:");
            chassis.movePower(0, 0, 0);
        } else {
            seekArtifacts(target);
            _targetArtifact = targetBall;
            timeSeenBall = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - timeSeenBall > 5000) {
            stopOpMode = true;
            telemetry.speak("I'm done !");
            sleep(2000);
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
            chassis.movePower(0, 0, 0);
        }
    }

    private void captureArtifact() {
        chassis.movePower(0, 0.2, 0);
        sleep(500);
        chassis.movePower(0, 0, 0);
        chassis.closeGate();
        artifactCaptured = true;
    }

    private double getSeekingVelocity(double inverseArea, double rotationalPower) {
        return (inverseArea / Math.max((Math.abs(rotationalPower) * 20), 1) + rotationalPower) * VELOCITY_CONSTANT;
    }

    private double getRotationalPower(double targetAngle){
        double rotationPower = 0.0;
        double angleError = getNormalizedAngle(targetAngle);
        double _maxRotationalError = 1;
        if (Math.abs(angleError) > _maxRotationalError) {
            rotationPower = Math.max(-1.0, Math.min(-angleError / TURNING_DAMPING, 1.0));
        }

        return rotationPower;
    }

    private double getNormalizedAngle(double rawError) {
        return (rawError + 180) % 360 - 180;
    }

    // SORTING CODE IS HERE
    private static final int PURPLE_TAG = 1;
    private static final int GREEN_TAG = 2;
    public void sortBall(Artifact ballType) {
        chassis.waitForSwitch(5);
        System.out.println("Switched to pipeline 5");
        int tag = ballType == Artifact.GREEN ? GREEN_TAG : PURPLE_TAG;
        mainloop:
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            chassis.update();
            LLResult llresult = chassis.limelight.getLatestResult();
            if (llresult == null || !llresult.isValid()) {
                cantSeeBall();
                continue;
            }
            List<LLResultTypes.FiducialResult> results = llresult.getFiducialResults();
            for (LLResultTypes.FiducialResult result : results) {
                if (result.getFiducialId() == tag) {
                    if (goToTag(result)) break mainloop;
                    else continue mainloop;
                }
            }
            cantSeeBall();
        }
        System.out.println("Dropping the ball off");
        // drop ball off
        chassis.movePower(0, 0, 0);
        chassis.openGate();
        sleep(500);
        chassis.movePower(0, -0.25, 0);
        sleep(1250);
        chassis.movePower(0, 0, 0);

        System.out.println("Rotating back to 0");
        // Rotate back to 0 degrees
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            chassis.update();
            double degrees = chassis.yawDeg;
            double sign = Math.signum(degrees);
            chassis.movePower(0, 0, -sign * Math.min(Math.abs(degrees / 30), 1) * 0.5);
            telemetry.addData("rot", degrees);
            telemetry.update();
            if (Math.abs(degrees) < 5) {
                if (_targetArtifact == Artifact.GREEN) _targetArtifact = Artifact.PURPLE;
                else _targetArtifact = Artifact.GREEN;
                System.out.println("Found angle less than 5: " + degrees);
                System.out.println("Target artifact: " + _targetArtifact.name());
                chassis.movePower(0, 0, 0);
                return;
            }
        }

    }

    private long lastTimeSawBall = 0;
    private boolean goToTag(LLResultTypes.FiducialResult result) {
        lastTimeSawBall = System.currentTimeMillis();
        telemetry.addData("Going to", result.getFiducialId());
        telemetry.addData("txDeg", result.getTargetXDegrees());
        telemetry.addData("area", result.getTargetArea());
        double x = result.getTargetXPixels();
        telemetry.addData("x", x);
        telemetry.addData("pose", result.getTargetPoseRobotSpace());
        telemetry.update();
        double txDeg = result.getTargetXDegrees();
        if (Math.abs(txDeg) > 10) {
            chassis.movePower(0, 0, -txDeg / 60);
        } else {
            double area = result.getTargetArea();
            if (area == 0) area = 0.001;
            chassis.movePower(0,  Math.min((1/area) * 0.01, 1) * 0.25, -txDeg / 30);
            if (area > 0.09) {
                return true;
            }
        }
        return false;
    }

    private void cantSeeBall() {
        telemetry.addLine("Can't see ball");
        telemetry.update();
        if (System.currentTimeMillis() - lastTimeSawBall < 1000) {
            chassis.movePower(0, 0, 0);
        } else {
            chassis.movePower(0, 0, 0.25);
        }
    }
}
