package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "Demo")
public class Demo extends LinearOpMode {
    private static final double VELOCITY_CONSTANT = 750;
    private static final double TARGET_AREA = 0.55;
    private static final double ATTACK_THRESHOLD = 0.5;
    private static final double TURNING_DAMPING = 25;
    private static final double EDGE = 18;

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

    IMU imu;
    Limelight3A limelight;

    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    Servo attackServo;

    private LLResult _purpleResult;
    private Artifact _targetArtifact = Artifact.PURPLE;
    private int _targetArtifactIndex = 0;
    int state = 0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft  = hardwareMap.get(DcMotorEx.class,  "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class,   "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        attackServo = hardwareMap.get(Servo.class, "attackServo");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        attackServo.scaleRange(0, 1);
        attackServo.setPosition(0.1);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(2);

        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Robot Yaw", orientation.getYaw());

            _purpleResult = limelight.getLatestResult();

            if (_purpleResult == null) {
                telemetry.addLine("No Results");
                telemetry.update();
                return;
            }

            if (_purpleResult.isValid()) {
                List<LLResultTypes.ColorResult> purpleResults = _purpleResult.getColorResults();
                int purpleAmount = purpleResults.size();
                telemetry.addData("Purple Amount", purpleAmount);

                if(_targetArtifact == Artifact.PURPLE) {
                    LLResultTypes.ColorResult target = _purpleResult.getColorResults().get(_targetArtifactIndex);
                    // Move state
                    if (state == 0) {
                        moveRobot(target);
                    }
//                    // Attack state
//                    else if (state == 1) {
//
//                        state = 2;
//                    }
                }
            } else {
                telemetry.addData("Purple Amount", "INVALID");
                if(_targetArtifact == Artifact.PURPLE) {
                    frontLeft.setVelocity(0);
                    frontRight.setVelocity(0);
                    backRight.setVelocity(0);
                    backLeft.setVelocity(0);
                }
            }

            telemetry.update();
        }
    }

    long attackStartTime = System.currentTimeMillis();
    private void moveRobot(LLResultTypes.ColorResult target) {
        double targetOffset = target.getTargetXDegrees();
        double rotationalPower = getRotationalPower(targetOffset);
        double inverseArea = 2 / ((target.getTargetArea()) * 100);

        telemetry.addData("Target Offset", targetOffset);

        if (target.getTargetArea() >= TARGET_AREA || Math.abs(targetOffset) >= EDGE) inverseArea = 0;

        if (inverseArea == 0 && rotationalPower == 0) {
            frontLeft.setVelocity(0);
            frontRight.setVelocity(0);
            backLeft.setVelocity(0);
            backRight.setVelocity(0);
        }

        telemetry.addData("Target Area", target.getTargetArea());
        telemetry.addData("Inverse Area", inverseArea);
        telemetry.addData("State", state);
        telemetry.addData("Rotational Power", rotationalPower);
        frontLeft.setVelocity(getPower(inverseArea, rotationalPower));
        frontRight.setVelocity(getPower(inverseArea, -rotationalPower));
        backLeft.setVelocity(getPower(inverseArea, rotationalPower));
        backRight.setVelocity(getPower(inverseArea, -rotationalPower));
        if (target.getTargetArea() >= ATTACK_THRESHOLD) {
            frontLeft.setVelocity(0);
            frontRight.setVelocity(0);
            backLeft.setVelocity(0);
            backRight.setVelocity(0);
            attackStartTime = System.currentTimeMillis();
            for (int i = 0; i < 6; i++) {
                attackServo.setPosition(0.65);
                sleep(300);
                attackServo.setPosition(0.765);
                sleep(225);
            }
            attackServo.setPosition(0.1);
            state = 1;
        }
    }
    private double getPower(double inverseArea, double rotationalPower) {
        return (inverseArea / Math.max((Math.abs(rotationalPower) * 20), 1) + rotationalPower) * VELOCITY_CONSTANT;
    }

    private final double _maxRotationalError = 1;

    private double getRotationalPower(double targetAngle){
        double rotationPower = 0.0;
        double angleError = getNormalizedAngle(targetAngle);
        if (Math.abs(angleError) > _maxRotationalError) {
            rotationPower = Math.max(-1.0, Math.min(-angleError / TURNING_DAMPING, 1.0));
        }

        return -rotationPower;
    }

    private double getNormalizedAngle(double rawError) {
        return (rawError + 180) % 360 - 180;
    }
}
