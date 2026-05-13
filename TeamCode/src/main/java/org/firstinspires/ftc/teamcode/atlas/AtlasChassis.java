package org.firstinspires.ftc.teamcode.atlas;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.atlas.utils.Vector2;

public abstract class AtlasChassis {
    public DcMotorEx backLeft, backRight, frontLeft, frontRight;
    public IMU imu;
    public AtlasPose pose;

    public double yawRads = 0.0;
    public double yawDeg = 0.0;
    public double yawOffsetFromOrigin = 0.0;
    public double robotYawOffset = 0.0;

    public int backLeftTicks = 0;
    public int backRightTicks = 0;
    public int frontLeftTicks = 0;
    public int frontRightTicks = 0;

    public double metersPerTick = 0.00048;
    public double fieldRelativeOffset = 0.0;

    private long lastUpdateTime = System.currentTimeMillis();

    public static final double RAD_TO_DEG = 180.0 / Math.PI;
    public static final double DEG_TO_RAD = Math.PI / 180.0;

    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    public OpMode opMode;
    public AtlasChassis(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init(ChassisConfig config) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        zeroPowerBehavior = config.zeroPowerBehavior;
        backLeft = getDcMotorEx(hardwareMap, config.backLeftName, config.backLeftIsReversed);
        backRight = getDcMotorEx(hardwareMap, config.backRightName, config.backRightIsReversed);
        frontLeft = getDcMotorEx(hardwareMap, config.frontLeftName, config.frontLeftIsReversed);
        frontRight = getDcMotorEx(hardwareMap, config.frontRightName, config.frontRightIsReversed);
        // Make sure imu exists in hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        if (imu == null) throw new NullPointerException("IMU is null????");
        pose = new AtlasPose(metersPerTick);
        if (config.imuParameters == null) throw new NullPointerException("IMU Paramters is null");
        imu.initialize(config.imuParameters);
        imu.resetYaw();
    }

    public DcMotorEx getDcMotorEx(HardwareMap hardwareMap, String name, boolean reversed) {
        DcMotorEx motor = (DcMotorEx) hardwareMap.get(DcMotor.class, name);
        if (reversed) motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        return motor;
    }
    public DcMotorEx getDcMotorEx(HardwareMap hardwareMap, String name) {
        return getDcMotorEx(hardwareMap, name, false);
    }

    public void moveFieldRelative(double x, double y, double rx) {
        double yaw = yawRads + fieldRelativeOffset;
        double rotatedX = -x * Math.cos(yaw) - y * Math.sin(yaw);
        double rotatedY = -x * Math.sin(yaw) + y * Math.cos(yaw);
        movePower(rotatedX, rotatedY, rx);
    }

    public void movePower(double x, double y, double r) {
        double denominator = max(abs(y) + abs(x) + abs(r), 1.0);
        frontLeft.setPower((y + x + r) / denominator);
        backLeft.setPower((y - x + r) / denominator);
        frontRight.setPower((y - x - r) / denominator);
        backRight.setPower((y + x - r) / denominator);
    }


    public void moveFieldRelativeDegraded(double x, double y, double rx) {
        double yaw = yawRads + fieldRelativeOffset;
        double rotatedX = x * Math.cos(-yaw) - y * Math.sin(-yaw);
        double rotatedY = x * Math.sin(-yaw) + y * Math.cos(-yaw);
        movePower(rotatedX, rotatedY, rx);
    }


    public void runToPosition(int x, int y) {
        Vector2 vect = new Vector2(x, y);
        vect.rotate(robotYawOffset);
        x = (int) vect.x;
        y = (int) vect.y;
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setTargetPosition(y + x);
        frontLeft.setTargetPosition(y + x);
        backLeft.setTargetPosition(y - x);
        frontRight.setTargetPosition(y - x);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Vector2 normalized = vect.normalized();
        movePower(normalized.x * 0.5, normalized.y * 0.5, 0);
    }

    public double update() {
        return update(null);
    }
    public double update(Telemetry telemetry) {
        long currentTime = System.currentTimeMillis();
        double deltaTimeMS = currentTime - lastUpdateTime;
        if (deltaTimeMS == 0) deltaTimeMS = 0.1;
        double deltaTime = deltaTimeMS * 0.001;
        lastUpdateTime = currentTime;

        Orientation orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        );

        yawRads = orientation.firstAngle + robotYawOffset * DEG_TO_RAD;
        yawDeg = orientation.firstAngle * RAD_TO_DEG + robotYawOffset;

        if (Double.isNaN(yawRads) || Double.isNaN(yawDeg) || Double.isInfinite(yawRads) || Double.isInfinite(yawDeg)) {
            throw new RuntimeException("i fucking hate the imu so much");
        }

        int[] positions = new int[]{
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition(),
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition()
        };

        int deltaBackLeft = positions[0] - backLeftTicks;
        int deltaBackRight = positions[1] - backRightTicks;
        int deltaFrontLeft = positions[2] - frontLeftTicks;
        int deltaFrontRight = positions[3] - frontRightTicks;

        backLeftTicks = positions[0];
        backRightTicks = positions[1];
        frontLeftTicks = positions[2];
        frontRightTicks = positions[3];

        double yaw = ((yawOffsetFromOrigin - 90) * DEG_TO_RAD) + yawRads;
        pose.updateEncoders(deltaFrontLeft, deltaFrontRight, deltaBackLeft, deltaBackRight, yaw);
        if (Double.isNaN(pose.x) || Double.isNaN(pose.y) || Double.isInfinite(pose.x) || Double.isInfinite(pose.y)) {
            throw new RuntimeException("i fucking hate the imu so much");
        }
        tick();
        if (telemetry != null) {
            telemetry.addLine("Chassis debug data:");
            telemetry.addLine("position (" + pose.x + ", " + pose.y + ")");
            telemetry.addLine("yaw " + yawDeg);
        }
        return deltaTime;
    }

    public abstract void tick();

    public abstract void initLoop(OpMode opMode);

    public void waitForStart(LinearOpMode opMode) {
        while (opMode.opModeInInit()) {
            initLoop(opMode);
        }
    }
}
