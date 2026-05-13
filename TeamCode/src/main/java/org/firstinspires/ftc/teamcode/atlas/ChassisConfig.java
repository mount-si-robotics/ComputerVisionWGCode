package org.firstinspires.ftc.teamcode.atlas;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

public class ChassisConfig {
    public ChassisConfig() {}

    public String backLeftName;
    public boolean backLeftIsReversed = false;
    public String frontLeftName;
    public boolean frontLeftIsReversed = false;
    public String backRightName;
    public boolean backRightIsReversed = false;
    public String frontRightName;
    public boolean frontRightIsReversed = false;
    public DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    public IMU.Parameters imuParameters = null;
}