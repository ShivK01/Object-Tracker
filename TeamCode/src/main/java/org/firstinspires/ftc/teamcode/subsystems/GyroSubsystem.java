package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Class which allows use of the gyro sensor
public class GyroSubsystem {
    private BNO055IMU imu;

    public GyroSubsystem(HardwareMap hardwareMap) {
        //Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //Method which returns gyro reading in radians
    public float getAngleRad() {
        return imu.getAngularOrientation().firstAngle;
    }

    //Method which returns gyro reading in degrees
    public double getAngleDeg() {
        return imu.getAngularOrientation().firstAngle * 180 / Math.PI;
    }
}
