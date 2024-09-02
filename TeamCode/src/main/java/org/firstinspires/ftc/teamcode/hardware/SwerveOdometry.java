package org.firstinspires.ftc.teamcode.hardware;

import static org.ftc9974.thorcore.control.navigation.roadrunner.Odometry.resetEncoder;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.navigation.roadrunner.Odometry;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.sensors.Encoder;
import org.ftc9974.thorcore.util.MathUtilities;

public class SwerveOdometry implements Odometry {

    @Hardware(name = "frontLeft-motor")
    public DcMotorEx left;
    @Hardware(name = "backRight-motor")
    public DcMotorEx right;
    @Hardware(name = "backLeft-motor")
    public DcMotorEx front;

    public final Encoder leftEncoder, rightEncoder, backEncoder;

    public SwerveOdometry(HardwareMap hw) {
        Realizer.realize(this, hw);

        resetEncoder(left);
        resetEncoder(right);
        resetEncoder(front);

        leftEncoder = new Encoder(left);
        rightEncoder = new Encoder(right);
        backEncoder = new Encoder(front);

        // the left and right odometers should count up when the robot moves forward, and the back
        // encoder should count up when the robot moves left.
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        backEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    @Override
    public Encoder leftEncoder() {
        return leftEncoder;
    }

    @Override
    public Encoder rightEncoder() {
        return rightEncoder;
    }

    @Override
    public Encoder horizontalEncoder() {
        return backEncoder;
    }

    // this is the distance between the center of the contact patches of the left and right odometer.
    @Override
    public double lateralDistance() {
        return 315;
    }

    // this is the distance from the center of the robot to the center of the contact patch of the
    // back odometer.
    @Override
    public double horizontalOffset() {
        return 25 - MathUtilities.inchesToMM(7);
    }

    @Override
    public double mmPerTick() {
        return (2.0 * Math.PI * 24.0) / 2000.0;
    }
}