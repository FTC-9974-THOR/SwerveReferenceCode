package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Swerve;
import org.ftc9974.thorcore.control.AutoHeadingHold;
import org.ftc9974.thorcore.control.math.Vector2;

import java.util.List;

/**
 * This is a minimal example of what it takes to get a swerve running.
 */
@TeleOp(name = "Swerve TeleOp (Simplified)")
public class SwerveTeleOpSimple extends OpMode {

    private Swerve swerve;
    private AutoHeadingHold autoHeadingHold;

    private List<LynxModule> hubs;

    @Override
    public void init() {
        swerve = new Swerve(hardwareMap);
        autoHeadingHold = new AutoHeadingHold(6, 0.7, 2, 0, swerve.imu);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        // clear bulk caches
        hubs.forEach(LynxModule::clearBulkCache);

        // get inputs from the gamepad (x, y, and turning)
        double xInput = Swerve.FULL_LINEAR_SPEED * -gamepad1.left_stick_y;
        double yInput = Swerve.FULL_LINEAR_SPEED * -gamepad1.left_stick_x;
        double tInput = Swerve.FULL_ANGULAR_SPEED * -gamepad1.right_stick_x;

        // drive() expects linear velocity in mm/s and angular velocity in rad/s.
        // +X is forward. +Y is left. +T is counterclockwise.
        swerve.drive(new Vector2(xInput, yInput), autoHeadingHold.update(tInput));

        // drive just sets the setpoints for the swerve modules. update is what actually runs the
        // controllers and makes things move. SwerveDrive2.update has 2 overloads: update() and
        // update(Vector2 currentRobotLinearVelocity, double currentRobotAngularVelocity). the
        // provided Swerve class only uses the first overload. this overload is meant for use with
        // constant gain schedules, like what this code uses. it passes zeros to most of the gain
        // schedule parameters. if you want to use a non-constant gain schedule, you'll need to use
        // this:
        // swerve.rb.drive(new Vector2(linearX, linearY), angular);
        // you'll need to provide the linear and angular velocities (linearX, linearyY, and angular)
        // yourself. the easiest way to get these is from odometry using OdometryLocalizer.
        swerve.update();
    }
}
