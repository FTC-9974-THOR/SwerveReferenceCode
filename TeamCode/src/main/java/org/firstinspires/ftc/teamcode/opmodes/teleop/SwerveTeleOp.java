package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.Swerve;
import org.ftc9974.thorcore.control.AutoHeadingHold;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.BooleanEdgeDetector;

import java.util.List;

/**
 * This is an example code for a teleop with the swerve.
 *
 * This is basically what the swerve was running at CRI, and it allows for on-the-fly switching
 * between drive handedness and field/robot relative driving.
 */
@TeleOp(name = "Swerve TeleOp")
public class SwerveTeleOp extends OpMode {

    private Swerve swerve;
    private AutoHeadingHold autoHeadingHold;

    private List<LynxModule> hubs;
    @Nullable private VoltageSensor voltageSensor;

    @Override
    public void init() {
        swerve = new Swerve(hardwareMap);
        autoHeadingHold = new AutoHeadingHold(6, 0.7, 2, 0, swerve.imu);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        List<VoltageSensor> voltageSensors = hardwareMap.getAll(VoltageSensor.class);
        if (!voltageSensors.isEmpty()) voltageSensor = voltageSensors.get(0);
        else voltageSensor = null;
    }

    @Override
    public void init_loop() {
        // these are used for setting servo offsets.
        telemetry.addData("FL Raw Position", swerve.fl.servo.getPosition());
        telemetry.addData("FR Raw Position", swerve.fr.servo.getPosition());
        telemetry.addData("BL Raw Position", swerve.bl.servo.getPosition());
        telemetry.addData("BR Raw Position", swerve.br.servo.getPosition());
    }

    private boolean rightHandDrive = false;
    private boolean isFieldCentric = false;
    private double fieldCentricReference = 0;
    private final BooleanEdgeDetector fieldCentricEdgeDetector = new BooleanEdgeDetector(false);

    // this object is reused to avoid having to allocate a new one every time through loop(). heap
    // allocations are expensive, and in the interest of loop times we want to avoid them when we
    // can.
    private final Vector2 driveInputLinear = new Vector2(0, 0);

    @Override
    public void loop() {
        // clear bulk caches
        hubs.forEach(LynxModule::clearBulkCache);

        // drive handedness selection logic
        if (gamepad1.dpad_left) rightHandDrive = false;
        if (gamepad1.dpad_right) rightHandDrive = true;
        // field centric driving toggle
        fieldCentricEdgeDetector.update(gamepad1.dpad_up);
        if (fieldCentricEdgeDetector.isRising()) isFieldCentric = !isFieldCentric;
        // field centric reference reset logic
        if (gamepad1.dpad_down) fieldCentricReference = swerve.imu.getHeading();

        // speed selection logic
        double speed = 0.8; // default speed
        if (gamepad1.right_trigger > 0) speed = 1; // full speed
        else if (gamepad1.left_trigger > 0) speed = 0.4; // granny speed

        // get inputs from the gamepad (x, y, and turning)
        double rawXInput, rawYInput, rawTInput;
        if (rightHandDrive) {
            rawXInput = -gamepad1.right_stick_y;
            rawYInput = -gamepad1.right_stick_x;
            rawTInput = -gamepad1.left_stick_x;
        } else {
            rawXInput = -gamepad1.left_stick_y;
            rawYInput = -gamepad1.left_stick_x;
            rawTInput = -gamepad1.right_stick_x;
        }

        double xInput = speed * Swerve.FULL_LINEAR_SPEED * rawXInput;
        double yInput = speed * Swerve.FULL_LINEAR_SPEED * rawYInput;
        double tInput = speed * Swerve.FULL_ANGULAR_SPEED * rawTInput;

        // field centric driving logic
        if (isFieldCentric) {
            // rotate x and y input from the field to the robot
            // reference: https://en.wikipedia.org/wiki/Rotation_matrix
            double theta = -(swerve.imu.getHeading() - fieldCentricReference);
            double rotatedX = xInput * cos(theta) - yInput * sin(theta);
            double rotatedY = xInput * sin(theta) + yInput * cos(theta);
            xInput = rotatedX;
            yInput = rotatedY;
        }

        driveInputLinear.setX(xInput);
        driveInputLinear.setY(yInput);
        double driveInputAngular = autoHeadingHold.update(tInput);

        // drive() expects linear velocity in mm/s and angular velocity in rad/s.
        // +X is forward. +Y is left. +T is counterclockwise.
        swerve.drive(driveInputLinear, driveInputAngular);

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

        telemetry.addData("Drive Mode", String.format("%s/%s",
                rightHandDrive ? "RHD" : "LHD",
                isFieldCentric ? "Field Centric" : "Robot Centric"
        ));
        double current = swerve.fl.getCurrentDraw() +
                swerve.fr.getCurrentDraw() +
                swerve.bl.getCurrentDraw() +
                swerve.br.getCurrentDraw();
        telemetry.addData("Motor Current Draw (A)", current);
        if (voltageSensor != null) {
            telemetry.addData("Motor Power Draw (W)", voltageSensor.getVoltage() * current);
        }
    }
}
