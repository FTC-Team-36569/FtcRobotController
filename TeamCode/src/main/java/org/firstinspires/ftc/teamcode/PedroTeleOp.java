package org.firstinspires.ftc.teamcode;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.OpModeStorage;

@TeleOp(name = "Pedro TeleOp")
public class PedroTeleOp extends OpMode {

    public static final double FORWARD = Math.toRadians(0);
    private CRServo windmill = null;
    private CRServo intakeServoLeft = null;

    private CRServo intakeServoRight = null;
    private DcMotor intake = null;

    private Follower follower;

    private Shooter shooter;


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        shooter = new Shooter(hardwareMap);
        //this set the names
        windmill = hardwareMap.get(CRServo.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeServoLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeServoRight = hardwareMap.get(CRServo.class, "intakeRight");
    }

    @Override
    public void loop() {
        //this loop is what happens when we start
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading(),
                -FORWARD
        );
        if (gamepad1.left_bumper) {
            telemetry.addData("left button","pressed");

        }
        telemetry.addData("right button",gamepad1.right_trigger);
        windmill.setPower(gamepad1.right_trigger>0.5 ? -1.0 : 0.0);
        intake.setPower(gamepad1.left_bumper ? 1.0 : -1.0);

        if (gamepad1.options) {
            follower.setPose(follower.pose().withHeading(0));
        }

        //this set power to the intakes
        intake.setPower(1.0);
        intakeServoLeft.setPower(1.0);
        intakeServoRight.setPower(-1.0);

        follower.manual(powers);
        follower.update();
        Pose robotPose = follower.pose();
        // returns a Pose object

        //this adds the data on the screen
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }

    @Override
    public void start() {
        //this tells it what to do once the start button is pressed
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
        shooter.spinUp();
    }
    //this stop the shooter
    @Override
    public void stop() {
        shooter.stop();
    }
}