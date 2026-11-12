package org.firstinspires.ftc.teamcode;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.OpModeStorage;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

@TeleOp(name = "Pedro TeleOp")
public class PedroTeleOp extends OpMode {

    public static final double FORWARD = Math.toRadians(0);
    private CRServo windmill = null;
    private CRServo intakeServoLeft = null;

    private CRServo intakeServoRight = null;
    private DcMotor intake = null;

    private Follower follower;

    private Shooter shooter;

    private AprilTagWebcam webcam;

    // ----- Smart shot aiming numbers (tune these) -----
// How hard to turn for each degree we are off target.
// Too small = slow to aim. Too big = robot wiggles back and forth.
    public static double TURN_GAIN = 0.02;
    // Fastest the robot is allowed to turn while aiming (0 to 1)
    public static double MAX_AIM_TURN = 0.4;
    // How many degrees off is still "lined up" and OK to shoot
    public static double AIM_TOLERANCE = 3.0;
    // If shots always miss to one side, change this (degrees). Try 1 or -1 first.
    public static double AIM_OFFSET = 4.0;


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        shooter = new Shooter(hardwareMap);
        webcam = new AprilTagWebcam();
        webcam.init(hardwareMap, telemetry);
        //this set the names
        windmill = hardwareMap.get(CRServo.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeServoLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeServoRight = hardwareMap.get(CRServo.class, "intakeRight");
    }

    @Override
    public void loop() {
        //this loop is what happens when we start

        double turn = -gamepad1.right_stick_x;

        boolean smartShoot = gamepad1.left_trigger > 0.5;
        boolean standardShoot = gamepad1.right_trigger > 0.5;

        boolean linedUp = false;
        boolean atSpeed = false;
        boolean farEnough = false;
        boolean readyToShoot = false;

        atSpeed = shooter.isAtSpeed();

        webcam.update();
        AprilTagClusterDetection target = webcam.getClosestCluster("BLUE");
        if (target != null) {
            telemetry.addData("Target", target.metadata.name);
            telemetry.addData("Distance (in)", "%.1f", target.ftcPose.range);
            telemetry.addData("Aim error (deg)", "%.1f", target.ftcPose.bearing + AIM_OFFSET);
            telemetry.addData("Speed for this distance", "%.0f", Shooter.speedForDistance(target.ftcPose.range));
            linedUp = Math.abs(target.ftcPose.bearing + AIM_OFFSET) < AIM_TOLERANCE;
            farEnough = target.ftcPose.range >= Shooter.MIN_SHOOT_DISTANCE;
        } else {
            telemetry.addData("Target", "NOT FOUND");
        }

        if (smartShoot) {
            if (target != null){

                double aimError = target.ftcPose.bearing + AIM_OFFSET;
                double distance = target.ftcPose.range;
                turn = Range.clip(aimError * TURN_GAIN, -MAX_AIM_TURN, MAX_AIM_TURN);
                if (turn < 0 && turn > -0.1) {
                    turn = -0.1;
                }
                if (turn < 0.1 && turn > 0) {
                    turn = 0.1;
                }
                shooter.setSpeed(Shooter.speedForDistance(distance));
                readyToShoot = linedUp && atSpeed && farEnough;
            }
        } else {
            shooter.spinUp();
        }
        telemetry.addData("TurnPower", turn);
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turn,
                follower.pose().heading(),
                -FORWARD
        );
        if (gamepad1.left_bumper) {
            telemetry.addData("left button","pressed");

        }
        telemetry.addData("right button",gamepad1.right_trigger);
        windmill.setPower((standardShoot || readyToShoot) ? -1.0 : 0.0);
        intake.setPower(gamepad1.left_bumper ? -1.0 : 1.0);

        if (gamepad1.options) {
            follower.setPose(follower.pose().withHeading(0));
        }
        // This sets the power to the intakes
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