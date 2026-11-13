package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.OpModeStorage;

@Autonomous(name = "Path2", group = "Autonomous")
public class Path2 extends BaseLinearOpMode {

    //this set the points
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(58.5963, 132.8835, 90);
    private final Pose path1 = poseFactory.of(45.355, 116.4862, 180);
    private final Pose point2Start = poseFactory.of(45.355, 116.4862, 180);
    private final Pose point2 = poseFactory.of(11.0344, 116.5622, 0);

    @Override
    protected Pose startPose() {
        return start;
    }

    public Command autoRoutine() {
        return sequential(
                //this is what we do once we start
                waitMs(10000),
                //this is the shooting
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                //this is the pedro part
                follow(follower, path1()),
                follow(follower, path2())
        );
    }



    //these are all of the actual paths
    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

}