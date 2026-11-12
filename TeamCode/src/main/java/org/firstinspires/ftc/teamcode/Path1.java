package org.firstinspires.ftc.teamcode;


import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Path1", group = "Autonomous")
public class Path1 extends BaseLinearOpMode {



    //this set the points
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(54.9817, 8.3251, -90);
    private final Pose path1 = poseFactory.of(55.1701, 26.456, 180);
    private final Pose point2Start = poseFactory.of(55.1701, 26.456, 180);
    private final Pose point2 = poseFactory.of(36.3087, 8.1867, 180);
    private final Pose point3 = poseFactory.of(8.4, 8.1867, 180);
    private final Pose point4Start = poseFactory.of(8.4, 8.1867, 180);
    private final Pose point4 = poseFactory.of(24.1229, 8.2743, 180);
    private final Pose point5Start = poseFactory.of(24.1229, 8.2743, 180);
    private final Pose point5 = poseFactory.of(30.5069, 93.1953, 180);
    private final Pose point6Start = poseFactory.of(30.5069, 93.1953, 180);
    private final Pose point6 = poseFactory.of(12.5922, 92.9356, 0);

    @Override
    protected Pose startPose() {
        return start;
    }

    public Command autoRoutine() {
        return sequential(
                //this is what we do once we start
                waitMs(1000),
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
                follow(follower, path2()),
                race(
                        waitMs(3000),
                        follow(follower, path3())
                ),
//                race(follow(follower, path3()), waitMs(2000)),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
        );
    }




    //these are all of the actual paths

    public Path path1() {
        return Paths.line(start, path1)
                .linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2)
                .linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3)
                .tangent()
                .with(Constants.foresightConfig.maxPathSpeed.at(0.2));

    }


    public Path path4() {
        return Paths.line(point4Start, point4)
                .linear(point4Start, point4)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));

    }

    public Path path5() {
        return Paths.line(point5Start, point5)
                .linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6)
                .linear(point6Start, point6);
    }
}