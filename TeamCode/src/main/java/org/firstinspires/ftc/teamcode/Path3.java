package org.firstinspires.ftc.teamcode;


import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Path3", group = "Autonomous")
public class Path3 extends BaseLinearOpMode {



    //this set the points
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(54.9817, 8.3251, -90);
    private final Pose path1 = poseFactory.of(55.1701, 26.456, 180);
    private final Pose point2Start = poseFactory.of(55.1701, 26.456, 180);
    private final Pose point2 = poseFactory.of(36.3087, 8.1867, 180);
    private final Pose point3 = poseFactory.of(8.4, 8.1867, 180);
    private final Pose point4Start = poseFactory.of(8.4, 8.1867, 180);
    private final Pose point4 = poseFactory.of(24.1229, 8.2743, 180);
    private final Pose point5 = poseFactory.of(22.3284, 85.1467, 97.8533);
    private final Pose point6Start = poseFactory.of(22.3284, 85.1467, 90);
    private final Pose point6 = poseFactory.of(61, 133, 90);
    private final Pose point7Start = poseFactory.of(56.3404, 121.4953, 180);
    private final Pose point7 = poseFactory.of(35.8294, 93.7145, 0);
    private final Pose point8Start = poseFactory.of(35.8294, 93.7145, 180);
    private final Pose point8 = poseFactory.of(12.9817, 93.7145, 180);
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
                waitMs(2000),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                shooter.shoot(),
                follow(follower, path7()),
                follow(follower, path8())
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
        return Paths.line(point4, point5).tangent();
    }

    public Path path6() {
        return Paths.line(point6Start, point6)
                .linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }
}