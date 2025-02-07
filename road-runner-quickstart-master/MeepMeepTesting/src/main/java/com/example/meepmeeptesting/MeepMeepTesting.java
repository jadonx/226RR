package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(115, 115, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d initialPose = new Pose2d(-37, -62, Math.toRadians(0));

        TrajectoryActionBuilder placeSample1 = myBot.getDrive().actionBuilder(initialPose)
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(180))
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample2 = placeSample1.endTrajectory().fresh()
                .setTangent(Math.toRadians(45))
                .lineToYLinearHeading(-48, Math.toRadians(90))
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample2 = grabSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(225))
                .lineToYLinearHeading(-57, Math.toRadians(45))
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample3 = placeSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(96))
                .lineToYLinearHeading(-48, Math.toRadians(88))
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample3 = grabSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(276))
                .lineToYLinearHeading(-57, Math.toRadians(45))
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample4 = placeSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(85))
                .lineToYLinearHeading(-52, Math.toRadians(105))
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample4 = grabSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(260))
                .lineToYLinearHeading(-57, Math.toRadians(45))
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample5 = placeSample4.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25, -9, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample5 = grabSample5.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(270))
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample6 = placeSample5.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25, -5, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample6 = grabSample6.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(270))
                .waitSeconds(1);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

        myBot.runAction(
                new SequentialAction(
                        placeSample1.build(),
                        grabSample2.build(),
                        placeSample2.build(),
                        grabSample3.build(),
                        placeSample3.build(),
                        grabSample4.build(),
                        placeSample4.build(),
                        grabSample5.build(),
                        placeSample5.build(),
                        grabSample6.build(),
                        placeSample6.build()
                )
        );
    }
}