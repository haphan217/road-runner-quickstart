package com.example.meepmeepftc;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepFTC {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(58, 10, Math.toRadians(135)))
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(40, 46,Math.toRadians(180)), Math.toRadians(170))
                .strafeTo(new Vector2d(58, 10))
                .turn(Math.toRadians(-25))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(18, 47,Math.toRadians(180)), Math.toRadians(165))
                .strafeTo(new Vector2d(58, 10))
                .turn(Math.toRadians(-25))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(20, 20))

//                .actionBuilder(new Pose2d(70, -16, Math.toRadians(220)))
//                .waitSeconds(1)
//                .splineToSplineHeading(new Pose2d(35, -32, Math.toRadians(270)), Math.toRadians(270))
//                .lineToYConstantHeading(-50)
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(55, -10), Math.toRadians(90)).waitSeconds(1)
//                .splineToSplineHeading(new Pose2d(20, -28, Math.toRadians(270)), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(12, -50), Math.toRadians(270))
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(55, -10), Math.toRadians(90))
                .build());

        myBot2.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, -48, Math.toRadians(270)))
        .strafeTo(new Vector2d(-12.00, -10)).waitSeconds(2)
                .strafeTo(new Vector2d(-12, -50)).waitSeconds(1).strafeTo(new Vector2d(-12.00, -10)).waitSeconds(2).strafeTo(new Vector2d(
                        0, -50)).build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }
}