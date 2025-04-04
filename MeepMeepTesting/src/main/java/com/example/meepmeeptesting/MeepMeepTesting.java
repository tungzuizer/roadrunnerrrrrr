package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();



        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, 62, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(52,52),Math.toRadians(225))
                .waitSeconds(0)
                .splineTo(new Vector2d(48,38),Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(52,52),Math.toRadians(225))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(58,39),Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(52,52),Math.toRadians(225))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(55,28.15),Math.toRadians(0))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(52,52),Math.toRadians(225))
                .build());



        myBot.setDimensions ( 15, 16);


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
