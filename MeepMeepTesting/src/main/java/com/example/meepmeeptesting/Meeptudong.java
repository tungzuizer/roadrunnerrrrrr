package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Meeptudong {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 62, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(0,34),Math.toRadians(270))


                .strafeToLinearHeading(new Vector2d(0,40),Math.toRadians(270))



                .strafeToLinearHeading(new Vector2d(-24,38),Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-24,39),Math.toRadians(150))
                .strafeToLinearHeading(new Vector2d(-34.5,38),Math.toRadians(210))
                .strafeToLinearHeading(new Vector2d(-34.5,39),Math.toRadians(150))





                .strafeToLinearHeading(new Vector2d(-38,55),Math.toRadians(270))



                .strafeToLinearHeading(new Vector2d(-38,61),Math.toRadians(270))

                .strafeToLinearHeading(new Vector2d(0,34),Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-38,61),Math.toRadians(270))
                .waitSeconds(0)

                 .strafeToLinearHeading(new Vector2d(0,34),Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-38,61),Math.toRadians(270))
                .waitSeconds(0)

                .strafeToLinearHeading(new Vector2d(0,34),Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-38,61),Math.toRadians(270))
                .waitSeconds(0)

                .strafeToLinearHeading(new Vector2d(0,34),Math.toRadians(270))
                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-38,61),Math.toRadians(270))
                .waitSeconds(0)

                .build());



        myBot.setDimensions ( 16.5, 16);


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
