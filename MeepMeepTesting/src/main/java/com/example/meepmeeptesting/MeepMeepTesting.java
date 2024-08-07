package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(910, 682);
//        MeepMeep meepMeep = new MeepMeep(500);

        // Left = 1, center = 2, right = 3


        RoadRunnerBotEntity blueCenter2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(33, 61, Math.toRadians(270)))
                                //drop yellow
                                .setReversed(true)
                                .forward(5)
                                .lineToLinearHeading(new Pose2d(49, 22,  Math.toRadians(180)))
                                //drop yellow
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(76, 34, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(35, 10, Math.toRadians(180)))
                                .setReversed(false)
                                .splineTo(new Vector2d(-15, 6), Math.toRadians(220))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(-6, 12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(30, 13, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(76, 6, Math.toRadians(180)))



                                .build()
                );

        RoadRunnerBotEntity blueCenter1 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-12, 61, Math.toRadians(270)))
                                //drop yellow
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(-12, 11, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-17, 10, Math.toRadians(230)))
                                .lineToLinearHeading(new Pose2d(-3, 13, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(48, 13, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(71, 33, Math.toRadians(180)))


                                .build()
                );

        RoadRunnerBotEntity blueCenter3 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-12, 61, Math.toRadians(270)))
                                //drop yellow
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(-12, 11, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-17, 10, Math.toRadians(230)))
                                .lineToLinearHeading(new Pose2d(-3, 13, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(48, 13, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(71, 33, Math.toRadians(180)))

                                //drop purple
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK_CRI)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
//                  .addEntity(blueBotPos1)
                 .addEntity(blueCenter2)
//                 .addEntity(blueBotPos3)
//               .addEntity(redBotPos1)
//                 .addEntity(redBotPos2)
//                .addEntity(redBotPos3)
                .start();
    }
}