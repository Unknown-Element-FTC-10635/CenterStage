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
        MeepMeep meepMeep = new MeepMeep(780);

        // Declare our first bot
        RoadRunnerBotEntity blueBotPos1 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, 62, Math.toRadians(270)))
                                .splineTo(new Vector2d(9, 38.5), Math.toRadians(225))
                                .waitSeconds(2)
                                .setReversed(true)
                                .splineTo(new Vector2d(50, 41.5), Math.toRadians(0))
                                .forward(15)
                                .lineTo(new Vector2d(45,60))
                                .build()
                );

        RoadRunnerBotEntity blueBotPos2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(15, 62, Math.toRadians(270)))
                        .splineTo(new Vector2d(10, 35), Math.toRadians(270))
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineTo(new Vector2d(50, 36), Math.toRadians(0))
                        .forward(15)
                        .lineTo(new Vector2d(45,60))
                        .build()
        );
//
        RoadRunnerBotEntity blueBotPos3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, 62, Math.toRadians(270)))
                                .splineTo(new Vector2d(25, 38.5), Math.toRadians(230))
                                .waitSeconds(2)
                                .setReversed(true)
                                .splineTo(new Vector2d(50, 30), Math.toRadians(0))
                                .forward(15)
                                .lineTo(new Vector2d(45,60))
                                .build()
                );


//
        RoadRunnerBotEntity redBotPos1 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -62, Math.toRadians(90)))
                            .splineTo(new Vector2d(9, -38.5), Math.toRadians(135))
                            .waitSeconds(2)
                            .setReversed(true)
                            .splineTo(new Vector2d(50, -30), Math.toRadians(0))
                            .forward(15)
                            .lineTo(new Vector2d(45, -60))
                            .build()
                );


        RoadRunnerBotEntity redBotPos2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -62, Math.toRadians(90)))
                                .splineTo(new Vector2d(10, -35), Math.toRadians(95))
                                .waitSeconds(2)
                                .setReversed(true)
                                .splineTo(new Vector2d(50, -36), Math.toRadians(0))
                                .forward(15)
                                .lineTo(new Vector2d(45, -60))
                                .build()
                );

//
        RoadRunnerBotEntity redBotPos3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -62, Math.toRadians(90)))
                                .splineTo(new Vector2d(25, -38.5), Math.toRadians(135))
                                .waitSeconds(2)
                                .setReversed(true)
                                .splineTo(new Vector2d(50, -41.5), Math.toRadians(0))
                                .forward(15)
                                .lineTo(new Vector2d(45, -60))
                                .build()
                );

//        redBotPos3.runAction(redBotPos3.getDrive().actionBuilder(new Pose2d(15, -62, Math.toRadians(90)))
//                .splineTo(new Vector2d(25, -38.5), Math.toRadians(135))
//                .waitSeconds(2)
//                .setReversed(true)
//                .splineTo(new Vector2d(50, -41.5), Math.toRadians(0))
//                .lineToX(45)
//                .strafeTo(new Vector2d(45, -60))
//                .build());
//
//

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                // .addEntity(blueBotPos1)
                // .addEntity(mySecondBot)
                // .addEntity(blueBotPos2)
                // .addEntity(blueBotPos1)
                // .addEntity(blueBotPos2)
                .addEntity(redBotPos3)
                // .addEntity(redBotPos1)
                // .addEntity(redBotPos2)
//                .addEntity(redBotPos3)
                .start();
    }
}