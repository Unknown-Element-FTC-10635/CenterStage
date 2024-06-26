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
        MeepMeep meepMeep = new MeepMeep(700);

        // Declare our first bot
        RoadRunnerBotEntity blueBotPos1 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9, 61, Math.toRadians(270)))
                                //drop yellow
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(48, 32.5, Math.toRadians(180)))
                                //drop purple
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(24, 24,  Math.toRadians(180)))
                                //drive to stack
                                .lineToLinearHeading(new Pose2d(5,35, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-40,35, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-62,34,Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(0,32, Math.toRadians(180)))
                                //drive back
                                .lineToLinearHeading(new Pose2d(48, 31, Math.toRadians(180)))
                                //drive back to stack
                                .lineToLinearHeading(new Pose2d(-60,32, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(0,32, Math.toRadians(180)))
                                //drive back
                                .lineToLinearHeading(new Pose2d(48, 32.5, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity blueBotPos2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
                        .splineTo(new Vector2d(-46, -33), Math.toRadians(100))
                        .back(8)
                        .back(10)
                        .setReversed(false)
                        .splineTo(new Vector2d(-58, -39), Math.toRadians(180))
                        .setReversed(true)
                        .splineTo(new Vector2d(-30, -58), Math.toRadians(180))
                        .back(53)
                        .setReversed(true)
                        .splineTo(new Vector2d(48, -26), Math.toRadians(180))
                        .back(5)
                        .build()


        );
//
        RoadRunnerBotEntity blueBotPos3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
                                .splineTo(new Vector2d(-30, -38), Math.toRadians(0))
                                .build()

                );


//
        RoadRunnerBotEntity redBotPos1 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -62, Math.toRadians(90)))
                            .splineTo(new Vector2d(9, -38.5), Math.toRadians(135))
                            .waitSeconds(2)
                            .back(10)
                            .turn(Math.toRadians(90))
                            .back(5)
                            .setReversed(true)
                            .splineTo(new Vector2d(46, -28), Math.toRadians(90))
                            .back(10)
                            .waitSeconds(1)
                            .forward(15)
                            .lineTo(new Vector2d(45, -60))
                            .build()
                );


        RoadRunnerBotEntity redBotPos2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -62, Math.toRadians(90)))
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
                        drive.trajectorySequenceBuilder(new Pose2d(10, -62, Math.toRadians(90)))
                                .lineTo(new Vector2d(25, -42))
                                .waitSeconds(2)
                                .back(10)
                                .turn(Math.toRadians(90))
                                .setReversed(true)
                                .lineTo(new Vector2d(50, -38))
                                .back(8)
                                .waitSeconds(2)
                                .forward(10)
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
//                  .addEntity(blueBotPos1)
                 .addEntity(blueBotPos1)
//                 .addEntity(blueBotPos3)
//               .addEntity(redBotPos1)
//                 .addEntity(redBotPos2)
//                .addEntity(redBotPos3)
                .start();
    }
}