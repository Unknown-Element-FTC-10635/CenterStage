package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingRightSideRed {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity blueBotPos1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32.5, 62, Math.toRadians(270)))
                                .splineTo(new Vector2d(-39, 38.5), Math.toRadians(210))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 57), Math.toRadians(0))
                                .back(45)
                                .splineTo(new Vector2d(50, 41.5), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity blueBotPos2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32.5, 62, Math.toRadians(270)))
                                .splineTo(new Vector2d(-35, 33), Math.toRadians(270))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 57), Math.toRadians(0))
                                .back(45)
                                .splineTo(new Vector2d(50, 36), Math.toRadians(0))
                                .build()
                );
        RoadRunnerBotEntity blueBotPos3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32.5, 62, Math.toRadians(270)))
                                .splineTo(new Vector2d(-32.5, 38.5), Math.toRadians(330))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 57), Math.toRadians(0))
                                .back(45)
                                .splineTo(new Vector2d(50, 30), Math.toRadians(0))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueBotPos1)
                .addEntity(blueBotPos2)
                .addEntity(blueBotPos3)
                .start();
    }

}
