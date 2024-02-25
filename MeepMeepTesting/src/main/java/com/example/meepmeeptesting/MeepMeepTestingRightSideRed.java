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
                        drive.trajectorySequenceBuilder(new Pose2d(-32.5, -62, Math.toRadians(90)))
                                .splineTo(new Vector2d(-39, -38.5), Math.toRadians(130))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, -57), Math.toRadians(0))
                                .back(45)
                                .splineTo(new Vector2d(50, -30), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity blueBotPos2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32.5, -62, Math.toRadians(90)))
                                .splineTo(new Vector2d(-35, -33), Math.toRadians(90))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, -57), Math.toRadians(0))
                                .back(45)
                                .splineTo(new Vector2d(50, -36), Math.toRadians(0))
                                .build()


                );
        RoadRunnerBotEntity blueBotPos3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9, -62, Math.toRadians(90)))
                                .lineTo(new Vector2d(14, -53))
                                .splineTo(new Vector2d(27,-38), Math.toRadians(100))
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(57, -40), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(2)
                                //cycle
                                .splineTo(new Vector2d(20, -59), Math.toRadians(180))
                                .splineTo(new Vector2d(-27, -59), Math.toRadians(180))
                                .splineTo(new Vector2d(-59, -39), Math.toRadians(145))
                                .waitSeconds(3)
                                .setReversed(true)
                                .splineTo(new Vector2d(-23, -59), Math.toRadians(0))
                                .splineTo(new Vector2d(23, -59), Math.toRadians(0))
                                .splineTo(new Vector2d(49, -40), Math.toRadians(25))
                                .waitSeconds(2)
                                //park
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(45, -40, Math.toRadians(180)))
                                .strafeLeft(25)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(blueBotPos1)
//                .addEntity(blueBotPos2)
                .addEntity(blueBotPos3)
                .start();
    }

}
