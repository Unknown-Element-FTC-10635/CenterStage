package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingLeftSide{
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(500);

        // Declare our first bot
        RoadRunnerBotEntity blueBotPos1 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        blueBotPos1.runAction(blueBotPos1.getDrive().actionBuilder(new Pose2d(-38, 62, Math.toRadians(270)))
                .splineTo(new Vector2d(-45, 38.5), Math.toRadians(270))
                .build());

        RoadRunnerBotEntity blueBotPos2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .build();

        blueBotPos2.runAction(blueBotPos2.getDrive().actionBuilder(new Pose2d(-38, 62, Math.toRadians(270)))
                .splineTo(new Vector2d(-35, 35), Math.toRadians(270))
                .build());

        RoadRunnerBotEntity blueBotPos3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .build();

        blueBotPos3.runAction(blueBotPos3.getDrive().actionBuilder(new Pose2d(-38, 62, Math.toRadians(270)))
                .splineTo(new Vector2d(-37, 35), Math.toRadians(340))
                .build());

        RoadRunnerBotEntity redBotPos1 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redBotPos1.runAction(redBotPos1.getDrive().actionBuilder(new Pose2d(-38, -62, Math.toRadians(90)))
                .splineTo(new Vector2d(-45, -38.5), Math.toRadians(90))
                .build());

        RoadRunnerBotEntity redBotPos2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .build();

        redBotPos2.runAction(redBotPos2.getDrive().actionBuilder(new Pose2d(-38, -62, Math.toRadians(90)))
                .splineTo(new Vector2d(-35, -35), Math.toRadians(95))
                .build());

        RoadRunnerBotEntity redBotPos3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60,60, Math.toRadians(180), Math.toRadians(105), 15)
                .build();

        redBotPos3.runAction(redBotPos3.getDrive().actionBuilder(new Pose2d(-38, -62, Math.toRadians(90)))
                .splineTo(new Vector2d(-34, -38.5), Math.toRadians(45))
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(blueBotPos3)
                .addEntity(redBotPos3)
                .addEntity(blueBotPos2)
                .addEntity(redBotPos2)
                .addEntity(blueBotPos1)
                .addEntity(redBotPos1)

                .start();

    }
}
