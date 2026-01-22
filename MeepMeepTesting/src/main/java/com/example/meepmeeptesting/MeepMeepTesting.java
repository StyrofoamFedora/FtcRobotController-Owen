package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity BlueCloseBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();
        RoadRunnerBotEntity RedCloseBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setColorScheme(new ColorSchemeRedDark())
                .build();
        RoadRunnerBotEntity BlueFarBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setColorScheme(new ColorSchemeBlueLight())
                .build();
        RoadRunnerBotEntity RedFarBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setColorScheme(new ColorSchemeRedLight())
                .build();

        //BLUE CLOSE
        BlueCloseBot.runAction(BlueCloseBot.getDrive().actionBuilder(new Pose2d(-56, -46, Math.toRadians(233.5)))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-30,-20),Math.toRadians(233.5))
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(-11,-20),Math.toRadians(270))
                .strafeTo(new Vector2d(-11,-30))
                .strafeTo(new Vector2d(-11,-52), new TranslationalVelConstraint(4))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-30,-20), Math.toRadians(233.5))
                .waitSeconds(5)
                .strafeTo(new Vector2d(0,-40))
                .build());
        //RED CLOSE
        RedCloseBot.runAction(RedCloseBot.getDrive().actionBuilder(new Pose2d(-56, 46, Math.toRadians(130)))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-30,20),Math.toRadians(130))
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(-11,20),Math.toRadians(90))
                .strafeTo(new Vector2d(-11,30))
                .strafeTo(new Vector2d(-11,52), new TranslationalVelConstraint(4))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-30,20), Math.toRadians(130))
                .waitSeconds(5)
                .strafeTo(new Vector2d(0,40))
                .build());

        BlueFarBot.runAction(BlueFarBot.getDrive().actionBuilder(new Pose2d(60, -16, Math.toRadians(210)))
                .waitSeconds(6)
                .strafeToLinearHeading(new Vector2d(36,-20),Math.toRadians(270))
                .strafeTo(new Vector2d(36,-30))
                .strafeTo(new Vector2d(36,-52), new TranslationalVelConstraint(4))
                .strafeToLinearHeading(new Vector2d(55,-16), Math.toRadians(210))
                .waitSeconds(5)
                .strafeTo(new Vector2d(36,-36))
                .build());

        RedFarBot.runAction(RedFarBot.getDrive().actionBuilder(new Pose2d(60, 16, Math.toRadians(150)))
                .waitSeconds(6)
                .strafeToLinearHeading(new Vector2d(36,20),Math.toRadians(90))
                .strafeTo(new Vector2d(36,30))
                .strafeTo(new Vector2d(36,52), new TranslationalVelConstraint(4))
                .strafeToLinearHeading(new Vector2d(55,16), Math.toRadians(150))
                .waitSeconds(5)
                .strafeTo(new Vector2d(36,36))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueCloseBot)
                .addEntity(RedCloseBot)
                .addEntity(BlueFarBot)
                .addEntity(RedFarBot)
                .start();
    }
}