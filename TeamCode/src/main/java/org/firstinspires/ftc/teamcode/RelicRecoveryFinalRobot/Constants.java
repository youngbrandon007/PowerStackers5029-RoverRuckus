package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobotConstants;

import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.ciphers.BIRDBROWN;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.ciphers.BIRDGREY;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.ciphers.FROGBROWN;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.ciphers.FROGGREY;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.ciphers.SNAKEBROWN;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.ciphers.SNAKEGREY;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.glyph.BROWN;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.glyph.GREY;

/**
 * Created by Brandon on 1/8/2018.
 */

public class Constants {
    public static class PID {
        public static final double P = 0.025;
        public static final double I = .0002;
        public static final double D = 0.055;
    }
    public static class alignment {
        public static final double ALIGNLEFTINIT = 0.78;
        public static final double ALIGNLEFTDOWN = 0.45;
        public static final double ALIGNLEFTUP = 0.75;
        public static final double ALIGNRIGHTINIT = 0.17;
        public static final double ALIGNRIGHTDOWN = 0.83;
        public static final double ALIGNRIGHTUP = 0.2;
    }
    public static class drive {
        public static final double wheelSize = 4.0;
        public static final double gearRatio = 2.0 / 3.0;
        public static final double wheelCir = wheelSize * Math.PI;
        public static final double encoderCountPerInch = 1.0 / wheelCir * PineappleRobotConstants.NEV40CPR;

        public static int countsPerInches(double inches) {
            double counts = inches * encoderCountPerInch * gearRatio;
            return (int) counts;
        }
    }

    public static class flip {
        public static final double rightDown = 0.77;
        public static final double leftDown = 0.21;

        public static final double rightFlat = 0.67;
        public static final double leftFlat = 0.32;

        public static final double rightUp = 0.2;
        public static final double leftUp = 0.81;
        public static final double stopUp = 0.5;
        public static final double stopDown = 1;
    }

    public static class relic {
        public static final double grabClose = 0.15;
        public static final double grabIn = 0;
        public static final double grabOpen = 0.6;
        public static final double turnStraight = 0.7;
        public static final double turnDown = 0;
        public static final double turnFold = 1;

    }

    public static class auto {

        public static class autoGlyph {
            public enum glyph {
                GREY, BROWN, NONE
            }
            public enum column {
                LEFT, CENTER, RIGHT, NONE
            }
            //START IN TOP LEFT
            public static class ciphers {
                public static final glyph[][] FROGGREY = {
                        {GREY, BROWN, GREY},
                        {BROWN, GREY, BROWN},
                        {GREY, BROWN, GREY},
                        {BROWN, GREY, BROWN}
                };
                public static final glyph[][] FROGBROWN = {
                        {BROWN, GREY, BROWN},
                        {GREY, BROWN, GREY},
                        {BROWN, GREY, BROWN},
                        {GREY, BROWN, GREY}
                };
                public static final glyph[][] BIRDGREY = {
                        {GREY, BROWN, GREY},
                        {BROWN, GREY, BROWN},
                        {BROWN, GREY, BROWN},
                        {GREY, BROWN, GREY}
                };
                public static final glyph[][] BIRDBROWN = {
                        {BROWN, GREY, BROWN},
                        {GREY, BROWN, GREY},
                        {GREY, BROWN, GREY},
                        {BROWN, GREY, BROWN}
                };
                public static final glyph[][] SNAKEGREY = {
                        {GREY, GREY, BROWN},
                        {GREY, BROWN, BROWN},
                        {BROWN, BROWN, GREY},
                        {BROWN, GREY, GREY}
                };
                public static final glyph[][] SNAKEBROWN = {
                        {BROWN, BROWN, GREY},
                        {BROWN, GREY, GREY},
                        {GREY, GREY, BROWN},
                        {GREY, BROWN, BROWN}
                };

            }

            public static final glyph[][][] CIPHERS = {FROGBROWN, FROGGREY,BIRDBROWN,BIRDGREY,SNAKEBROWN,SNAKEGREY };

        }

        public static class aligning {
            public static final double leftUp = .5;
            public static final double leftDown = .4;
            public static final double rightUp = .5;
            public static final double rightDown = .6;


            public static final double[][][] AlignArmPosition =
                    {
                            {{alignment.ALIGNRIGHTUP, alignment.ALIGNRIGHTUP, alignment.ALIGNRIGHTDOWN  },{alignment.ALIGNLEFTDOWN, alignment.ALIGNLEFTDOWN, alignment.ALIGNLEFTUP}},
                            {{alignment.ALIGNRIGHTUP, alignment.ALIGNRIGHTUP, alignment.ALIGNRIGHTUP  },{alignment.ALIGNLEFTDOWN, alignment.ALIGNLEFTDOWN, alignment.ALIGNLEFTDOWN}}
                    };
            public static final double[][] AlignDrivingDirection =
                    {
                            {0,0,180},
                            {0, 0,0}
                    };
            public static final boolean[][][] AlignSwitchClicked =
                    {
                            {{false, false, true}, {true, true, false}},
                            {{false, false, false}, {true, true, true}}
                    };
            public static final double[][] AlignDrivingOffPlatformEncoder =
                    {
                            {22 , 18, 16},
                            {0,0,0}
                    };
            public static final double[] AlignDriveOffPlatformDirection =
                    {270, 90};
            public static final double[] AlignTurnAngle =
                    {90, 90};

            public static final double CollectDistToPit = 10;
            public static final int collectDriveIntoPitTime = 2000;
            public static final double GlyphDistanceToCrypto = 14;
            public static final double columnStraffDistance = 7;
            public static final double alignStraffDistance = 1.5;//1.5;
        }

        public static class jewel {
            public enum jewelState {
                BLUE_RED, RED_BLUE, NON_NON
            }
            public enum jewelHitSide {
                RIGHT, LEFT, NONE
            }
            public static final double JEWELDOWN = 0.6;
            public static final double JEWELUP = 0;
            public static final double JEWELHITRIGHT = 0;
            public static final double JEWELHITLEFT = 1;
            public static final double JEWELHITCENTER = 0.5;

            public static final int JEWELDOWNMILI = 1500;
            public static final int JEWELUPMILI = 1000;
            public static final int JEWELHITMILI = 1000;

        }
    }

}
