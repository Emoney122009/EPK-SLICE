// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
/**
 * Assorted Constant Numbers
 */
    public final class c{
        /*DriveTrain Constants*/
        public final static double wDiameter = 6; //in inches
        public final static double wCircumference = 0.4787787204; // IN METERS
        public final static double dtGearRatio = 10.75; //Gear Reduction 10.75:1
        public final static double xMod = 0.6;
        public final static double yMod = 0.6;

        public final class dt{
            public final static double kP = 0;
            public final static double kI = 0;
            public final static double kD = 0;
        }
        public final class sh{
            public final static double topA = 0.250845;
            public final static double topB = -128.478;
            public final static double topC =  1476.97;
            public final static double botA = -0.0578704;
            public final static double botB =  37.6364;
            public final static double botC = 3394.7;
            public final static double kP =  0.00001;
            public final static double kI =  0.00000001;
            public final static double kD = 0;
            public final static double kFF = 0;
            public final static double kiZ = 0;
            public final static double botHighFender = 1600;
            public final static double topHighFender = 400;
            public final static double botLowFender = 400;
            public final static double topLowFender = 400;
        }



        /*Climber Constants*/
        public final static double climberGearRatio = 25; //gear reduction 25:1
        public final static double maxClimbHeight = 360;
        public final static double lowMaxClimbHeight = 300; //How many steps required to reach 2nd rung.
        public final static double minClimbHeight = 0; //How many steps until passive hooks engage or when the climbers cant go any lower.
        public final static double kEncoderTick2Meter = 1.0/4096 *.1* Math.PI;

        /*Neo Motor Constants*/
        public final static double encoderSteps = 4096;

        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) encoderSteps;
    
        public static final boolean kGyroReversed = false;
    
        public static final double kStabilizationP = 1;
        public static final double kStabilizationI = 0.5;
        public static final double kStabilizationD = 0;
    
        public static final double kTurnP = 1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;
    
        public static final double kMaxTurnRateDegPerS = 100;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    
        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    }



/**
 * Motor Identification Numbers
 */
    public final class id{
        
    /* DriveTrain */
    public final static int flDrive = 3;
    public final static int blDrive = 4;
    public final static int frDrive = 1;
    public final static int brDrive = 2;

    /* Shooter */
    public final static int tShooter = 10;
    public final static int blShooter = 9;
    public final static int brShooter = 8;

    /* Indexer */
    public final static int lIndexer = 7;

    /* Climber */
    public final static int lClimber = 6;
    public final static int rClimber = 5;

    /* Intake */
    public final static int rIntake = 11;


    /*Controllers*/
    public final static int lJoy = 1;
    public final static int rJoy = 0;
    public final static int xbox = 3;

    }

}
