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
public final class constants {

  public static final class SubsystemConstants {
    public static final int kElevatorMotorCanId = 21;
    public static final int kArmMotorCanId = 16;
    public static final int kIntakeMotorCanId = 19;
    
//Done do more then -58
    public static final class ElevatorSetpoints {
      public static final double KIntake = 0;
      public static final int kResting = 0;
      public static final int kLevel1 = -2;
      public static final int kLevel2 = -10;
      public static final int kLevel3 = -28;
      public static final int kLevel4 = -57;
      //Needs to be determined
      public static final int kballgroundintake= 0;
      public static final int kballLevel1 = -18;
      public static final int kballLevel2 = -32;
      public static final int kballLevelbarge = -58; // MAX HIGHT

    }
    // to point the arm forward for scoring write as a negitive and if going for intake go for positive
    public static final class ArmSetpoints {
      public static final double Kscore = -4;
      public static final double KFeederIntake = 0.25;
      public static final double kStowPosition = -2.25;
      public static final double kLevel1 = -3;
      public static final double kLevel2 = -3;
      public static final double kLevel3 = -3;
      public static final double kLevel4 = -7;
      //Needs to be determined!!!
      public static final double kballgroundintake = -15;
      public static final double kballLevel1 = -15;
      public static final double kballLevel2 = -15;
      public static final double kballbarge = -17;
      public static final double kballStow = -15;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -0.5;
      public static final double kstop = 0;
    }
    }
  
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class LEDConfig {
    public static final int LED_Port = 9;
    public static final int LED_Length = 67;  
    
  }

  public static final int BLINKIN_LED_CONTROLLER_PORT = 9;

}