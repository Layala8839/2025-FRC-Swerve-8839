package frc.robot.subsystems.Other.Sensor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANRange extends SubsystemBase{
      public static final Boolean getIsDetected = getIsDetected().getValue();
            private final static CANrange canrange = new CANrange(1);
      
    public void Robot() {
    /* Configure CANcoder */
    var toApply = new CANrangeConfiguration();
    System.out.println("Configuration: " + toApply);

    /* User can change the configs if they want, or leave it empty for factory-default */
    canrange.getConfigurator().apply(toApply);

    /* Set the signal update rate */
    BaseStatusSignal.setUpdateFrequencyForAll(50, canrange.getIsDetected());


     boolean detected = canrange.getIsDetected().getValue();
      SmartDashboard.putBoolean("Detected", detected);
}

private static StatusSignal<Boolean> getIsDetected() {
      // TODO Auto-generated method stub
      SmartDashboard.putBoolean("Detected", getIsDetected);
            return null;}
}
