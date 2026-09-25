package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.TunableVariable.TunableDouble;
import java.util.HashMap;
import java.util.Map;

public class PreferenceManager {

  public class TunablePID {
    private static final Map<String, TunableDouble[]> PIDMap = new HashMap<>();

    public void ApplyTunableValues(String name, PIDController controller) {
      TunableDouble[] values =
          PIDMap.computeIfAbsent(
              name,
              subsystemName ->
                  new TunableDouble[] {
                    new TunableDouble(subsystemName + "kP", controller.getP()),
                    new TunableDouble(subsystemName + "kI", controller.getI()),
                    new TunableDouble(subsystemName + "kD", controller.getD())
                  });

      if (values[0].hasChanged() || values[1].hasChanged() || values[2].hasChanged()) {
        controller.setPID(values[0].get(), values[1].get(), values[2].get());
      }
    }

    /*       public static void autoTune(PIDController controller, SparkBase motor, RelativeEncoder encoder, double defaultkP, double setpoint, double tolerance) {
        double startPos = encoder.getPosition();

        TunableDouble[] PIDArray = new TunableDouble[]{
            new TunableDouble("Auto kP", defaultkP),
            new TunableDouble("Auto kI", 0.0),
            new TunableDouble("Auto KD", 0.0)};

        TunableDouble[] lastPIDValues = PIDArray;

        controller.setTolerance(tolerance);
        double output = controller.calculate(encoder.getPosition(), setpoint);
        double percentOutput = output/12; //12 volts is battery volts
        motor.set(percentOutput);


    } */
  }
}
