package frc.robot.verification;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonDevice;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.List;

public class ComponentVerifier {
    public record VerificationResult(Status status, String message, String name, String subsystem) {
        public enum Status {
            SUCCESS(Color.kGreen, "No errors were detected."),
            WARNING(Color.kOrange, "At least one component had a warning."),
            FAILURE(Color.kRed, "At least one component had an error.");

            public final Color color;
            public final String message;
            Status(Color color, String message) {
                this.color = color;
                this.message = message;
            }
        }
    }

    public static List<VerificationResult> verify(Verifiable verifiable) {
        List<VerificationResult> results = new ArrayList<>();
        for (Sendable comp : verifiable.components()) {
            if (comp instanceof CommonDevice device) {
                String name = String.valueOf(device.getDeviceID());

                if (comp instanceof TalonFX talon) {
                    results.add(verifyComponent(name, verifiable.subsystem(), talon, () -> {
                        double temp = talon.getProcessorTemp().getValueAsDouble();
                        if (1 > temp || temp > 50)
                            throw new RuntimeException(name + " is reporting a temperature of " + temp + " Celsius, something is very wrong.", new Throwable("FAILURE"));
                    }));
                } else if (comp instanceof CANcoder can) {
                    results.add(verifyComponent(name, verifiable.subsystem(), can, () -> {
                        if (can.getFaultField().getValue() != 0)
                            throw new RuntimeException("Hardware error with " + name + ".", new Throwable("FAILURE"));
                    }));
                } else if (comp instanceof Pigeon2 pigeon) {
                    results.add(verifyComponent(name, verifiable.subsystem(), pigeon, () -> {
                        Measure<AngleUnit> degrees = pigeon.getYaw().getValue();
                        if (degrees.isNear(Units.Degrees.of(0), 1e-5))
                            throw new RuntimeException("Suspicious Pigeon yaw: " + degrees.toShortString(), new Throwable("FAILURE"));
                        if (!pigeon.isConnected() || pigeon.getFaultField().getValue() != 0)
                            throw new RuntimeException("The Pigeon reports a hardware failure.", new Throwable("FAILURE"));
                    }));
                }
            }
        }

        return results;
    }

    public static <T> VerificationResult verifyComponent(String name, String subsystem, T comp, Runnable function) {
        if (comp == null) return new VerificationResult(VerificationResult.Status.FAILURE, "This component is detected as null.", name, subsystem);
        try {
            function.run();
        } catch (RuntimeException e) {
            return new VerificationResult(VerificationResult.Status.valueOf(e.getCause().getMessage()), e.getMessage(), name, subsystem);
        }

        return new VerificationResult(VerificationResult.Status.SUCCESS, "This component is functional.", name, subsystem);
    }
}