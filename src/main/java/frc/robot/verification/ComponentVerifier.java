package frc.robot.verification;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonDevice;
import edu.wpi.first.util.sendable.Sendable;

import java.util.ArrayList;
import java.util.List;

public class ComponentVerifier {
    public record VerificationResult(Status status, String message, String name) {
        public enum Status {
            SUCCESS,
            FAILURE
        }
    }

    public static List<VerificationResult> verify(Verifiable verifiable) {
        List<VerificationResult> results = new ArrayList<>();
        for (Sendable comp : verifiable.components()) {
            if (comp instanceof CommonDevice device) {
                String name = String.valueOf(device.getDeviceID());

                if (comp instanceof TalonFX talon) {
                    results.add(verifyComponent(name, talon, () -> {
                        double temp = talon.getProcessorTemp().getValueAsDouble();
                        if (1 > temp || temp > 50)
                            throw new RuntimeException(name + " is reporting a temperature of " + temp + " Celsius, something is very wrong.");
                    }));
                } else if (comp instanceof CANcoder can) {
                    results.add(verifyComponent(name, can, () -> {
                        if (can.getFaultField().getValue() != 0)
                            throw new RuntimeException("Hardware error with " + name + ".");
                    }));
                }
            }
        }

        return results;
    }

    private static <T extends Sendable> VerificationResult verifyComponent(String name, T comp, Runnable function) {
        if (comp == null) return new VerificationResult(VerificationResult.Status.FAILURE, "This component detected as null.", name);
        try {
            function.run();
        } catch (RuntimeException e) {
            return new VerificationResult(VerificationResult.Status.FAILURE, e.getMessage(), name);
        }

        return new VerificationResult(VerificationResult.Status.SUCCESS, "This component is functional.", name);
    }
}