package frc.robot.verification;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

public class ComponentVerifier {
    public static final VerificationResult POSITIVE = new VerificationResult(VerificationResult.Status.SUCCESS, "This component is functional.");

    public record VerificationResult(Status status, String message) {
        enum Status {
            SUCCESS,
            FAILURE
        }
    }

    public static List<VerificationResult> verify(Verifiable verifiable) {
        List<VerificationResult> results = new ArrayList<>();
        for (Sendable comp : verifiable.components()) {
            if (comp instanceof TalonFX talon) {
                results.add(verifyComponent("", talon, (t, n) -> {
                    double temp = talon.getProcessorTemp().getValueAsDouble();
                    if (1 > temp || temp > 50) throw new RuntimeException(n + " is reporting a temperature of " + temp + " Celsius, something is very wrong.");
                }));
            }
            if (comp instanceof CANcoder can) {
                results.add(verifyComponent("", can, (c, n) -> {
                    if (can.getFaultField().getValue() != 0) throw new RuntimeException("Hardware error with " + n + ".");
                }));
            }
        }

        return results;
    }

    private static <T extends Sendable> VerificationResult verifyComponent(String name, T comp, BiConsumer<T, String> function) {
        if (comp == null) return new VerificationResult(VerificationResult.Status.FAILURE, name + " was detected as null.");
        try {
            function.accept(comp, name);
        } catch (RuntimeException e) {
            return new VerificationResult(VerificationResult.Status.FAILURE, e.getMessage());
        }

        return POSITIVE;
    }
}
