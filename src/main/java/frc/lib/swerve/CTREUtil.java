package frc.lib.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

public class CTREUtil {
  public static final void attemptUntilSuccess(Supplier<StatusCode> action) {
    StatusCode statusCode;
    do {
      statusCode = action.get();
    } while (statusCode != StatusCode.OK);
  }
}
