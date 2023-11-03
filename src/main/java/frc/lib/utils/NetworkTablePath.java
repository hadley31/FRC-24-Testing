package frc.lib.utils;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Subscriber;

public interface NetworkTablePath {
  public String getPathRoot();

  public default DoubleSubscriber getDoubleSub(String name, double defaultValue, PubSubOption... options) {
    return NetworkTableInstance.getDefault().getDoubleTopic(getFullPath(name)).subscribe(defaultValue,
        options);
  }

  public default DoubleEntry getDoubleEntry(String name, double defaultValue, PubSubOption... options) {
    return NetworkTableInstance.getDefault().getDoubleTopic(getFullPath(name)).getEntry(defaultValue, options);
  }

  public default void addSubListener(Subscriber sub, Consumer<Double> consumer) {
    NetworkTableInstance.getDefault().addListener(sub, EnumSet.of(Kind.kValueAll),
        (event) -> consumer.accept(event.valueData.value.getDouble()));
  }

  public default String getFullPath(String name) {
    return getPathRoot() + "/" + name;
  }
}
