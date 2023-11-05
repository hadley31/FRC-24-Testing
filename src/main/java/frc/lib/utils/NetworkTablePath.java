package frc.lib.utils;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufEntry;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.util.struct.Struct;
import us.hebi.quickbuf.ProtoMessage;

public interface NetworkTablePath {
  public String getPathRoot();

  //#region Double

  public default DoubleSubscriber getDoubleSub(String name, double defaultValue, PubSubOption... options) {
    return NetworkTableInstance.getDefault().getDoubleTopic(getFullPath(name)).subscribe(defaultValue,
        options);
  }

  public default DoubleEntry getDoubleEntry(String name, double defaultValue, PubSubOption... options) {
    return NetworkTableInstance.getDefault().getDoubleTopic(getFullPath(name)).getEntry(defaultValue, options);
  }

  public default void addDoubleSubListener(Subscriber sub, Consumer<Double> consumer) {
    NetworkTableInstance.getDefault().addListener(sub, EnumSet.of(Kind.kValueAll),
        (event) -> consumer.accept(event.valueData.value.getDouble()));
  }

  //#endregion

  //#region Double Array

  public default DoubleArraySubscriber getDoubleArraySub(String name, double[] defaultValue, PubSubOption... options) {
    return NetworkTableInstance.getDefault().getDoubleArrayTopic(getFullPath(name)).subscribe(defaultValue,
        options);
  }

  public default DoubleArrayPublisher getDoubleArrayPub(String name, PubSubOption... options) {
    return NetworkTableInstance.getDefault().getDoubleArrayTopic(getFullPath(name)).publish(options);
  }

  public default DoubleArrayEntry getDoubleArrayEntry(String name, double[] defaultValue, PubSubOption... options) {
    return NetworkTableInstance.getDefault().getDoubleArrayTopic(getFullPath(name)).getEntry(defaultValue, options);
  }

  //#endregion

  //#region Protobuf

  public default <T, MessageType extends ProtoMessage<?>> ProtobufEntry<T> getProtobufTopic(String name,
      Protobuf<T, MessageType> proto, T defaultValue) {
    return NetworkTableInstance.getDefault().getProtobufTopic(getFullPath(name), proto).getEntry(defaultValue);
  }

  public default <T, MessageType extends ProtoMessage<?>> ProtobufEntry<T> getProtobufTopic(String name,
      Protobuf<T, MessageType> proto) {
    return getProtobufTopic(name, proto, null);
  }

  //#endregion

  //#region Struct

  public default <T> StructEntry<T> getStructEntry(String name,
      Struct<T> struct, T defaultValue) {
    return NetworkTableInstance.getDefault().getStructTopic(getFullPath(name), struct).getEntry(defaultValue);
  }

  public default <T> StructEntry<T> getStructEntry(String name,
      Struct<T> struct) {
    return getStructEntry(name, struct, null);
  }

  //#endregion

  public default String getFullPath(String name) {
    return "/" + getPathRoot() + "/" + name;
  }
}
