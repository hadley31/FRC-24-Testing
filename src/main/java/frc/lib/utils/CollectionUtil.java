package frc.lib.utils;

import java.util.function.Supplier;

public class CollectionUtil {
  public static <T> T[] fillArray(T[] array, Supplier<T> supplier) {
    for (int i = 0; i < array.length; i++) {
      array[i] = supplier.get();
    }
    return array;
  }
}
