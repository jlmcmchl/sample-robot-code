package net.teamrush27.frc2019.util;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.util.concurrent.ConcurrentLinkedDeque;

/**
 * Writes data to a CSV file
 */
public class ReflectingCSVWriter<T> {
  ConcurrentLinkedDeque<T> mLinesToWrite = new ConcurrentLinkedDeque<>();
  PrintWriter mOutput = null;
  Field[] mFields;
  boolean wrote_header = false;

  public ReflectingCSVWriter(String fileName, Class<T> typeClass) {
    mFields = typeClass.getFields();
    try {
      mOutput = new PrintWriter(fileName);
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
  }

  public void add(T value) {
    mLinesToWrite.add(value);
  }

  private String valToHeader(T value) {
    // Write field names.
    StringBuffer line = new StringBuffer();
    for (Field field : mFields) {

      if (line.length() != 0) {
        line.append(", ");
      }
      if (CSVWritable.class.isAssignableFrom(field.getType())) {
        try {
          line.append(((CSVWritable) field.get(value)).header(field.getName()));
        } catch (IllegalAccessException e) {
          e.printStackTrace();
        }
      } else {
        line.append(field.getName());
      }
    }
    return line.toString();
  }

  private String valToString(T value) {
    StringBuffer line = new StringBuffer();
    for (Field field : mFields) {
      if (line.length() != 0) {
        line.append(", ");
      }
      try {
        if (CSVWritable.class.isAssignableFrom(field.getType())) {
          line.append(((CSVWritable) field.get(value)).toCSV());
        } else {
          line.append(field.get(value).toString());
        }
      } catch (IllegalArgumentException e) {
        e.printStackTrace();
      } catch (IllegalAccessException e) {
        e.printStackTrace();
      }
    }

    return line.toString();
  }

  protected synchronized void writeLine(String line) {
    if (mOutput != null) {
      mOutput.println(line);
    }
  }

  // Call this periodically from any thread to write to disk.
  public void write() {
    while (true) {
      T val = mLinesToWrite.pollFirst();
      if (val == null) {
        break;
      }
      if (!wrote_header) {
        writeLine(valToHeader(val));
        wrote_header = true;
      }
      writeLine(valToString(val));
    }
  }

  public synchronized void flush() {
    if (mOutput != null) {
      write();
      mOutput.flush();
    }
  }
}
