package test;

import java.io.*;

public class FilterTest {
  private static byte[]   coeffloi = { 64,  45,   0, -45, -64, -45,   0,  45};
  private static byte[]   coeffloq = {  0,  45,  64,  45,   0, -45, -64, -45};
  private static byte[]   coeffhii = { 64,   8, -62, -24,  55,  39, -45, -51};
  private static byte[]   coeffhiq = {  0,  63,  17, -59, -32,  51,  45, -39};

  private static int demodulate (byte[] data, int idx) {
    short outloi = 0, outloq = 0, outhii = 0, outhiq = 0;
    for (int ii = 0; ii < 8; ii++) {
      byte sample = data[ii + idx];
      outloi += sample * coeffloi[ii];
      outloq += sample * coeffloq[ii];
      outhii += sample * coeffhii[ii];
      outhiq += sample * coeffhiq[ii];
    }
    return (outhii >> 8) * (outhii >> 8) + (outhiq >> 8) * (outhiq >> 8) -
           (outloi >> 8) * (outloi >> 8) - (outloq >> 8) * (outloq >> 8);
  }

  private static int demodulatelo (byte[] data, int idx) {
    short outloi = 0, outloq = 0;
    for (int ii = 0; ii < 8; ii++) {
      byte sample = data[ii + idx];
      outloi += sample * coeffloi[ii];
      outloq += sample * coeffloq[ii];
    }
    System.out.format("%d %d\n", outloi, outloq);
    return 0 -
           (outloi >> 8) * (outloi >> 8) - (outloq >> 8) * (outloq >> 8);
  }

  // 1200 hz cos
  private static byte[] test1 = { 64,  45,   0, -45, -64, -45,   0,  45};
  // 1200 hz 90 offset cos
  private static byte[] test2 = { 45,  64,  45,   0, -45, -64, -45,   0};
  // 1200 hz ~50 offset cos
  private static byte[] test3 = { 19,  57,  61,  30, -19, -57, -61, -30};
  // 2200 hz cos
  private static byte[] test4 = { 64,   8, -62, -24,  55,  39, -45, -51};
  // transition from 1200->2200hz
  private static byte[] trans0 = {45, 64, 45, 0, -45, -64, -45, 0};
  private static byte[] trans1 = {64, 45, 0, -45, -64, -45, 0, 63};
  private static byte[] trans2 = {45, 0, -45, -64, -45, 0, 63, 17};
  private static byte[] trans3 = {0, -45, -64, -45, 0, 63, 17, -59};
  private static byte[] trans4 = {-45, -64, -45, 0, 63, 17, -59, -32};
  private static byte[] trans5 = {-64, -45, 0, 63, 17, -59, -32, 51};
  private static byte[] trans6 = {-45, 0, 63, 17, -59, -32, 51, 45};
  private static byte[] trans7 = {0, 63, 17, -59, -32, 51, 45, -39};


  public static void main(String[] args) {
    System.out.format("test1: %d\n", demodulate(test1, 0));
    System.out.format("test2: %d\n", demodulate(test2, 0));
    System.out.format("test3: %d\n", demodulate(test3, 0));
    System.out.format("test4: %d\n", demodulate(test4, 0));
    System.out.format("test3 lo: %d\n", demodulatelo(test3, 0));
    System.out.format("test4 lo: %d\n", demodulatelo(test4, 0));
    System.out.format("trans: %d\n", demodulate(trans0, 0));
    System.out.format("trans: %d\n", demodulate(trans1, 0));
    System.out.format("trans: %d\n", demodulate(trans2, 0));
    System.out.format("trans: %d\n", demodulate(trans3, 0));
    System.out.format("trans: %d\n", demodulate(trans4, 0));
    System.out.format("trans: %d\n", demodulate(trans5, 0));
    System.out.format("trans: %d\n", demodulate(trans6, 0));
    System.out.format("trans: %d\n", demodulate(trans7, 0));
  }
}