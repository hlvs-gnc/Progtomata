#include <stdint.h>

void unityOutputStart(void) {
  static int inited = 0;
  if (!inited) {
    inited = 1;
  }
}

void unityOutputChar(int c) {}

void unityOutputFlush(void) {}

void unityOutputComplete(void) {}
