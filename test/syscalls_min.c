#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// Minimal heap via linker symbol '_end'
extern char _end; // end of BSS
static char *heap_end;

int _close(int fd) {
  (void)fd;
  errno = ENOSYS;
  return -1;
}

int _fstat(int fd, struct stat *st) {
  (void)fd;
  if (st) {
    st->st_mode = S_IFCHR;
  }
  return 0;
}

int _isatty(int fd) {
  (void)fd;
  return 1;
}

off_t _lseek(int fd, off_t off, int w) {
  (void)fd;
  (void)off;
  (void)w;
  errno = ENOSYS;
  return -1;
}

int _read(int fd, void *buf, size_t n) {
  (void)fd;
  (void)buf;
  (void)n;
  errno = ENOSYS;
  return -1;
}

int _write(int fd, const void *buf, size_t n) {
  (void)fd;
  (void)buf;
  return (int)n;
}

void *_sbrk(ptrdiff_t incr) {
  if (!heap_end) {
    heap_end = &_end;
  }
  char *prev = heap_end;
  heap_end += incr;
  return prev;
}
