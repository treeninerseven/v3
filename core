#pragma once
#include <stddef.h>
#include <stdint.h>

namespace AtTerminal {
constexpr int hex(char c) {
  return c >= '0' && c <= '9' ? c - '0' :
         c >= 'a' && c <= 'f' ? c - 'a' + 10 :
         c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
}

// Decode the entire line before transmitting anything. Always append one CR.
// Returns zero for invalid escapes or insufficient space; no partial TX occurs.
constexpr size_t decode(const char *line, size_t length, uint8_t *out, size_t capacity) {
  size_t n = 0;
  for (size_t i = 0; i < length; ++i) {
    uint8_t c = static_cast<uint8_t>(line[i]);
    if (c == '\\') {
      if (++i >= length) return 0;
      switch (line[i]) {
        case '\\': c = '\\'; break;
        case 't': c = '\t'; break;
        case 'r': c = '\r'; break;
        case 'n': c = '\n'; break;
        case 'x': {
          if (i + 2 >= length) return 0;
          const int high = hex(line[i + 1]), low = hex(line[i + 2]);
          if (high < 0 || low < 0) return 0;
          c = static_cast<uint8_t>((high << 4) | low); i += 2;
          break;
        }
        default: return 0;
      }
    }
    if (n >= capacity) return 0;
    out[n++] = c;
  }
  if (n >= capacity) return 0;
  out[n++] = '\r';
  return n;
}
} // namespace AtTerminal
