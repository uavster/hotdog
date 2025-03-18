#include "string_view.h"

StringView::StringView()
  : start(nullptr), end(nullptr) {}

StringView::StringView(const char *s)
  : start(s) {
  while (*s != '\0') { ++s; }
  end = s;
}

bool StringView::Length() const {
  return end - start;
}

bool StringView::operator==(const StringView other) const {
  const char *p1 = start;
  const char *p2 = other.start;
  while (*p1 == *p2 && p1 != end && p2 != other.end) {
    ++p1;
    ++p2;
  }
  return p1 == end && p2 == other.end;
}

void StringView::Print(Stream &s) const {
  const char *p = start;
  while (p != end) {
    s.write(*p);
    ++p;
  }
}

void StringView::PrintLine(Stream &s) const {
  Print(s);
  s.println();
}