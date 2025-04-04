#ifndef STRING_VIEW_INCLUDED_
#define STRING_VIEW_INCLUDED_

#include <Arduino.h>
#include "status_or.h"

struct StringView {
  const char *start;
  const char *end;

  StringView();
  StringView(const char *s);
  StringView(const StringView &other) : start(other.start), end(other.end) {}

  int Length() const;

  bool operator==(const StringView other) const;

  void Print(Stream &s) const;
  void PrintLine(Stream &s) const;

  StringView Trimmed() const;

  void ToCString(char *s) const;

  StatusOr<float> ToFloat() const;
};

#endif  // STRING_VIEW_INCLUDED_