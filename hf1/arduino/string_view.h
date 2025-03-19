#ifndef STRING_VIEW_INCLUDED_
#define STRING_VIEW_INCLUDED_

#include <Arduino.h>

struct StringView {
  const char *start;
  const char *end;

  StringView();
  StringView(const char *s);

  int Length() const;

  bool operator==(const StringView other) const;

  void Print(Stream &s) const;
  void PrintLine(Stream &s) const;

  StringView Trimmed() const;

  void ToCString(char *s) const;
};

#endif  // STRING_VIEW_INCLUDED_