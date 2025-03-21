#include "string_view.h"

StringView::StringView()
  : start(nullptr), end(nullptr) {}

StringView::StringView(const char *s)
  : start(s) {
  while (*s != '\0') { ++s; }
  end = s;
}

int StringView::Length() const {
  return end - start;
}

bool StringView::operator==(const StringView other) const {
  if (Length() == 0 && other.Length() == 0) { return true; }
  if (Length() == 0 || other.Length() == 0) { return false; }
  const char *p1 = start;
  const char *p2 = other.start;
  while (p1 < end && p2 < other.end) {
    if (*p1 != *p2) {
      return false;
    }
    ++p1;
    ++p2;
  }
  return true;
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

StringView StringView::Trimmed() const {
  if (start == end) { return StringView(*this); }
  StringView result = *this;
  while (*result.start == ' ' && result.start < result.end) { ++result.start; }
  while (*(result.end - 1) == ' ' && result.end > result.start) { --result.end; }
  return result;
}

void StringView::ToCString(char *s) const {
  const char *p = start;
  while (p < end) {
    *s = *p;
    ++p;
    ++s;
  }
  *s = '\0';
}

StatusOr<float> StringView::ToFloat() const {
  char tmp[Length() + 1];
  ToCString(tmp);
  float value;
  if (sscanf(tmp, "%f", &value) != 1) {
    return Status::kMalformedError;
  }
  return value;
}