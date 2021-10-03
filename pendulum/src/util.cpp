#include <vector>
#include <string>

int idxOf(const std::vector <std::string> a, const std::string &b) {
  for (int i = 0; i < a.size(); i++) {
    if (a[i].compare(b) == 0)
      return i;
  }
  return -1;
}