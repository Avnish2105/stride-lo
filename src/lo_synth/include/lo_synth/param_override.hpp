// "name=value" overrides of lo::Config through the schema (CLI tools, tests).
#pragma once

#include <sstream>
#include <string>
#include <vector>

#include "lo_core/config/schema.hpp"

namespace lo::synth {

/// Apply one "name=value" override; returns false if the name is unknown or the value bad.
inline bool applyOverride(Config & c, const std::string & kv)
{
  const auto eq = kv.find('=');
  if (eq == std::string::npos) {return false;}
  struct Setter
  {
    std::string name, value;
    bool done{false}, ok{true};
    void number(const char * n, double & v, double, double, const char *)
    {
      if (name == n) {done = true; try {v = std::stod(value);} catch (...) {ok = false;}}
    }
    void integer(const char * n, int & v, int, int, const char *)
    {
      if (name == n) {done = true; try {v = std::stoi(value);} catch (...) {ok = false;}}
    }
    void boolean(const char * n, bool & v, const char *)
    {
      if (name == n) {done = true; v = (value == "true" || value == "1");}
    }
    void choice(const char * n, std::string & v, std::initializer_list<const char *>, const char *)
    {
      if (name == n) {done = true; v = value;}
    }
    void vector(const char * n, std::vector<double> & v, std::size_t, const char *)
    {
      if (name != n) {return;}
      done = true;
      v.clear();
      std::stringstream ss(value);
      std::string item;
      while (std::getline(ss, item, ',')) {
        try {v.push_back(std::stod(item));} catch (...) {ok = false;}
      }
    }
  } s{kv.substr(0, eq), kv.substr(eq + 1)};
  visitParams(c, s);
  return s.done && s.ok;
}

}  // namespace lo::synth
