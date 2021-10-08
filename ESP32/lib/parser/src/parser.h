#ifndef __parser_h
#define __parser_h
#include "Arduino.h"
//   Parser p = Parser(c.c_str(),delimiter.c_str());
//   std::vector<std::float_t> myvector = p.array();
//   for (auto it = myvector.begin(); it != myvector.end(); ++it){
//     std::float_t val = *it;
//     Serial.println(val);
//   }
class Parser
{
private:
    std::string s;
    std::string delimiter;

public:
    Parser(std::string s, std::string delimiter);
    std::vector<std::float_t> array();
};

#endif