#include "parser.h"

Parser::Parser(std::string s, std::string delimiter)
{
    this->s = s;
    this->delimiter = delimiter;
}

std::vector<std::float_t> Parser::array()
{
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::float_t> res;

    while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (atof(token.c_str()));
    }

    res.push_back (atof(s.substr (pos_start).c_str()));
    return res;
}