#pragma once

#include <vector>
#include <string>

class SubProtocol
{
public:
    unsigned char value;
    const char* name;
    SubProtocol(unsigned char newValue, const char* newName) : value(newValue), name(newName){};
};


class Protocol : public SubProtocol
{
public:
    std::vector<SubProtocol> subProtocolList;
    Protocol(unsigned char newValue, const char* newName, std::vector<SubProtocol> newSubProtocolList) : SubProtocol(newValue, newName), subProtocolList(newSubProtocolList){};
};

