#ifndef __PARSER_H__
#define __PARSER_H__

#include <string>

namespace kuam
{

class Parser
{
public:
    Parser();
    virtual ~Parser();
private:
    bool ReadFile(std::string file_path);

};


}
#endif // __PARSER_H__