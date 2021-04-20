#include <aruco_tracking/parser.h>
#include <fstream>
#include <iostream>

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

Parser::Parser()
{

}

Parser::~Parser()
{}

bool Parser::ReadFile(std::string file_path)
{
	std::string test_path = "test.txt";

	// write File
	std::ofstream writeFile(test_path.data());
	if( writeFile.is_open() ){
		writeFile << "Hello World!\n";
		writeFile << "This is C++ File Contents.\n";
		writeFile.close();
	}
    else{
        return false;
    }

	// read File
	std::ifstream openFile(test_path.data());
	if( openFile.is_open() ){
		std::string line;
		while(getline(openFile, line)){
			std::cout << line << std::endl;
		}
		openFile.close();
	}
    else{
        return false;
    }

    return true;    
}


int main()
{
    Parser parser;

	std::cout << "test" << std::endl;
}

}
