/*
 *   Config Parser
 *   Authour:   Bax
 *   Date:       9-9-2014
 *   Version:    1.1
 *
 *   Parses config files in fielded, tab-delimeted format.
 */



#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <utility>
#include <map>
#include <queue>
#include <iterator>

namespace ConfigParser {
    typedef std::map<std::string, int*> ParamMap_int;
    typedef std::map<std::string, double*> ParamMap_double;
    typedef std::map<std::string, char*> ParamMap_char;
    typedef std::map<std::string, char> ParamMap_type;

    class ParamMap {
    public:
        ParamMap(void);
        ParamMap(const ParamMap&);
        virtual ~ParamMap(void);
        
        void insert(std::string, int*);
        void insert(std::string, double*);
        void insert(std::string, char*);
        
        int* getInt(std::string);
        double* getDouble(std::string);
        char* getChar(std::string);
        char getType(std::string);
        
        std::string listParameters(void);
        
    private:
        ParamMap_int intMap;
        ParamMap_double doubleMap;
        ParamMap_char charMap;
        ParamMap_type typeMap;
    };
    
    int loadParameters(std::string className, ParamMap *parameters, std::string fileName);
    std::string printConfigParserError(int);
    std::string printParameterParserError(int);
}