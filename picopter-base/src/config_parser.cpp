/*
*   Config Parser
*   Authour:   Bax
*   Date:       8-9-2014
*   Version:    1.0
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

ParamMap::ParamMap() {}
ParamMap::ParamMap(const ParamMap& orig) {}
ParamMap::~ParamMap() {}

void ParamMap::insert(std::string label, int* ptr) {
    intMap.insert(std::pair<std::string, int*>(label, ptr));
    typeMap.insert(std::pair<std::string, char>(label, 'i'));
}

void ParamMap::insert(std::string label, double* ptr) {
    doubleMap.insert(std::pair<std::string, double*>(label, ptr));
    typeMap.insert(std::pair<std::string, char>(label, 'd'));
}

void ParamMap::insert(std::string label, char* ptr) {
    charMap.insert(std::pair<std::string, char*>(label, ptr));
    typeMap.insert(std::pair<std::string, char>(label, 'c'));
}

int* ParamMap::getInt(std::string label) {
    return intMap[label];
}

double* ParamMap::getDouble(std::string label) {
    return doubleMap[label];
}

char* ParamMap::getChar(std::string label) {
    return charMap[label];
}

char ParamMap::getType(std::string label) {
    if(typeMap.find(label) != typeMap.end()) {
        return typeMap[label];
    } else {
        return '_';
    }
}

std::string ParamMap::listParameters() {
    std::stringstream ss;
    ParamMap_type::reverse_iterator rit;
    for(rit = typeMap.rbegin(); rit!=typeMap.rend(); ++rit) {
        ss << rit->first << " = ";
        switch(rit->second) {
            case 'i':
                ss << *(intMap[rit->first]) << ", ";
                break;
            case 'd':
                ss << *(doubleMap[rit->first]) << ", ";
                break;
            case 'c':
                ss << *(charMap[rit->first]) << ", ";
                break;
            default:
                ss << "ERROR" << ", ";
                break;
        }
    }
    return ss.str();
}



int loadParameters(std::string className, ParamMap *parameters, std::string fileName);
std::string printConfigParserError(int);

int loadParameters(std::string className, ParamMap *parameters, std::string fileName) {
    
    
    std::ifstream configFile(fileName.c_str(), std::ifstream::in);
    if(!configFile) {
        std::cout << "Error opening file" << std::endl;
        return -1;
    }
    
    std::queue<std::string> fields;
    
    std::string line;
    std::string word;
    bool inTable = false;
    
    int* intPtr;
    double* doublePtr;
    char* charPtr;
    
    std::istringstream iss;
    
    while(getline(configFile, line)) {
        std::cout << "LOAD:\tgot line:" << line << std::endl;
        if(line.at(0) == '#') continue;
        if(line.at(0) != '%') return -2;
        char ch = line.at(1);
        switch(ch) {
            case 'T':
                iss.str(line.substr(2, std::string::npos));
                iss.clear();
                iss >> std::ws >> word;
                std::cout << "LOAD:\tgot title:" << word << std::endl;
                if(word.compare(className) == 0) {
                    inTable = true;
                } else {
                    inTable = false;
                }
                break;
                
            case 'F':
                if(inTable) {
                    std::cout << "LOAD:\tup to f:" << std::endl;
                    iss.str(line.substr(2, std::string::npos));
                    iss.clear();
                    while(!iss.str().empty()) {
                        iss >> std::ws >> word;
                        std::cout << "LOAD:\tword:" << word << std::endl;
                        fields.push(word);
                        iss.str(iss.str().substr(iss.str().find(word) + word.length(), std::string::npos));
                        //Haters gonna hate.
                    }
                    
                }
                break;
                
            case 'R':
                if(inTable) {
                    iss.str(line.substr(2, std::string::npos));
                    iss.clear();
                    while(!iss.str().empty()) {                        
                        if(fields.empty()) {
                            break;
                        }
                        std::cout << "LOAD:\tfields:" << fields.front() << std::endl;
                        char type = parameters->getType(fields.front());
                        std::cout << "LOAD:\ttype:" << type << std::endl;
                        switch(type) {
                            case 'i':
                                intPtr = parameters->getInt(fields.front());
                                std::cout << "LOAD:\tintPtr:" << *intPtr << std::endl;
                                iss >> std::ws >> *intPtr;
                                std::cout << "LOAD:\tintPtr:" << *intPtr << std::endl;
                                break;
                            case 'd':
                                doublePtr = parameters->getDouble(fields.front());
                                std::cout << "LOAD:\tdoublePtr:" << *doublePtr << std::endl;
                                iss >> std::ws >> *doublePtr;
                                std::cout << "LOAD:\tdoublePtr:" << *doublePtr << std::endl;
                                break;
                            case 'c':
                                charPtr = parameters->getChar(fields.front());
                                std::cout << "LOAD:\tcharPtr:" << *charPtr << std::endl;
                                iss >> std::ws >> *charPtr;
                                std::cout << "LOAD:\tcharPtr:" << *charPtr << std::endl;
                                break;
                            default:
                                std::cout << "Error loading parameters: field "<< fields.front() << " is not a variable" << std::endl;
                                return -4;
                                break;
                        }
                        fields.pop();
                    }
                }
                break;
                
            case 'E':
                if(!fields.empty()) {
                    return -5;
                } else {
                    return 0;
                }
                break;
                
            default:
                return -3;
                break;
            
                
        }
    }
    return 0;
}

std::string printParameterParserError(int code) {
    if(code == 0)           return "No error.";
    else if(code == -1)     return "No config file found.";
    else if(code == -2)     return "Not a recognised line start.";
    else if(code == -3)     return "End of file reached before %e token.";
    else if(code == -4)     return "Field not a variable.";
    else if(code == -4)     return "Row not complete.";
    else                    return "Unknown error code";
}

int main(int argc, char* argv[]) {
    int x = 7;
    double catfish = 0;
    char z = '0';
    int a = 42;
    
    ParamMap parameters;
    parameters.insert("x", &x);
    parameters.insert("catfish", &catfish);
    parameters.insert("z", &z);
    parameters.insert("a", &a);
    
    std::cout << loadParameters("config", &parameters, "./config/config.txt") << std::endl;
    std::cout << parameters.listParameters() << std::endl;
    
    return 0;
}
