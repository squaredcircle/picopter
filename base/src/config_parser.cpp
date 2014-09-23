#include <config_parser.h>

ConfigParser::ParamMap::ParamMap() {}
ConfigParser::ParamMap::ParamMap(const ParamMap& orig) {}
ConfigParser::ParamMap::~ParamMap() {}

void ConfigParser::ParamMap::insert(std::string label, int* ptr) {
    intMap.insert(std::pair<std::string, int*>(label, ptr));
    typeMap.insert(std::pair<std::string, char>(label, 'i'));
}

void ConfigParser::ParamMap::insert(std::string label, double* ptr) {
    doubleMap.insert(std::pair<std::string, double*>(label, ptr));
    typeMap.insert(std::pair<std::string, char>(label, 'd'));
}

void ConfigParser::ParamMap::insert(std::string label, char* ptr) {
    charMap.insert(std::pair<std::string, char*>(label, ptr));
    typeMap.insert(std::pair<std::string, char>(label, 'c'));
}

int* ConfigParser::ParamMap::getInt(std::string label) {
    return intMap[label];
}

double* ConfigParser::ParamMap::getDouble(std::string label) {
    return doubleMap[label];
}

char* ConfigParser::ParamMap::getChar(std::string label) {
    return charMap[label];
}

char ConfigParser::ParamMap::getType(std::string label) {
    if(typeMap.find(label) != typeMap.end()) {
        return typeMap[label];
    } else {
        return '_';
    }
}

std::string ConfigParser::ParamMap::listParameters() {
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


int ConfigParser::loadParameters(std::string className, ParamMap *parameters, std::string fileName) {
    
    
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
        //std::cout << "LOAD:\tgot line:" << line << std::endl;
        if(line.empty()) continue;
        if(line.at(0) == ' ' || line.at(0) == '\t') continue;
        if(line.at(0) == '#') continue;
        if(line.at(0) != '%') {
			configFile.close();
			return -2;
		}
        char ch = line.at(1);
        switch(ch) {
            case 'T':
                iss.str(line.substr(2, std::string::npos));
                iss.clear();
                iss >> std::ws >> word;
                //std::cout << "LOAD:\tgot title:" << word << std::endl;
                if(word.compare(className) == 0) {
                    inTable = true;
                } else {
                    inTable = false;
                }
                break;
                
            case 'F':
                if(inTable) {
                    //std::cout << "LOAD:\tup to f:" << std::endl;
                    iss.str(line.substr(2, std::string::npos));
                    iss.clear();
                    while(!iss.str().empty()) {
                        iss >> std::ws >> word;
                        //std::cout << "LOAD:\tword:" << word << std::endl;
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
                        //std::cout << "LOAD:\tfields:" << fields.front() << std::endl;
                        char type = parameters->getType(fields.front());
                        //std::cout << "LOAD:\ttype:" << type << std::endl;
                        switch(type) {
                            case 'i':
                                intPtr = parameters->getInt(fields.front());
                                //std::cout << "LOAD:\tintPtr:" << *intPtr << std::endl;
                                iss >> std::ws >> *intPtr;
                                //std::cout << "LOAD:\tintPtr:" << *intPtr << std::endl;
                                std::cout << "LOAD:\t" << fields.front() << " = " << *intPtr << std::endl;
                                break;
                            case 'd':
                                doublePtr = parameters->getDouble(fields.front());
                                //std::cout << "LOAD:\tdoublePtr:" << *doublePtr << std::endl;
                                iss >> std::ws >> *doublePtr;
                                //std::cout << "LOAD:\tdoublePtr:" << *doublePtr << std::endl;
                                std::cout << "LOAD:\t" << fields.front() << " = " << *doublePtr << std::endl;
                                break;
                            case 'c':
                                charPtr = parameters->getChar(fields.front());
                                //std::cout << "LOAD:\tcharPtr:" << *charPtr << std::endl;
                                iss >> std::ws >> *charPtr;
                                //std::cout << "LOAD:\tcharPtr:" << *charPtr << std::endl;
                                std::cout << "LOAD:\t" << fields.front() << " = " << *charPtr << std::endl;
                                break;
                            default:
                                std::cout << "Error loading parameters: field "<< fields.front() << " is not a variable" << std::endl;
                                configFile.close();
                                return -4;
                                break;
                        }
                        fields.pop();
                    }
                }
                break;
                
            case 'E':
                if(!fields.empty()) {
					configFile.close();
                    return -5;
                } else {
					configFile.close();
                    return 0;
                }
                break;
                
            default:
				configFile.close();
                return -3;
                break;
            
                
        }
    }
    configFile.close();
    return 0;
}

std::string ConfigParser::printParameterParserError(int code) {
    if(code == 0)           return "No error.";
    else if(code == -1)     return "No config file found.";
    else if(code == -2)     return "Not a recognised line start.";
    else if(code == -3)     return "End of file reached before %e token.";
    else if(code == -4)     return "Field not a variable.";
    else if(code == -4)     return "Row not complete.";
    else                    return "Unknown error code";
}
