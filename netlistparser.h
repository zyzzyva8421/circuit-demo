#ifndef NETLIST_PARSER_H
#define NETLIST_PARSER_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>

// Use simple string for now, could be an enum
using NodeType = std::string;

struct Port {
    std::string name;
    std::string direction; // "input", "output", "inout"
    int width = 1;         // Bus width
};

struct Instance {
    std::string type;      // The module name being instantiated (e.g., "nand2")
    std::string name;      // The instance name (e.g., "u1")
    std::map<std::string, std::string> portMap; // .A(n1) -> port "A" connects to net "n1"
    
    // Position hint if available (not standard Verilog)
    double x = 0, y = 0;
};

struct Net {
    std::string name;
    int width = 1;
};

struct Module {
    std::string name;
    std::vector<Port> ports;
    std::vector<Net> wires;
    std::vector<Instance> instances;
    // For simple assign statements, treat as generic combinational logic
    // assign Y = ~(A & B) -> Instance type="Assign", ports={"Y": "Y", "expr": "~(A & B)"}
    std::vector<std::string> assigns; 
};

struct Netlist {
    std::map<std::string, Module> modules;
    std::string topModule;
};

class NetlistParser {
public:
    static Netlist parse(const std::string& filename) {
        Netlist netlist;
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Cannot open file: " << filename << std::endl;
            return netlist;
        }

        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        // Remove comments
        content = std::regex_replace(content, std::regex("//.*"), ""); 
        content = std::regex_replace(content, std::regex("/\\*[\\s\\S]*?\\*/"), "");

        // Manual parsing instead of regex to handle large files (avoid stack overflow)
        size_t pos = 0;
        while ((pos = content.find("module ", pos)) != std::string::npos) {
            size_t endModule = content.find("endmodule", pos);
            if (endModule == std::string::npos) break;
            
            // Extract Name
            size_t nameStart = content.find_first_not_of(" \t\r\n", pos + 6); // "module" is 6 chars
            if (nameStart == std::string::npos) break;
            size_t nameEnd = content.find_first_of(" \t\r\n;(#", nameStart);
            if (nameEnd == std::string::npos) nameEnd = endModule;
            
            std::string moduleName = content.substr(nameStart, nameEnd - nameStart);
            
            // Find Ports: (...)
            // Simple logic: seek first '(' after name, seek ');' after that.
            // Assumption: no nested parenthesis in ports or params for this simple parser
            size_t portsStart = content.find('(', nameEnd);
            size_t portsEnd = std::string::npos;
            
            if (portsStart != std::string::npos && portsStart < endModule) {
                 // Check for params #(...)
                 // If there is a '#' before the found '(', implies parameters.
                 // We need to skip them to find real ports.
                 size_t paramIndicator = content.find('#', nameEnd);
                 if (paramIndicator != std::string::npos && paramIndicator < portsStart) {
                      size_t paramEnd = content.find(')', portsStart); // End of params
                      if (paramEnd != std::string::npos) {
                           portsStart = content.find('(', paramEnd); // Real ports start
                      }
                 }
                 
                 if (portsStart != std::string::npos && portsStart < endModule) {
                      portsEnd = content.find(");", portsStart);
                 }
            }
            
            std::string portsStr = "";
            std::string body = "";
            
            if (portsStart != std::string::npos && portsEnd != std::string::npos && portsEnd < endModule) {
                 portsStr = content.substr(portsStart + 1, portsEnd - portsStart - 1);
                 body = content.substr(portsEnd + 2, endModule - (portsEnd + 2));
            } else {
                 // Case: module name (...); body endmodule
                 // Or no ports: module name; body endmodule
                 size_t semi = content.find(';', nameEnd);
                 if (semi != std::string::npos && semi < endModule) {
                     body = content.substr(semi + 1, endModule - (semi + 1));
                 }
            }

            Module module;
            module.name = moduleName;
            parsePorts(module, portsStr); 
            parseBody(module, body);
            
            netlist.modules[module.name] = module;
            if (netlist.topModule.empty()) netlist.topModule = module.name; 
            
            pos = endModule + 9;
        }

        return netlist;
    }

private:
   static void parsePorts(Module& module, const std::string& portsStr) {
        // Simplified ANSI-style port parser
        std::regex tokenRegex("([\\w\\[\\]:]+)");
        auto begin = std::sregex_iterator(portsStr.begin(), portsStr.end(), tokenRegex);
        auto end = std::sregex_iterator();
        
        std::string currentDir = "";
        
        for (std::sregex_iterator i = begin; i != end; ++i) {
             std::string token = (*i).str();
             
             if (token == "input" || token == "output" || token == "inout") {
                 currentDir = token;
                 continue;
             }
             if (token == "wire" || token == "reg") continue;
             if (token.find('[') != std::string::npos) continue; // Skip ranges like [31:0]
             
             // Verify it's a valid identifier (not a number/range)
             if (!token.empty() && std::isdigit(token[0])) continue;
             Port port;
             port.name = token;
             port.direction = currentDir;
             module.ports.push_back(port);
        }
   }

   static void parseBody(Module& module, const std::string& body) {
       // Identify input/output declarations to update ports
       std::regex dirRegex("(input|output|inout)\\s+(?:wire|reg)?\\s*(?:\\[[^]]+\\])?\\s*([^;]+);");
       auto dirBegin = std::sregex_iterator(body.begin(), body.end(), dirRegex);
       for (auto i = dirBegin; i != std::sregex_iterator(); ++i) {
           std::string dir = (*i)[1];
           std::string names = (*i)[2];
           
           // Split names
           std::regex nameRegex("(\\w+)"); // Ignoring ranges for mapping for now
           auto nameIt = std::sregex_iterator(names.begin(), names.end(), nameRegex);
           for (; nameIt != std::sregex_iterator(); ++nameIt) {
               std::string n = (*nameIt).str();
               // Skip numeric values from bit ranges like [31:0]
               if (!n.empty() && std::isdigit(n[0])) continue;
               // check explicitly if it's already in ports
               bool found = false;
               for(auto& p : module.ports) {
                    if (p.name == n) {
                        p.direction = dir;
                        found = true;
                    }
               }
               if(!found) {
                   // ANSI style ports might be declared inline, but here we found a separate decl
                   Port p; p.name = n; p.direction = dir;
                   module.ports.push_back(p);
               }
           }
       }

       // Parse instances: type name ( .port(net), ... );
       // Simplification: assume 1 instance per statement
       // Regex for instance: (\w+)\s+(\w+)\s*\(([^;]+)\)\s*;
       // But wait, "assign" is special
       
       std::istringstream stream(body);
       std::string statement;
       while (std::getline(stream, statement, ';')) {
            // Remove newlines and tabs to simplify parsing
            std::replace(statement.begin(), statement.end(), '\n', ' ');
            std::replace(statement.begin(), statement.end(), '\r', ' ');
            std::replace(statement.begin(), statement.end(), '\t', ' ');

            // Cleanup whitespace
            statement = std::regex_replace(statement, std::regex("^\\s+|\\s+$"), "");
            if (statement.empty()) continue;

            if (statement.rfind("assign", 0) == 0) {
                 module.assigns.push_back(statement);
                 continue;
            }
            
            // Should be an instance
            // type name ( connections )
            std::regex instRegex("(\\w+)\\s+(\\w+)\\s*\\((.*)\\)");
            std::smatch match;
            if (std::regex_search(statement, match, instRegex)) {
                std::string type = match[1];
                // Skip Verilog/SystemVerilog keywords that aren't modules
                if (type == "module" || type == "input" || type == "output" || type == "inout" || 
                    type == "wire" || type == "reg" || type == "always" || type == "assign" ||
                    type == "if" || type == "else" || type == "case" || type == "for" || type == "while" ||
                    type == "begin" || type == "end" || type == "initial" || type == "generate" ||
                    type == "endgenerate" || type == "endcase" || type == "endt" || type == "task" ||
                    type == "function" || type == "endfunction" || type == "endtask") continue;

                Instance inst;
                inst.type = type;
                inst.name = match[2];
                std::string conns = match[3];
                
                // Parse connections .port(net)
                std::regex connRegex("\\.(\\w+)\\s*\\(([^)]*)\\)");
                auto connBegin = std::sregex_iterator(conns.begin(), conns.end(), connRegex);
                for (auto i = connBegin; i != std::sregex_iterator(); ++i) {
                     inst.portMap[(*i)[1]] = (*i)[2];
                }
                
                module.instances.push_back(inst);
            }
       }
   }
};

#endif // NETLIST_PARSER_H
