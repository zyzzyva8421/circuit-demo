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
#include <cctype>
#include <cstring>

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

        // Remove comments using a manual state machine (avoids std::regex stack overflow on large files)
        content = stripComments(content);

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

        // Infer top module: prefer modules that are not instantiated by any other module.
        // This avoids defaulting to the first module in file order (often a leaf macro).
        if (!netlist.modules.empty()) {
            std::set<std::string> instantiated;
            for (const auto& mkv : netlist.modules) {
                for (const auto& inst : mkv.second.instances) {
                    if (netlist.modules.count(inst.type)) {
                        instantiated.insert(inst.type);
                    }
                }
            }

            std::vector<std::string> roots;
            for (const auto& mkv : netlist.modules) {
                if (!instantiated.count(mkv.first)) {
                    roots.push_back(mkv.first);
                }
            }

            auto scoreTopName = [](const std::string& name) {
                std::string s = name;
                std::transform(s.begin(), s.end(), s.begin(), ::tolower);
                int score = 0;
                if (s == "top") score += 100;
                if (s.find("top") != std::string::npos) score += 20;
                if (s.find("chip") != std::string::npos) score += 15;
                if (s.find("soc") != std::string::npos) score += 15;
                if (s.find("core") != std::string::npos) score += 10;
                if (s.find("wrapper") != std::string::npos) score += 8;
                // Boost for common top-level names like "ariane", "rocket", "boom", "hwacha"
                if (s == "ariane" || s == "rocket" || s == "boom" || s == "hwacha") score += 200;
                // Heuristic: prefer longer names (usually more specific/hierarchical)
                score += std::min<int>(20, (int)name.size() / 5);
                return score;
            };

            if (!roots.empty()) {
                netlist.topModule = roots.front();
                int bestInsts = (int)netlist.modules[netlist.topModule].instances.size();
                int bestScore = scoreTopName(netlist.topModule);
                for (const auto& m : roots) {
                    int insts = (int)netlist.modules[m].instances.size();
                    int sc = scoreTopName(m);
                    // Prefer higher score first, then more instances as tiebreaker
                    if (sc > bestScore || (sc == bestScore && insts > bestInsts)) {
                        bestInsts = insts;
                        bestScore = sc;
                        netlist.topModule = m;
                    }
                }
            }
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
        static const std::set<std::string> typeKeywords = {
            "wire", "reg", "logic", "bit", "tri", "tri0", "tri1", "wand", "wor", "uwire",
            "signed", "unsigned", "var", "const", "integer", "time", "byte", "shortint", "int", "longint"
        };
        
        for (std::sregex_iterator i = begin; i != end; ++i) {
             std::string token = (*i).str();
             
             if (token == "input" || token == "output" || token == "inout") {
                 currentDir = token;
                 continue;
             }
             if (typeKeywords.count(token)) continue;
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

            // Cleanup whitespace (manual trim to avoid regex stack issues)
            {
                size_t s = statement.find_first_not_of(" \t");
                if (s == std::string::npos) continue;
                size_t e = statement.find_last_not_of(" \t");
                statement = statement.substr(s, e - s + 1);
            }
            if (statement.empty()) continue;

            // Strip one or more leading Verilog/SystemVerilog attributes: (* ... *)
            // Some synthesized netlists annotate instances this way.
            while (statement.rfind("(*", 0) == 0) {
                size_t attrEnd = statement.find("*)");
                if (attrEnd == std::string::npos) break;
                statement = statement.substr(attrEnd + 2);
                size_t s2 = statement.find_first_not_of(" \t");
                if (s2 == std::string::npos) {
                    statement.clear();
                    break;
                }
                size_t e2 = statement.find_last_not_of(" \t");
                statement = statement.substr(s2, e2 - s2 + 1);
            }
            if (statement.empty()) continue;

            // Parse non-ANSI direction declarations, e.g.:
            // input logic [31:0] a, b
            auto startsWithKw = [&](const char* kw) -> bool {
                size_t n = std::strlen(kw);
                if (statement.size() < n) return false;
                if (statement.compare(0, n, kw) != 0) return false;
                return statement.size() == n || std::isspace((unsigned char)statement[n]);
            };

            std::string dir;
            if (startsWithKw("input")) dir = "input";
            else if (startsWithKw("output")) dir = "output";
            else if (startsWithKw("inout")) dir = "inout";

            if (!dir.empty()) {
                static const std::set<std::string> typeKeywords = {
                    "wire", "reg", "logic", "bit", "tri", "tri0", "tri1", "wand", "wor", "uwire",
                    "signed", "unsigned", "var", "const", "integer", "time", "byte", "shortint", "int", "longint"
                };

                std::vector<std::string> names;
                bool inRange = false;
                for (size_t i = 0; i < statement.size();) {
                    char c = statement[i];
                    if (c == '[') { inRange = true; ++i; continue; }
                    if (c == ']') { inRange = false; ++i; continue; }

                    if (!inRange && (std::isalpha((unsigned char)c) || c == '_')) {
                        size_t j = i + 1;
                        while (j < statement.size()) {
                            char cj = statement[j];
                            if (!(std::isalnum((unsigned char)cj) || cj == '_' || cj == '$')) break;
                            ++j;
                        }
                        std::string tok = statement.substr(i, j - i);
                        if (tok != dir && !typeKeywords.count(tok)) {
                            names.push_back(tok);
                        }
                        i = j;
                        continue;
                    }
                    ++i;
                }

                for (const auto& n : names) {
                    bool found = false;
                    for (auto& p : module.ports) {
                        if (p.name == n) {
                            p.direction = dir;
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        Port p;
                        p.name = n;
                        p.direction = dir;
                        module.ports.push_back(p);
                    }
                }
                continue;
            }

            if (statement.rfind("assign", 0) == 0) {
                 module.assigns.push_back(statement);
                 continue;
            }
            
            // Should be an instance: type name ( connections )
            // Manual parse to avoid regex stack overflow on long lines
            {
                // Extract first two words
                size_t p0 = statement.find_first_not_of(" \t");
                if (p0 == std::string::npos) continue;
                size_t p1 = statement.find_first_of(" \t(", p0);
                if (p1 == std::string::npos) continue;
                std::string type = statement.substr(p0, p1 - p0);

                // Skip keywords
                static const std::set<std::string> kw = {
                    "module","input","output","inout","wire","reg","always","assign",
                    "if","else","case","for","while","begin","end","initial",
                    "generate","endgenerate","endcase","task","function",
                    "endfunction","endtask","localparam","parameter"
                };
                if (kw.count(type)) continue;

                size_t p2 = statement.find_first_not_of(" \t", p1);
                if (p2 == std::string::npos) continue;
                // Skip optional #(...) parameter override
                if (p2 < statement.size() && statement[p2] == '#') {
                    size_t lp = statement.find('(', p2);
                    if (lp == std::string::npos) continue;
                    int depth = 1; size_t k = lp + 1;
                    while (k < statement.size() && depth > 0) {
                        if (statement[k] == '(') depth++;
                        else if (statement[k] == ')') depth--;
                        k++;
                    }
                    p2 = statement.find_first_not_of(" \t", k);
                    if (p2 == std::string::npos) continue;
                }
                size_t p3 = statement.find_first_of(" \t(", p2);
                if (p3 == std::string::npos) continue;
                std::string instName = statement.substr(p2, p3 - p2);
                if (instName.empty()) continue;

                // Find the outermost ( ... ) for port list
                size_t lp = statement.find('(', p3);
                if (lp == std::string::npos) continue;
                int depth = 1; size_t k = lp + 1;
                while (k < statement.size() && depth > 0) {
                    if (statement[k] == '(') depth++;
                    else if (statement[k] == ')') depth--;
                    k++;
                }
                std::string conns = statement.substr(lp + 1, k - lp - 2);

            if (true) {
                std::string type2 = type;
                Instance inst;
                inst.type = type2;
                inst.name = instName;
                // conns is already extracted above by bracket-matching
                
                // Parse connections .port(net) with a manual scanner.
                // Supports nested parentheses in net expressions and avoids regex stack overflow.
                size_t ci = 0;
                while (ci < conns.size()) {
                    // Find next '.' starting a named port connection
                    while (ci < conns.size() && conns[ci] != '.') ++ci;
                    if (ci >= conns.size()) break;
                    ++ci; // skip '.'

                    // Parse port name
                    size_t ps = ci;
                    if (ps >= conns.size() || !(std::isalpha((unsigned char)conns[ps]) || conns[ps] == '_')) {
                        continue;
                    }
                    ++ci;
                    while (ci < conns.size()) {
                        char ch = conns[ci];
                        if (!(std::isalnum((unsigned char)ch) || ch == '_' || ch == '$')) break;
                        ++ci;
                    }
                    std::string portName = conns.substr(ps, ci - ps);

                    // Skip whitespace to '('
                    while (ci < conns.size() && std::isspace((unsigned char)conns[ci])) ++ci;
                    if (ci >= conns.size() || conns[ci] != '(') {
                        continue;
                    }
                    ++ci; // skip '('

                    // Parse expression until matching ')'
                    int depth2 = 1;
                    size_t es = ci;
                    while (ci < conns.size() && depth2 > 0) {
                        if (conns[ci] == '(') depth2++;
                        else if (conns[ci] == ')') depth2--;
                        ++ci;
                    }
                    if (depth2 != 0 || ci <= es) {
                        continue;
                    }

                    std::string expr = conns.substr(es, (ci - 1) - es);
                    // Trim whitespace
                    size_t ts = expr.find_first_not_of(" \t");
                    size_t te = expr.find_last_not_of(" \t");
                    if (ts == std::string::npos) expr.clear();
                    else expr = expr.substr(ts, te - ts + 1);

                    inst.portMap[portName] = expr;
                }
                
                module.instances.push_back(inst);
            } // if (true)
            } // manual parse block
       }
   }

   // Strip // and /* */ comments using a character-level state machine.
   // This avoids std::regex recursion stack overflow on large files.
   static std::string stripComments(const std::string& src) {
       std::string out;
       out.reserve(src.size());
       size_t i = 0, n = src.size();
       while (i < n) {
           if (i + 1 < n && src[i] == '/' && src[i+1] == '/') {
               // Line comment: skip to end of line
               while (i < n && src[i] != '\n') i++;
           } else if (i + 1 < n && src[i] == '/' && src[i+1] == '*') {
               // Block comment: skip to */
               i += 2;
               while (i + 1 < n && !(src[i] == '*' && src[i+1] == '/')) {
                   if (src[i] == '\n') out += '\n'; // preserve line numbers
                   i++;
               }
               i += 2; // skip */
           } else if (src[i] == '"') {
               // String literal: copy verbatim (handle escape sequences)
               out += src[i++];
               while (i < n && src[i] != '"') {
                   if (src[i] == '\\') { out += src[i++]; }
                   if (i < n) out += src[i++];
               }
               if (i < n) out += src[i++];
           } else {
               out += src[i++];
           }
       }
       return out;
   }
};

#endif // NETLIST_PARSER_H
