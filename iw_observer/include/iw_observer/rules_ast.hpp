#ifndef RULES_AST_HPP
#define RULES_AST_HPP

#include <string>
#include <vector>
#include <iostream>

struct Rule
{
    Rule() { std::cerr << "New rule added" << std::endl; }
};

typedef std::vector<Rule*> Rules;

struct Command
{
    Command(const std::string& ident)
    {
        std::cerr << "New command: " << ident << std::endl;
    }

};

typedef std::vector<Command*> Commands;

#endif

