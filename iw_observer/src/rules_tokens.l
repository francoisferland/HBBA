%{

#include "rules_parser.hpp"

extern "C" int yywrap() { return 0; }

%}

%option nounput

%%
[ \t\n]                 ;
[a-zA-Z_][a-zA-Z0-9_]*  return TIDENTIFIER;
[0-9]+                  return TINTEGER;
","                     return TCOMMA;
":"                     return TCOLON;
";"                     return TSEMICOLON;
"->"                    return TARROW;
"("                     return TLP;
")"                     return TRP;

%%

