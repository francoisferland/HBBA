%{

#include <iw_observer/rules_ast.hpp>
#include "rules_parser.hpp"

extern "C" int yywrap() { return 0; }

#define SAVE_TOKEN yylval.string = new std::string(yytext)

%}

%option nounput

%%
[ \t\n]                 ;
[a-zA-Z_][a-zA-Z0-9_]*  SAVE_TOKEN; return TIDENTIFIER;
[0-9]+                  return TINTEGER;
\".\"                   return TSTRING;
","                     return TCOMMA;
":"                     return TCOLON;
";"                     return TSEMICOLON;
"->"                    return TARROW;
"{"                     return TLB;
"}"                     return TRB;

%%

