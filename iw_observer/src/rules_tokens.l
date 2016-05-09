%{

#include <iw_observer/rules_ast.hpp>
#include "rules_parser.hpp"

extern "C" int yywrap() 
{ 
    return 1; // Stop at EOF. 
}

#define SAVE_TOKEN yylval.string = new std::string(yytext)

using namespace iw_observer;

%}

%option nounput

%%
[ \t\n]                     ;
add                         return TIDENT_ADD;
del                         return TIDENT_DEL;
id                          return TIDENT_ID;
utility                     return TIDENT_UTIL;
intensity                   return TIDENT_INT;
params                      return TIDENT_PARAMS;
[a-zA-Z_\-][a-zA-Z0-9_]*    SAVE_TOKEN; return TIDENTIFIER;
[0-9]+                      yylval.value = atoi(yytext); return TINTEGER;
\".\"                       return TSTRING;
","                         return TCOMMA;
":"                         return TCOLON;
";"                         return TSEMICOLON;
"->"                        return TARROW;
"{"                         return TLB;
"}"                         return TRB;

%%

