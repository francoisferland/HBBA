%{

#include <iw_observer/rules_ast.hpp>
#include <ros/ros.h>

extern int yylex();

void yyerror(const char* err) 
{
    ROS_ERROR("Rules parser error: %s", err);
}

%}

%union {
    Rules*       rules;
    Rule*        rule;
    Commands*    cmds;
    Command*     cmd;
    std::string* string;
    int          token;

}

%token <string> TSTRING;
%token <string> TIDENTIFIER;
%token <token>  TINTEGER;
%token <token>  TARROW;
%token <token>  TCOLON;
%token <token>  TCOMMA;
%token <token>  TSEMICOLON;
%token <token>  TLB;
%token <token>  TRB;

%type <rules> rules;
%type <rule>  rule;
%type <cmds>  cmds;
%type <cmd>   cmd;

%start rules

%% 

rules : rule       { $$ = new Rules(); $$->push_back($<rule>1); }
      | rules rule { $1->push_back($<rule>2); }
      ;

rule : filter TARROW cmd_block { $$ = new Rule(); }
     ;

filter : ident TCOLON idents

cmd_block : cmd
          | TLB cmds TRB

cmds : cmd      { $$ = new Commands(); $$->push_back($<cmd>1); } 
     | cmds cmd { $1->push_back($<cmd>2); }
     ;

cmd : ident ident TSEMICOLON       { $$ = new Command(*$<string>1); }
    | ident ident args TSEMICOLON  { $$ = new Command(*$<string>1); }

idents : ident
       | idents ident

ident : TIDENTIFIER
      ;

args : arg
     | args arg
     ;

arg : ident TCOLON arg_value 
    ;

arg_value : TIDENTIFIER
          | TINTEGER
          | TSTRING
          ;

%%
