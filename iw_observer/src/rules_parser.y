%{
//#include "rules_ast.hpp"
#include <ros/ros.h>

extern int yylex();

void yyerror(const char* err) 
{
    ROS_ERROR("Rules parser error: %s", err);
}

%}

%token TWSPACE;
%token TIDENTIFIER;
%token TINTEGER;
%token TSTRING;
%token TARROW;
%token TCOLON;
%token TCOMMA;
%token TSEMICOLON;
%token TLB;
%token TRB;


%start rules

%% 

rules : rule
      | rules rule
      ;

rule : filter TARROW cmd
     | filter TARROW TLB cmds TRB
     ;

filter : ident TCOLON idents

cmds : cmd
     | cmds cmd
     ;

cmd : ident ident TSEMICOLON
    | ident ident args TSEMICOLON

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
