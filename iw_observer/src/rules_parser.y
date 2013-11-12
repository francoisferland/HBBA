%{

#include <iw_observer/rules_ast.hpp>

extern int yylex();

void yyerror(const char* err) 
{
    std::cerr <<  "Rules parser: " << err << std::endl;
}

void clearArgs(const iw_observer::Args& args)
{
    typedef iw_observer::Args::const_iterator It;
    for (It i = args.begin(); i != args.end(); ++i) {
        delete *i;
    }
}

using namespace iw_observer;

%}

%union {
    iw_observer::Rules*    rules;
    iw_observer::Rule*     rule;
    iw_observer::Commands* cmds;
    iw_observer::Command*  cmd;
    iw_observer::Args*     args;
    iw_observer::Arg*      arg;
    iw_observer::Filter*   filter;
    iw_observer::Ident*    ident;
    iw_observer::Idents*   idents;
    std::string*           string;
    int                    token;
    int                    value;

}

%token <string> TSTRING;
%token <ident>  TIDENTIFIER;
%token <value>  TINTEGER;
%token <token>  TARROW;
%token <token>  TCOLON;
%token <token>  TCOMMA;
%token <token>  TSEMICOLON;
%token <token>  TLB;
%token <token>  TRB;
%token <token>  TIDENT_ADD;
%token <token>  TIDENT_DEL;
%token <token>  TIDENT_ID;
%token <token>  TIDENT_UTIL;
%token <token>  TIDENT_INT;
%token <token>  TIDENT_PARAMS;

%type <rules>  rules;
%type <rule>   rule;
%type <cmds>   cmds cmd_block;
%type <cmd>    cmd;
%type <args>   args;
%type <arg>    arg;
%type <ident>  ident;
%type <idents> idents;
%type <filter> filter;

%start rules

%% 

rules : rule       { $$ = &ruleset(); $$->push_back($<rule>1); }
      | rules rule { $1->push_back($<rule>2); }
      ;

rule : filter TARROW cmd_block { $$ = new Rule($<filter>1, $<cmds>3); }
     ;

filter : ident TCOLON idents 
         { $$ = new Filter(*$<ident>1, *$<idents>3); delete $1; delete $3; }

cmd_block : cmd          { $$ = new Commands(); $$->push_back($<cmd>1); }
          | TLB cmds TRB { $$ = $<cmds>2; }
          ;

cmds : cmd      { $$ = new Commands(); $$->push_back($<cmd>1); } 
     | cmds cmd { $1->push_back($<cmd>2); }
     ;

cmd : TIDENT_ADD ident TSEMICOLON
      { $$ = new AddCommand(*$<ident>2, Args()); delete $2; }
    | TIDENT_ADD ident args TSEMICOLON
      { 
          $$ = new AddCommand(*$<ident>2, *$<args>3); 
          delete $2; 
          clearArgs(*$<args>3);
          delete $3; 
      }
    | TIDENT_DEL ident TSEMICOLON
      { $$ = new DelCommand(*$<ident>2); delete $2; }
    | error { yyerror("Unrecognized command"); }
    ;


idents : ident        
         { $$ = new Idents(); $$->push_back(*$<ident>1); delete $1; }
       | idents ident { $1->push_back(*$<ident>2); delete $2; }
       ;

ident : TIDENTIFIER
      ;

args : arg      { $$ = new Args(); $$->push_back($<arg>1); }
     | args arg { $1->push_back($<arg>2); }
     ;

arg : TIDENT_ID     TCOLON TIDENTIFIER { $$ = new IdArg(*$3);     delete $3; }
    | TIDENT_UTIL   TCOLON TINTEGER    { $$ = new UtilArg($3);    }
    | TIDENT_INT    TCOLON TINTEGER    { $$ = new IntArg($3);     }
    | TIDENT_PARAMS TCOLON TSTRING     { $$ = new ParamsArg(*$3); delete $3; }
    ;

%%
