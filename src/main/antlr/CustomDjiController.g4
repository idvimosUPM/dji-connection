grammar CustomDjiController;

program: block EOF ;

block
 : stat*
 ;

stat
 : runStatement
 | initKeyboardStatement
 | displaySearchOptionsStatement
 | setTargetAltitudeStatement
 | startDroneStatement
 | hoverStatement
 | upStatement
 | downStatement
 | rotateRightStatement
 | rotateLeftStatement
 | moveAheadStatement
 | moveBackStatement
 | logStatement        // Logging
 | assignmentStatement // Assignation
 | if_stat             // IF
 | while_stat          // WHILE
 ;

// AtomE specific statements
runStatement: ('initManualDrive' | 'habilitarControlManual') '(' ')' ';' ;
initKeyboardStatement: ('initKeyboard' | 'iniciarTeclado') '(' INT ')' ';' ;
displaySearchOptionsStatement: ('displaySearchOptions' | 'mostrarOpcionesDeBusqueda') ';' ;
setTargetAltitudeStatement: ('setTargetAltitude' | 'establecerAltitudObjetivo') '(' FLOAT ')' ';' ;
startDroneStatement: ('start' | 'iniciar') '(' expr ')' ';' ;
hoverStatement: ('hold' | 'mantener') '(' expr ')' ';' ;
upStatement: ('ascend' | 'ascender') '(' expr ')' ';' ;
downStatement: ('descend' | 'descender') '(' expr ')' ';' ;
rotateRightStatement: ('turnRight' | 'girarDerecha') '(' ')' ';' ;
rotateLeftStatement: ('turnLeft' | 'girarIzquierda') '(' ')' ';' ;
moveAheadStatement: ('forward' | 'avanzar') '(' expr ')' ';' ;
moveBackStatement: ('backward' | 'retroceder') '(' expr ')' ';' ;
logStatement: ('log' | 'imprimir') expr ';' ; // Logging
assignmentStatement: ID ASSIGN expr ';' ; // Assignment
if_stat: ('if' | 'si') condition_block (('else if' | 'sino si') condition_block)* (('else' | 'sino') stat_block)? ; // IF
while_stat: ('while' | 'mientras') expr stat_block ; //  WHILE


condition_block
 : expr stat_block
 ;

stat_block
 : '{' block '}'
 | stat
 ;

// expression's rule
expr
 : <assoc=right>expr POW expr           #powExpr
 | MINUS expr                           #unaryMinusExpr
 | NOT expr                             #notExpr
 | expr op=(MULT | DIV | MOD) expr      #multiplicationExpr
 | expr op=(PLUS | MINUS) expr          #additiveExpr
 | expr op=(LTEQ | GTEQ | LT | GT) expr #relationalExpr
 | expr op=(EQ | NEQ) expr              #equalityExpr
 | expr AND expr                        #andExpr
 | expr OR expr                         #orExpr
 | atom                                 #atomExpr
 ;

atom
 : '(' expr ')' #parExpr
 | (INT | FLOAT)  #numberAtom
 | (TRUE | FALSE) #booleanAtom
 | ID             #idAtom
 | STRING         #stringAtom
 | NIL            #nilAtom
 ;

// TOKENS
OR : '||';
AND : '&&';
EQ : '==';
NEQ : '!=';
GT : '>';
LT : '<';
GTEQ : '>=';
LTEQ : '<=';
PLUS : '+';
MINUS : '-';
MULT : '*';
DIV : '/';
MOD : '%';
POW : '^';
NOT : '!';

ASSIGN : '=';

TRUE : 'true';
FALSE : 'false';
NIL : 'nil';

ID
 : [a-zA-Z_] [a-zA-Z_0-9]*
 ;

INT
 : [0-9]+
 ;

FLOAT
 : [0-9]+ '.' [0-9]*
 | '.' [0-9]+
 ;

STRING
 : '"' (~["\r\n] | '""')* '"'
 ;

COMMENT
 : '#' ~[\r\n]* -> skip
 ;

SPACE
 : [ \t\r\n]+ -> skip
 ;

OTHER
 : .
 ;