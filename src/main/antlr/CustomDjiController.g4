grammar CustomDjiController;

program: statement+ ;

statement: runStatement
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
         | logStatement // Logging
         | assignmentStatement // Asigment
         ;

// AtomE
runStatement: ('initManualDrive' | 'habilitarControlManual') ';' ;
initKeyboardStatement: ('initKeyboard' | 'iniciarTeclado') '(' INT ')' ';' ;
displaySearchOptionsStatement: ('displaySearchOptions' | 'mostrarOpcionesDeBusqueda') ';' ;
setTargetAltitudeStatement: ('setTargetAltitude' | 'establecerAltitudObjetivo') '(' DOUBLE ')' ';' ;
startDroneStatement: ('start' | 'iniciar') '(' DOUBLE ')' ';' ;
hoverStatement: ('hold' | 'mantener') '(' DOUBLE ')' ';' ;
upStatement: ('ascend' | 'ascender') '(' DOUBLE ')' ';' ;
downStatement: ('descend' | 'descender') '(' DOUBLE ')' ';' ;
rotateRightStatement: ('turnRight' | 'girarDerecha') '(' ')' ';' ;
rotateLeftStatement: ('turnLeft' | 'girarIzquierda') '(' ')' ';' ;
moveAheadStatement: ('forward' | 'avanzar') '(' DOUBLE ')' ';' ;
moveBackStatement: ('backward' | 'retroceder') '(' DOUBLE ')' ';' ;

// Logging
logStatement: ('log' | 'imprimir') '(' expr ')' ';' ;

// Asigment
assignmentStatement: ID ASSIGN expr ';' ;



expr: INT     #intExpr
    | DOUBLE  #doubleExpr
    | STRING  #stringExpr
    | FLOAT   #floatExpr
    | ID      #idExpr
    ;

// Variables

STRING : '"' (~["\r\n] | '""')* '"' ;
DOUBLE: [0-9]+ '.' [0-9]+ ;
FLOAT: [0-9]+ '.' [0-9]* | '.' [0-9]+ ;
INT: [0-9]+ ;
WS: [ \t\r\n]+ -> skip ;

// Assigment variables
ASSIGN : '=';
ID : [a-zA-Z_] [a-zA-Z_0-9]* ;