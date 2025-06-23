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
         ;

runStatement: ('initManualDrive' | 'habilitarControlManual') ';' ;
initKeyboardStatement: ('initKeyboard' | 'iniciarTeclado') '(' INT ')' ';' ;
displaySearchOptionsStatement: ('displaySearchOptions' | 'mostrarOpcionesDeBusqueda') ';' ;
setTargetAltitudeStatement: ('setTargetAltitude' | 'establecerAltitudObjetivo') '(' DOUBLE ')' ';' ;
startDroneStatement: ('start' | 'iniciar') '(' DOUBLE ')' ';' ;
hoverStatement: ('hold' | 'mantener') '(' DOUBLE ')' ';' ;
upStatement: ('ascend' | 'ascender') '(' DOUBLE ')' ';' ;
downStatement: ('descend' | 'descender') '(' DOUBLE ')' ';' ;
rotateRightStatement: ('turnRight' | 'girarDerecha') ';' ;
rotateLeftStatement: ('turnLeft' | 'girarIzquierda') ';' ;
moveAheadStatement: ('forward' | 'avanzar') '(' DOUBLE ')' ';' ;
moveBackStatement: ('backward' | 'retroceder') '(' DOUBLE ')' ';' ;

DOUBLE: [0-9]+ '.' [0-9]+ ;
INT: [0-9]+ ;
WS: [ \t\r\n]+ -> skip ;