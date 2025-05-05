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

runStatement: 'initManualDrive' ';' ;
initKeyboardStatement: 'initKeyboard' '(' INT ')' ';' ;
displaySearchOptionsStatement: 'displaySearchOptions' ';' ;
setTargetAltitudeStatement: 'setTargetAltitude' '(' DOUBLE ')' ';' ;
startDroneStatement: 'start' '(' DOUBLE ')' ';' ;
hoverStatement: 'hold' '(' DOUBLE ')' ';' ;
upStatement: 'ascend' '(' DOUBLE ')' ';' ;
downStatement: 'descend' '(' DOUBLE ')' ';' ;
rotateRightStatement: 'turnRight' ';' ;
rotateLeftStatement: 'turnLeft' ';' ;
moveAheadStatement: 'forward' '(' DOUBLE ')' ';' ;
moveBackStatement: 'backward' '(' DOUBLE ')' ';' ;

DOUBLE: [0-9]+ '.' [0-9]+ ;
INT: [0-9]+ ;
WS: [ \t\r\n]+ -> skip ;