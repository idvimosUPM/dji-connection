package org.tfg.custom.listener;

import org.antlr.v4.runtime.tree.TerminalNode;
import org.tfg.custom.CustomDjiController;
import org.tfg.custom.gen.CustomDjiControllerBaseListener;
import org.tfg.custom.gen.CustomDjiControllerParser;

import java.util.HashMap;
import java.util.Map;

@Deprecated
public class CustomDjiControllerListener extends CustomDjiControllerBaseListener {

    private final CustomDjiController controller;

    private final Map<String, Object> variables;

    public CustomDjiControllerListener(CustomDjiController controller) {
        this.controller = controller;
        this.variables = new HashMap<>();
    }

    @Override
    public void enterRunStatement(CustomDjiControllerParser.RunStatementContext ctx) {
        controller.run();
    }

    @Override
    public void enterInitKeyboardStatement(CustomDjiControllerParser.InitKeyboardStatementContext ctx) {
        int timeStep = Integer.parseInt(ctx.INT().getText());
        controller.initKeyboard(timeStep);
    }

    @Override
    public void enterDisplaySearchOptionsStatement(CustomDjiControllerParser.DisplaySearchOptionsStatementContext ctx) {
        controller.displaySearchOptions();
    }

    @Override
    public void enterSetTargetAltitudeStatement(CustomDjiControllerParser.SetTargetAltitudeStatementContext ctx) {
        double altitude = Double.parseDouble(ctx.FLOAT().getText());
        controller.up(altitude);
    }

    @Override
    public void enterStartDroneStatement(CustomDjiControllerParser.StartDroneStatementContext ctx) {
        // double velocity = Double.parseDouble(ctx.FLOAT().getText());
        // controller.startDrone(velocity);
    }

    @Override
    public void enterHoverStatement(CustomDjiControllerParser.HoverStatementContext ctx) {
        //double duration = Double.parseDouble(ctx.FLOAT().getText());
        //controller.hover(duration);
    }

    @Override
    public void enterUpStatement(CustomDjiControllerParser.UpStatementContext ctx) {
        //double altitude = Double.parseDouble(ctx.FLOAT().getText());
        //controller.up(altitude);
    }

    @Override
    public void enterDownStatement(CustomDjiControllerParser.DownStatementContext ctx) {
        //double deltaAltitude = Double.parseDouble(ctx.FLOAT().getText());
        //controller.down(deltaAltitude);
    }

    @Override
    public void enterRotateRightStatement(CustomDjiControllerParser.RotateRightStatementContext ctx) {
        controller.rotateRight();
    }

    @Override
    public void enterRotateLeftStatement(CustomDjiControllerParser.RotateLeftStatementContext ctx) {
        controller.rotateLeft();
    }

    @Override
    public void enterMoveAheadStatement(CustomDjiControllerParser.MoveAheadStatementContext ctx) {
        //double distance = Double.parseDouble(ctx.FLOAT().getText());
        //controller.moveAhead(distance);
    }

    @Override
    public void enterMoveBackStatement(CustomDjiControllerParser.MoveBackStatementContext ctx) {
        // double distance = Double.parseDouble(ctx.FLOAT().getText());
        // controller.moveBack(distance);
    }

    // Logging
    @Override
    public void enterLogStatement(CustomDjiControllerParser.LogStatementContext ctx) {
        Object valueToLog = evaluateExpression(ctx.expr());

        if (valueToLog != null) {
            controller.log(valueToLog);
        } else {
            controller.log("ERROR: value to log cannot be resolved.");
        }
    }

    // Assignment
    @Override
    public void enterAssignmentStatement(CustomDjiControllerParser.AssignmentStatementContext ctx) {
        String variableName = ctx.ID().getText();
        Object value = evaluateExpression(ctx.expr());

        if (value != null) {
            variables.put(variableName, value);
        } else {
            controller.log("ERROR: Assigment failed in variable '" + variableName + "'.");
        }
    }

    private Object evaluateExpression(CustomDjiControllerParser.ExprContext exprCtx) {
        if (exprCtx == null) {
            return null;
        }

        // --- Manejo de tipos atómicos (hojas del árbol de expresiones) ---
        if (exprCtx instanceof CustomDjiControllerParser.AtomExprContext) {
            return evaluateAtom(((CustomDjiControllerParser.AtomExprContext) exprCtx).atom());
        }
        // --- Manejo de operadores unarios ---
        else if (exprCtx instanceof CustomDjiControllerParser.UnaryMinusExprContext) {
            Object value = evaluateExpression(((CustomDjiControllerParser.UnaryMinusExprContext) exprCtx).expr());
            if (value instanceof Number) {
                return -((Number) value).doubleValue();
            }
            controller.log("ERROR: Operador unario '-' aplicado a tipo no numérico.");
            return null;
        } else if (exprCtx instanceof CustomDjiControllerParser.NotExprContext) {
            Object value = evaluateExpression(((CustomDjiControllerParser.NotExprContext) exprCtx).expr());
            if (value instanceof Boolean) {
                return !(Boolean) value;
            }
            controller.log("ERROR: Operador unario '!' aplicado a tipo no booleano.");
            return null;
        }
        // --- Manejo de operadores binarios (con precedencia) ---
        else if (exprCtx instanceof CustomDjiControllerParser.PowExprContext) {
            double left = ((Number) evaluateExpression(((CustomDjiControllerParser.PowExprContext) exprCtx).expr(0))).doubleValue();
            double right = ((Number) evaluateExpression(((CustomDjiControllerParser.PowExprContext) exprCtx).expr(1))).doubleValue();
            return Math.pow(left, right);
        } else if (exprCtx instanceof CustomDjiControllerParser.MultiplicationExprContext) {
            Object left = evaluateExpression(((CustomDjiControllerParser.MultiplicationExprContext) exprCtx).expr(0));
            Object right = evaluateExpression(((CustomDjiControllerParser.MultiplicationExprContext) exprCtx).expr(1));
            String op = ((CustomDjiControllerParser.MultiplicationExprContext) exprCtx).op.getText(); // MULT, DIV, MOD

            if (!(left instanceof Number) || !(right instanceof Number)) {
                controller.log("ERROR: Operación '" + op + "' requiere operandos numéricos.");
                return null;
            }
            double valLeft = ((Number) left).doubleValue();
            double valRight = ((Number) right).doubleValue();

            switch (op) {
                case "*":
                    return valLeft * valRight;
                case "/":
                    return valLeft / valRight;
                case "%":
                    return valLeft % valRight;
                default:
                    controller.log("ERROR: Operador de multiplicación no reconocido: " + op);
                    return null;
            }
        } else if (exprCtx instanceof CustomDjiControllerParser.AdditiveExprContext) {
            Object left = evaluateExpression(((CustomDjiControllerParser.AdditiveExprContext) exprCtx).expr(0));
            Object right = evaluateExpression(((CustomDjiControllerParser.AdditiveExprContext) exprCtx).expr(1));
            String op = ((CustomDjiControllerParser.AdditiveExprContext) exprCtx).op.getText(); // PLUS, MINUS

            // Concatenación de Strings si al menos un operando es String
            if (op.equals("+") && (left instanceof String || right instanceof String)) {
                return String.valueOf(left) + String.valueOf(right);
            }

            if (!(left instanceof Number) || !(right instanceof Number)) {
                controller.log("ERROR: Operación '" + op + "' requiere operandos numéricos (o String para concatenación).");
                return null;
            }
            double valLeft = ((Number) left).doubleValue();
            double valRight = ((Number) right).doubleValue();

            switch (op) {
                case "+":
                    return valLeft + valRight;
                case "-":
                    return valLeft - valRight;
                default:
                    controller.log("ERROR: Operador aditivo no reconocido: " + op);
                    return null;
            }
        } else if (exprCtx instanceof CustomDjiControllerParser.RelationalExprContext) { // >, <, >=, <=
            Object left = evaluateExpression(((CustomDjiControllerParser.RelationalExprContext) exprCtx).expr(0));
            Object right = evaluateExpression(((CustomDjiControllerParser.RelationalExprContext) exprCtx).expr(1));
            String op = ((CustomDjiControllerParser.RelationalExprContext) exprCtx).op.getText();

            if (!(left instanceof Number) || !(right instanceof Number)) {
                controller.log("ERROR: Comparación '" + op + "' requiere operandos numéricos.");
                return null;
            }
            double valLeft = ((Number) left).doubleValue();
            double valRight = ((Number) right).doubleValue();

            switch (op) {
                case ">":
                    return valLeft > valRight;
                case "<":
                    return valLeft < valRight;
                case ">=":
                    return valLeft >= valRight;
                case "<=":
                    return valLeft <= valRight;
                default:
                    controller.log("ERROR: Operador relacional no reconocido: " + op);
                    return null;
            }
        } else if (exprCtx instanceof CustomDjiControllerParser.EqualityExprContext) { // ==, !=
            Object left = evaluateExpression(((CustomDjiControllerParser.EqualityExprContext) exprCtx).expr(0));
            Object right = evaluateExpression(((CustomDjiControllerParser.EqualityExprContext) exprCtx).expr(1));
            String op = ((CustomDjiControllerParser.EqualityExprContext) exprCtx).op.getText();

            // Comparación de igualdad genérica (maneja todos los tipos básicos)
            if (op.equals("==")) {
                // Manejo de NIL explícito
                if (left == null && right == null) return true;
                if (left == null || right == null) return false;
                return left.equals(right); // Usa equals para comparación de objetos
            } else if (op.equals("!=")) {
                if (left == null && right == null) return false;
                if (left == null || right == null) return true;
                return !left.equals(right);
            }
            return null; // Debería ser unreachable
        } else if (exprCtx instanceof CustomDjiControllerParser.AndExprContext) {
            // Evaluación cortocircuito: si el lado izquierdo es falso, no evaluar el derecho
            Object left = evaluateExpression(((CustomDjiControllerParser.AndExprContext) exprCtx).expr(0));
            if (!(left instanceof Boolean)) {
                controller.log("ERROR: Operando izquierdo de '&&' no es booleano.");
                return null;
            }
            if (!(Boolean) left) return false; // Cortocircuito

            Object right = evaluateExpression(((CustomDjiControllerParser.AndExprContext) exprCtx).expr(1));
            if (!(right instanceof Boolean)) {
                controller.log("ERROR: Operando derecho de '&&' no es booleano.");
                return null;
            }
            return (Boolean) left && (Boolean) right;
        } else if (exprCtx instanceof CustomDjiControllerParser.OrExprContext) {
            // Evaluación cortocircuito: si el lado izquierdo es verdadero, no evaluar el derecho
            Object left = evaluateExpression(((CustomDjiControllerParser.OrExprContext) exprCtx).expr(0));
            if (!(left instanceof Boolean)) {
                controller.log("ERROR: Operando izquierdo de '||' no es booleano.");
                return null;
            }
            if ((Boolean) left) return true; // Cortocircuito

            Object right = evaluateExpression(((CustomDjiControllerParser.OrExprContext) exprCtx).expr(1));
            if (!(right instanceof Boolean)) {
                controller.log("ERROR: Operando derecho de '||' no es booleano.");
                return null;
            }
            return (Boolean) left || (Boolean) right;
        }

        controller.log("Error: Tipo de expresión no reconocido o no implementado. Contexto: " + exprCtx.getClass().getName());
        return null;
    }

    private Object evaluateAtom(CustomDjiControllerParser.AtomContext atomCtx) {
        if (atomCtx == null) {
            return null;
        }

        if (atomCtx instanceof CustomDjiControllerParser.NumberAtomContext) {
            TerminalNode intNode = ((CustomDjiControllerParser.NumberAtomContext) atomCtx).INT();
            TerminalNode floatNode = ((CustomDjiControllerParser.NumberAtomContext) atomCtx).FLOAT();

            if (intNode != null) {
                return Integer.parseInt(intNode.getText());
            } else if (floatNode != null) {
                // Parseamos los FLOAT a Double en Java para mayor precisión
                return Double.parseDouble(floatNode.getText());
            }
        } else if (atomCtx instanceof CustomDjiControllerParser.BooleanAtomContext) {
            if (((CustomDjiControllerParser.BooleanAtomContext) atomCtx).TRUE() != null) {
                return true;
            } else if (((CustomDjiControllerParser.BooleanAtomContext) atomCtx).FALSE() != null) {
                return false;
            }
        } else if (atomCtx instanceof CustomDjiControllerParser.IdAtomContext) {
            String varName = ((CustomDjiControllerParser.IdAtomContext) atomCtx).ID().getText();
            if (variables.containsKey(varName)) {
                return variables.get(varName);
            } else {
                controller.log("ERROR: Variable '" + varName + "' no definida.");
                return null;
            }
        } else if (atomCtx instanceof CustomDjiControllerParser.StringAtomContext) {
            String value = ((CustomDjiControllerParser.StringAtomContext) atomCtx).STRING().getText();
            // Eliminar comillas
            if (value.length() >= 2 && value.startsWith("\"") && value.endsWith("\"")) {
                value = value.substring(1, value.length() - 1);
            }
            return value;
        } else if (atomCtx instanceof CustomDjiControllerParser.NilAtomContext) {
            return null; // Representación de 'nil' como null en Java
        } else if (atomCtx instanceof CustomDjiControllerParser.ParExprContext) {
            // Si es una expresión entre paréntesis, se evalúa la expresión interna
            return evaluateExpression(((CustomDjiControllerParser.ParExprContext) atomCtx).expr());
        }

        controller.log("Error: Tipo de átomo no reconocido o no implementado. Contexto: " + atomCtx.getClass().getName());
        return null;
    }
}