package org.tfg.custom.executor;

import org.antlr.v4.runtime.tree.TerminalNode;
import org.tfg.custom.CustomDjiController;
import org.tfg.custom.gen.CustomDjiControllerBaseVisitor;
import org.tfg.custom.gen.CustomDjiControllerParser;
import org.tfg.custom.types.Value;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class CustomDjiControllerExecutor extends CustomDjiControllerBaseVisitor<Value> {

    public static final double SMALL_VALUE = 0.00000000001;

    private final CustomDjiController controller;
    private final Map<String, Value> variables;

    public CustomDjiControllerExecutor(CustomDjiController controller) {
        this.controller = controller;
        this.variables = new HashMap<>();
    }

    @Override
    public Value visitProgram(CustomDjiControllerParser.ProgramContext ctx) {
        return visit(ctx.block());
    }

    @Override
    public Value visitBlock(CustomDjiControllerParser.BlockContext ctx) {
        for (CustomDjiControllerParser.StatContext stat : ctx.stat()) {
            if (stat != null) {
                visit(stat);
            }
        }
        return Value.VOID;
    }

    // Assigment
    @Override
    public Value visitAssignmentStatement(CustomDjiControllerParser.AssignmentStatementContext ctx) {
        String id = ctx.ID().getText();
        Value value = this.visit(ctx.expr());
        if (value != null) {
            variables.put(id, value);
        } else {
            controller.log("ERROR: Assigment failed to '" + id + "'.");
        }
        return Value.VOID;
    }

    // Logging
    @Override
    public Value visitLogStatement(CustomDjiControllerParser.LogStatementContext ctx) {
        Value valueToLog = this.visit(ctx.expr());
        if (valueToLog != null) {
            controller.log(valueToLog.asString());
        } else {
            controller.log("ERROR: Not resolved value to log.");
        }
        return Value.VOID;
    }

    // Drone commands
    @Override
    public Value visitRunStatement(CustomDjiControllerParser.RunStatementContext ctx) {
        controller.run();
        return Value.VOID;
    }

    @Override
    public Value visitInitKeyboardStatement(CustomDjiControllerParser.InitKeyboardStatementContext ctx) {
        int timeStep = Integer.parseInt(ctx.INT().getText());
        controller.initKeyboard(timeStep);
        return Value.VOID;
    }

    @Override
    public Value visitDisplaySearchOptionsStatement(CustomDjiControllerParser.DisplaySearchOptionsStatementContext ctx) {
        controller.displaySearchOptions();
        return Value.VOID;
    }

    @Override
    public Value visitSetTargetAltitudeStatement(CustomDjiControllerParser.SetTargetAltitudeStatementContext ctx) {
        try {
            double altitude = Double.parseDouble(ctx.FLOAT().getText());
            controller.up(altitude);
        } catch (NumberFormatException e) {
            controller.log("ERROR: Argumento inválido para establecerAltitudObjetivo. Se esperaba un número decimal.");
        }
        return Value.VOID;
    }

    @Override
    public Value visitStartDroneStatement(CustomDjiControllerParser.StartDroneStatementContext ctx) {
        Value velocityValue = visit(ctx.expr());
        if (velocityValue != null && velocityValue.isDouble()) {
            controller.startDrone(velocityValue.asDouble());
        } else {
            controller.log("ERROR: Argumento inválido para iniciar. Se esperaba un número.");
        }
        return Value.VOID;
    }

    @Override
    public Value visitHoverStatement(CustomDjiControllerParser.HoverStatementContext ctx) {
        Value durationValue = visit(ctx.expr());
        if (durationValue != null && durationValue.isDouble()) {
            controller.hover(durationValue.asDouble());
        } else {
            controller.log("ERROR: Argumento inválido para mantener. Se esperaba un número.");
        }
        return Value.VOID;
    }

    @Override
    public Value visitUpStatement(CustomDjiControllerParser.UpStatementContext ctx) {
        Value altitudeValue = visit(ctx.expr()); // <-- Visita la expresión
        if (altitudeValue != null && altitudeValue.isDouble()) {
            controller.up(altitudeValue.asDouble());
        } else {
            controller.log("ERROR: Argumento inválido para ascender. Se esperaba un número.");
        }
        return Value.VOID;
    }

    @Override
    public Value visitDownStatement(CustomDjiControllerParser.DownStatementContext ctx) {
        Value deltaAltitudeValue = visit(ctx.expr());
        if (deltaAltitudeValue != null && deltaAltitudeValue.isDouble()) {
            controller.down(deltaAltitudeValue.asDouble());
        } else {
            controller.log("ERROR: Argumento inválido para descender. Se esperaba un número.");
        }
        return Value.VOID;
    }

    @Override
    public Value visitRotateRightStatement(CustomDjiControllerParser.RotateRightStatementContext ctx) {
        controller.rotateRight();
        return Value.VOID;
    }

    @Override
    public Value visitRotateLeftStatement(CustomDjiControllerParser.RotateLeftStatementContext ctx) {
        controller.rotateLeft();
        return Value.VOID;
    }

    @Override
    public Value visitMoveAheadStatement(CustomDjiControllerParser.MoveAheadStatementContext ctx) {
        Value distanceValue = visit(ctx.expr());
        if (distanceValue != null && distanceValue.isDouble()) {
            controller.moveAhead(distanceValue.asDouble());
        } else {
            controller.log("ERROR: Argumento inválido para moverseAdelante. Se esperaba un número.");
        }
        return Value.VOID;
    }

    @Override
    public Value visitMoveBackStatement(CustomDjiControllerParser.MoveBackStatementContext ctx) {
        Value distanceValue = visit(ctx.expr());
        if (distanceValue != null && distanceValue.isDouble()) {
            controller.moveBack(distanceValue.asDouble());
        } else {
            controller.log("ERROR: Argumento inválido para moverseAtras. Se esperaba un número.");
        }
        return Value.VOID;
    }


    // IF
    @Override
    public Value visitIf_stat(CustomDjiControllerParser.If_statContext ctx) {
        List<CustomDjiControllerParser.Condition_blockContext> conditions = ctx.condition_block();
        boolean evaluatedBlock = false;

        for (CustomDjiControllerParser.Condition_blockContext condition : conditions) {
            Value evaluated = this.visit(condition.expr());

            if (evaluated != null && evaluated.isBoolean() && evaluated.asBoolean()) {
                evaluatedBlock = true;
                this.visit(condition.stat_block());
                break;
            }
        }

        if (!evaluatedBlock && ctx.ELSE() != null) {
            CustomDjiControllerParser.Stat_blockContext elseBlock = ctx.stat_block();

            if (elseBlock != null) {
                this.visit(elseBlock);
            }
        }
        return Value.VOID;
    }

    //  WHILE
    @Override
    public Value visitWhile_stat(CustomDjiControllerParser.While_statContext ctx) {
        Value conditionValue = this.visit(ctx.expr());

        while (conditionValue != null && conditionValue.isBoolean() && conditionValue.asBoolean()) {
            this.visit(ctx.stat_block());
            conditionValue = this.visit(ctx.expr());
        }
        return Value.VOID;
    }

    // Visitor methods for expressions
    @Override
    public Value visitAtomExpr(CustomDjiControllerParser.AtomExprContext ctx) {
        return visit(ctx.atom());
    }

    @Override
    public Value visitNumberAtom(CustomDjiControllerParser.NumberAtomContext ctx) {
        TerminalNode intNode = ctx.INT();
        TerminalNode floatNode = ctx.FLOAT();

        if (intNode != null) {
            return new Value(Integer.parseInt(intNode.getText()));
        } else if (floatNode != null) {
            return new Value(Double.parseDouble(floatNode.getText()));
        }
        controller.log("ERROR: Not recognized atomic number.");
        return Value.VOID;
    }

    @Override
    public Value visitBooleanAtom(CustomDjiControllerParser.BooleanAtomContext ctx) {
        return new Value(ctx.TRUE() != null);
    }

    @Override
    public Value visitIdAtom(CustomDjiControllerParser.IdAtomContext ctx) {
        String id = ctx.ID().getText();
        Value value = variables.get(id);
        if (value == null) {
            controller.log("ERROR: Variable '" + id + "' not defined.");
        }
        return value;
    }

    @Override
    public Value visitStringAtom(CustomDjiControllerParser.StringAtomContext ctx) {
        String str = ctx.STRING().getText();
        str = str.substring(1, str.length() - 1);
        return new Value(str);
    }

    @Override
    public Value visitNilAtom(CustomDjiControllerParser.NilAtomContext ctx) {
        return new Value(null);
    }

    @Override
    public Value visitParExpr(CustomDjiControllerParser.ParExprContext ctx) {
        return visit(ctx.expr());
    }

    @Override
    public Value visitPowExpr(CustomDjiControllerParser.PowExprContext ctx) {
        Value left = visit(ctx.expr(0));
        Value right = visit(ctx.expr(1));
        if (left == null || right == null || !left.isDouble() || !right.isDouble()) {
            controller.log("ERROR: Operator '^' requires numeric values and not null.");
            return Value.VOID;
        }
        return new Value(Math.pow(left.asDouble(), right.asDouble()));
    }

    @Override
    public Value visitUnaryMinusExpr(CustomDjiControllerParser.UnaryMinusExprContext ctx) {
        Value value = visit(ctx.expr());
        if (value == null || !value.isDouble()) {
            controller.log("ERROR: Operator '-' applied as a number or null.");
            return Value.VOID;
        }
        return new Value(-value.asDouble());
    }

    @Override
    public Value visitNotExpr(CustomDjiControllerParser.NotExprContext ctx) {
        Value value = visit(ctx.expr());
        if (value == null || !value.isBoolean()) {
            controller.log("ERROR: Operator '!' applied to not boolean or null.");
            return Value.VOID;
        }
        return new Value(!value.asBoolean());
    }

    @Override
    public Value visitMultiplicationExpr(CustomDjiControllerParser.MultiplicationExprContext ctx) {
        Value left = visit(ctx.expr(0));
        Value right = visit(ctx.expr(1));
        String op = ctx.op.getText();

        if (left == null || right == null || !left.isDouble() || !right.isDouble()) {
            controller.log("ERROR: Operation '" + op + "' requires numeric values and not null.");
            return Value.VOID;
        }
        double valLeft = left.asDouble();
        double valRight = right.asDouble();

        switch (op) {
            case "*":
                return new Value(valLeft * valRight);
            case "/":
                return new Value(valLeft / valRight);
            case "%":
                return new Value(valLeft % valRight);
            default:
                controller.log("ERROR: Unrecognized multiplier operator: " + op);
                return Value.VOID;
        }
    }

    @Override
    public Value visitAdditiveExpr(CustomDjiControllerParser.AdditiveExprContext ctx) {
        Value left = visit(ctx.expr(0));
        Value right = visit(ctx.expr(1));
        String op = ctx.op.getText();

        if (op.equals("+") && ((left != null && left.isString()) || (right != null && right.isString()))) {
            return new Value(left.asString() + right.asString());
        }

        if (left == null || right == null || !left.isDouble() || !right.isDouble()) {
            controller.log("ERROR: Operation '" + op + "' requires numeric values and not null.");
            return Value.VOID;
        }
        double valLeft = left.asDouble();
        double valRight = right.asDouble();

        switch (op) {
            case "+":
                return new Value(valLeft + valRight);
            case "-":
                return new Value(valLeft - valRight);
            default:
                controller.log("ERROR: Not recognized operator add: " + op);
                return Value.VOID;
        }
    }

    @Override
    public Value visitRelationalExpr(CustomDjiControllerParser.RelationalExprContext ctx) {
        Value left = visit(ctx.expr(0));
        Value right = visit(ctx.expr(1));
        String op = ctx.op.getText();

        if (left == null || right == null || !left.isDouble() || !right.isDouble()) {
            controller.log("ERROR: Comparison '" + op + "' requires numeric values and not null.");
            return Value.VOID;
        }
        double valLeft = left.asDouble();
        double valRight = right.asDouble();

        switch (op) {
            case ">":
                return new Value(valLeft > valRight);
            case "<":
                return new Value(valLeft < valRight);
            case ">=":
                return new Value(valLeft >= valRight);
            case "<=":
                return new Value(valLeft <= valRight);
            default:
                controller.log("ERROR: Unrecognized relational operation: " + op);
                return Value.VOID;
        }
    }

    @Override
    public Value visitEqualityExpr(CustomDjiControllerParser.EqualityExprContext ctx) {
        Value left = visit(ctx.expr(0));
        Value right = visit(ctx.expr(1));
        String op = ctx.op.getText();

        if (op.equals("==")) {
            if (left == null && right == null) return new Value(true);
            if (left == null || right == null) return new Value(false);
        } else if (op.equals("!=")) {
            if (left == null && right == null) return new Value(false);
            if (left == null || right == null) return new Value(true);
        }

        if (left.isDouble() && right.isDouble()) {
            double valLeft = left.asDouble();
            double valRight = right.asDouble();
            if (op.equals("==")) {
                return new Value(Math.abs(valLeft - valRight) < SMALL_VALUE);
            } else if (op.equals("!=")) {
                return new Value(Math.abs(valLeft - valRight) >= SMALL_VALUE);
            }
        }

        if (op.equals("==")) {
            return new Value(left.equals(right));
        } else if (op.equals("!=")) {
            return new Value(!left.equals(right));
        }

        controller.log("ERROR: Equals operator not recognized: " + op);
        return Value.VOID;
    }

    @Override
    public Value visitAndExpr(CustomDjiControllerParser.AndExprContext ctx) {
        Value left = visit(ctx.expr(0));
        if (left == null || !left.isBoolean()) {
            controller.log("ERROR: Left operator of '&&' is not boolean or is null.");
            return Value.VOID;
        }
        if (!left.asBoolean()) return new Value(false);

        Value right = visit(ctx.expr(1));
        if (right == null || !right.isBoolean()) {
            controller.log("ERROR: Right operator of '&&' is not boolean or is null.");
            return Value.VOID;
        }
        return new Value(left.asBoolean() && right.asBoolean());
    }

    @Override
    public Value visitOrExpr(CustomDjiControllerParser.OrExprContext ctx) {
        Value left = visit(ctx.expr(0));
        if (left == null || !left.isBoolean()) {
            controller.log("ERROR: Left operator of '||' is not boolean or is null.");
            return Value.VOID;
        }
        if (left.asBoolean()) return new Value(true);

        Value right = visit(ctx.expr(1));
        if (right == null || !right.isBoolean()) {
            controller.log("ERROR: Right operator of '||' is not boolean or is null.");
            return Value.VOID;
        }
        return new Value(left.asBoolean() || right.asBoolean());
    }
}