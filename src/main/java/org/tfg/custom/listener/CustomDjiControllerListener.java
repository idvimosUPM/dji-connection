package org.tfg.custom.listener;

import org.tfg.custom.CustomDjiController;
import org.tfg.custom.gen.CustomDjiControllerBaseListener;
import org.tfg.custom.gen.CustomDjiControllerParser;

import java.util.HashMap;
import java.util.Map;

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
        double altitude = Double.parseDouble(ctx.DOUBLE().getText());
        controller.up(altitude);
    }

    @Override
    public void enterStartDroneStatement(CustomDjiControllerParser.StartDroneStatementContext ctx) {
        double velocity = Double.parseDouble(ctx.DOUBLE().getText());
        controller.startDrone(velocity);
    }

    @Override
    public void enterHoverStatement(CustomDjiControllerParser.HoverStatementContext ctx) {
        double duration = Double.parseDouble(ctx.DOUBLE().getText());
        controller.hover(duration);
    }

    @Override
    public void enterUpStatement(CustomDjiControllerParser.UpStatementContext ctx) {
        double altitude = Double.parseDouble(ctx.DOUBLE().getText());
        controller.up(altitude);
    }

    @Override
    public void enterDownStatement(CustomDjiControllerParser.DownStatementContext ctx) {
        double deltaAltitude = Double.parseDouble(ctx.DOUBLE().getText());
        controller.down(deltaAltitude);
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
        double distance = Double.parseDouble(ctx.DOUBLE().getText());
        controller.moveAhead(distance);
    }

    @Override
    public void enterMoveBackStatement(CustomDjiControllerParser.MoveBackStatementContext ctx) {
        double distance = Double.parseDouble(ctx.DOUBLE().getText());
        controller.moveBack(distance);
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

        if (exprCtx instanceof CustomDjiControllerParser.IntExprContext) {
            return Integer.parseInt(((CustomDjiControllerParser.IntExprContext) exprCtx).INT().getText());
        } else if (exprCtx instanceof CustomDjiControllerParser.DoubleExprContext) {
            return Double.parseDouble(((CustomDjiControllerParser.DoubleExprContext) exprCtx).DOUBLE().getText());
        } else if (exprCtx instanceof CustomDjiControllerParser.FloatExprContext) {
            return Double.parseDouble(((CustomDjiControllerParser.FloatExprContext) exprCtx).FLOAT().getText());
        } else if (exprCtx instanceof CustomDjiControllerParser.StringExprContext) {
            String value = ((CustomDjiControllerParser.StringExprContext) exprCtx).STRING().getText();
            if (value.length() >= 2 && value.startsWith("\"") && value.endsWith("\"")) {
                value = value.substring(1, value.length() - 1);
            }
            return value;
        } else if (exprCtx instanceof CustomDjiControllerParser.IdExprContext) {
            String varName = ((CustomDjiControllerParser.IdExprContext) exprCtx).ID().getText();
            if (variables.containsKey(varName)) {
                return variables.get(varName);
            } else {
                controller.log("ERROR: Variable '" + varName + "' not defined.");
                return null;
            }
        }
        controller.log("Error: Not recognized expression. Contexto: " + exprCtx.getClass().getName());
        return null;
    }
}