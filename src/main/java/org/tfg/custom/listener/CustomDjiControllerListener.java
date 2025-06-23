package org.tfg.custom.listener;

import org.tfg.custom.CustomDjiController;
import org.tfg.custom.gen.CustomDjiControllerBaseListener;
import org.tfg.custom.gen.CustomDjiControllerParser;

public class CustomDjiControllerListener extends CustomDjiControllerBaseListener {

    private final CustomDjiController controller;

    public CustomDjiControllerListener(CustomDjiController controller) {
        this.controller = controller;
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

    @Override
    public void enterLogStatement(CustomDjiControllerParser.LogStatementContext ctx) {
        // Obtenemos el contexto genérico de la expresión
        CustomDjiControllerParser.ExprContext exprCtx = ctx.expr();

        // Verificamos qué tipo de expresión concreta se ha encontrado
        if (exprCtx instanceof CustomDjiControllerParser.IntExprContext) {
            // Si es un entero
            CustomDjiControllerParser.IntExprContext intExprContext = (CustomDjiControllerParser.IntExprContext) exprCtx; // Casteamos al tipo específico
            int value = Integer.parseInt(intExprContext.INT().getText());
            controller.log(value);
        } else if (exprCtx instanceof CustomDjiControllerParser.DoubleExprContext) {
            // Si es un double
            CustomDjiControllerParser.DoubleExprContext doubleExprContext = (CustomDjiControllerParser.DoubleExprContext) exprCtx; // Casteamos al tipo específico
            double value = Double.parseDouble(doubleExprContext.DOUBLE().getText());
            controller.log(value);
        } else if (exprCtx instanceof CustomDjiControllerParser.StringExprContext) {
            // Si es un string
            CustomDjiControllerParser.StringExprContext stringExprContext = (CustomDjiControllerParser.StringExprContext) exprCtx; // Casteamos al tipo específico
            String value = stringExprContext.STRING().getText();
            // Los strings vienen con las comillas, las quitamos para la impresión
            if (value.startsWith("\"") && value.endsWith("\"")) {
                value = value.substring(1, value.length() - 1);
            }
            controller.log(value);
        } else {
            // Esto no debería ocurrir si la gramática está bien definida y cubre todos los casos de 'expr'
            System.err.println("Error: Tipo de expresión no reconocido en logStatement.");
        }
    }
}