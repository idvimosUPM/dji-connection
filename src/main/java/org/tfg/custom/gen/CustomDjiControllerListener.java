// Generated from /Users/TFG/Documents/TFG/backend/dji-connection/src/main/antlr/CustomDjiController.g4 by ANTLR 4.13.2
package org.tfg.custom.gen;
import org.antlr.v4.runtime.tree.ParseTreeListener;

/**
 * This interface defines a complete listener for a parse tree produced by
 * {@link CustomDjiControllerParser}.
 */
public interface CustomDjiControllerListener extends ParseTreeListener {
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#program}.
	 * @param ctx the parse tree
	 */
	void enterProgram(CustomDjiControllerParser.ProgramContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#program}.
	 * @param ctx the parse tree
	 */
	void exitProgram(CustomDjiControllerParser.ProgramContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#statement}.
	 * @param ctx the parse tree
	 */
	void enterStatement(CustomDjiControllerParser.StatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#statement}.
	 * @param ctx the parse tree
	 */
	void exitStatement(CustomDjiControllerParser.StatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#runStatement}.
	 * @param ctx the parse tree
	 */
	void enterRunStatement(CustomDjiControllerParser.RunStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#runStatement}.
	 * @param ctx the parse tree
	 */
	void exitRunStatement(CustomDjiControllerParser.RunStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#initKeyboardStatement}.
	 * @param ctx the parse tree
	 */
	void enterInitKeyboardStatement(CustomDjiControllerParser.InitKeyboardStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#initKeyboardStatement}.
	 * @param ctx the parse tree
	 */
	void exitInitKeyboardStatement(CustomDjiControllerParser.InitKeyboardStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#displaySearchOptionsStatement}.
	 * @param ctx the parse tree
	 */
	void enterDisplaySearchOptionsStatement(CustomDjiControllerParser.DisplaySearchOptionsStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#displaySearchOptionsStatement}.
	 * @param ctx the parse tree
	 */
	void exitDisplaySearchOptionsStatement(CustomDjiControllerParser.DisplaySearchOptionsStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#setTargetAltitudeStatement}.
	 * @param ctx the parse tree
	 */
	void enterSetTargetAltitudeStatement(CustomDjiControllerParser.SetTargetAltitudeStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#setTargetAltitudeStatement}.
	 * @param ctx the parse tree
	 */
	void exitSetTargetAltitudeStatement(CustomDjiControllerParser.SetTargetAltitudeStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#startDroneStatement}.
	 * @param ctx the parse tree
	 */
	void enterStartDroneStatement(CustomDjiControllerParser.StartDroneStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#startDroneStatement}.
	 * @param ctx the parse tree
	 */
	void exitStartDroneStatement(CustomDjiControllerParser.StartDroneStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#hoverStatement}.
	 * @param ctx the parse tree
	 */
	void enterHoverStatement(CustomDjiControllerParser.HoverStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#hoverStatement}.
	 * @param ctx the parse tree
	 */
	void exitHoverStatement(CustomDjiControllerParser.HoverStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#upStatement}.
	 * @param ctx the parse tree
	 */
	void enterUpStatement(CustomDjiControllerParser.UpStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#upStatement}.
	 * @param ctx the parse tree
	 */
	void exitUpStatement(CustomDjiControllerParser.UpStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#downStatement}.
	 * @param ctx the parse tree
	 */
	void enterDownStatement(CustomDjiControllerParser.DownStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#downStatement}.
	 * @param ctx the parse tree
	 */
	void exitDownStatement(CustomDjiControllerParser.DownStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#rotateRightStatement}.
	 * @param ctx the parse tree
	 */
	void enterRotateRightStatement(CustomDjiControllerParser.RotateRightStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#rotateRightStatement}.
	 * @param ctx the parse tree
	 */
	void exitRotateRightStatement(CustomDjiControllerParser.RotateRightStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#rotateLeftStatement}.
	 * @param ctx the parse tree
	 */
	void enterRotateLeftStatement(CustomDjiControllerParser.RotateLeftStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#rotateLeftStatement}.
	 * @param ctx the parse tree
	 */
	void exitRotateLeftStatement(CustomDjiControllerParser.RotateLeftStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#moveAheadStatement}.
	 * @param ctx the parse tree
	 */
	void enterMoveAheadStatement(CustomDjiControllerParser.MoveAheadStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#moveAheadStatement}.
	 * @param ctx the parse tree
	 */
	void exitMoveAheadStatement(CustomDjiControllerParser.MoveAheadStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#moveBackStatement}.
	 * @param ctx the parse tree
	 */
	void enterMoveBackStatement(CustomDjiControllerParser.MoveBackStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#moveBackStatement}.
	 * @param ctx the parse tree
	 */
	void exitMoveBackStatement(CustomDjiControllerParser.MoveBackStatementContext ctx);
}