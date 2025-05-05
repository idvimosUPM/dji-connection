package org.tfg.custom.gen;// Generated from /Users/TFG/Documents/TFG/backend/dji-connection/src/main/antlr/CustomDjiController.g4 by ANTLR 4.13.2
import org.antlr.v4.runtime.tree.ParseTreeVisitor;

/**
 * This interface defines a complete generic visitor for a parse tree produced
 * by {@link CustomDjiControllerParser}.
 *
 * @param <T> The return type of the visit operation. Use {@link Void} for
 * operations with no return type.
 */
public interface CustomDjiControllerVisitor<T> extends ParseTreeVisitor<T> {
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#program}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitProgram(CustomDjiControllerParser.ProgramContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#statement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitStatement(CustomDjiControllerParser.StatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#runStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRunStatement(CustomDjiControllerParser.RunStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#initKeyboardStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitInitKeyboardStatement(CustomDjiControllerParser.InitKeyboardStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#displaySearchOptionsStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitDisplaySearchOptionsStatement(CustomDjiControllerParser.DisplaySearchOptionsStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#setTargetAltitudeStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitSetTargetAltitudeStatement(CustomDjiControllerParser.SetTargetAltitudeStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#startDroneStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitStartDroneStatement(CustomDjiControllerParser.StartDroneStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#hoverStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitHoverStatement(CustomDjiControllerParser.HoverStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#upStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitUpStatement(CustomDjiControllerParser.UpStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#downStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitDownStatement(CustomDjiControllerParser.DownStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#rotateRightStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRotateRightStatement(CustomDjiControllerParser.RotateRightStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#rotateLeftStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRotateLeftStatement(CustomDjiControllerParser.RotateLeftStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#moveAheadStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitMoveAheadStatement(CustomDjiControllerParser.MoveAheadStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#moveBackStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitMoveBackStatement(CustomDjiControllerParser.MoveBackStatementContext ctx);
}