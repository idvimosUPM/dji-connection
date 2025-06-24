// Generated from /Users/TFG/Documents/TFG/backend/dji-connection/src/main/antlr/CustomDjiController.g4 by ANTLR 4.13.2
package org.tfg.custom.gen;
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
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#block}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitBlock(CustomDjiControllerParser.BlockContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#stat}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitStat(CustomDjiControllerParser.StatContext ctx);
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
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#logStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitLogStatement(CustomDjiControllerParser.LogStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#assignmentStatement}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitAssignmentStatement(CustomDjiControllerParser.AssignmentStatementContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#if_stat}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitIf_stat(CustomDjiControllerParser.If_statContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#condition_block}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitCondition_block(CustomDjiControllerParser.Condition_blockContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#stat_block}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitStat_block(CustomDjiControllerParser.Stat_blockContext ctx);
	/**
	 * Visit a parse tree produced by {@link CustomDjiControllerParser#while_stat}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitWhile_stat(CustomDjiControllerParser.While_statContext ctx);
	/**
	 * Visit a parse tree produced by the {@code notExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitNotExpr(CustomDjiControllerParser.NotExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code unaryMinusExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitUnaryMinusExpr(CustomDjiControllerParser.UnaryMinusExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code multiplicationExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitMultiplicationExpr(CustomDjiControllerParser.MultiplicationExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code atomExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitAtomExpr(CustomDjiControllerParser.AtomExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code orExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitOrExpr(CustomDjiControllerParser.OrExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code additiveExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitAdditiveExpr(CustomDjiControllerParser.AdditiveExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code powExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitPowExpr(CustomDjiControllerParser.PowExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code relationalExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitRelationalExpr(CustomDjiControllerParser.RelationalExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code equalityExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitEqualityExpr(CustomDjiControllerParser.EqualityExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code andExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitAndExpr(CustomDjiControllerParser.AndExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code parExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitParExpr(CustomDjiControllerParser.ParExprContext ctx);
	/**
	 * Visit a parse tree produced by the {@code numberAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitNumberAtom(CustomDjiControllerParser.NumberAtomContext ctx);
	/**
	 * Visit a parse tree produced by the {@code booleanAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitBooleanAtom(CustomDjiControllerParser.BooleanAtomContext ctx);
	/**
	 * Visit a parse tree produced by the {@code idAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitIdAtom(CustomDjiControllerParser.IdAtomContext ctx);
	/**
	 * Visit a parse tree produced by the {@code stringAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitStringAtom(CustomDjiControllerParser.StringAtomContext ctx);
	/**
	 * Visit a parse tree produced by the {@code nilAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 * @return the visitor result
	 */
	T visitNilAtom(CustomDjiControllerParser.NilAtomContext ctx);
}