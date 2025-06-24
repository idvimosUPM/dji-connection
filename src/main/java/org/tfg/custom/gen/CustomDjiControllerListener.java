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
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#block}.
	 * @param ctx the parse tree
	 */
	void enterBlock(CustomDjiControllerParser.BlockContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#block}.
	 * @param ctx the parse tree
	 */
	void exitBlock(CustomDjiControllerParser.BlockContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#stat}.
	 * @param ctx the parse tree
	 */
	void enterStat(CustomDjiControllerParser.StatContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#stat}.
	 * @param ctx the parse tree
	 */
	void exitStat(CustomDjiControllerParser.StatContext ctx);
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
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#logStatement}.
	 * @param ctx the parse tree
	 */
	void enterLogStatement(CustomDjiControllerParser.LogStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#logStatement}.
	 * @param ctx the parse tree
	 */
	void exitLogStatement(CustomDjiControllerParser.LogStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#assignmentStatement}.
	 * @param ctx the parse tree
	 */
	void enterAssignmentStatement(CustomDjiControllerParser.AssignmentStatementContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#assignmentStatement}.
	 * @param ctx the parse tree
	 */
	void exitAssignmentStatement(CustomDjiControllerParser.AssignmentStatementContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#if_stat}.
	 * @param ctx the parse tree
	 */
	void enterIf_stat(CustomDjiControllerParser.If_statContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#if_stat}.
	 * @param ctx the parse tree
	 */
	void exitIf_stat(CustomDjiControllerParser.If_statContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#condition_block}.
	 * @param ctx the parse tree
	 */
	void enterCondition_block(CustomDjiControllerParser.Condition_blockContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#condition_block}.
	 * @param ctx the parse tree
	 */
	void exitCondition_block(CustomDjiControllerParser.Condition_blockContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#stat_block}.
	 * @param ctx the parse tree
	 */
	void enterStat_block(CustomDjiControllerParser.Stat_blockContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#stat_block}.
	 * @param ctx the parse tree
	 */
	void exitStat_block(CustomDjiControllerParser.Stat_blockContext ctx);
	/**
	 * Enter a parse tree produced by {@link CustomDjiControllerParser#while_stat}.
	 * @param ctx the parse tree
	 */
	void enterWhile_stat(CustomDjiControllerParser.While_statContext ctx);
	/**
	 * Exit a parse tree produced by {@link CustomDjiControllerParser#while_stat}.
	 * @param ctx the parse tree
	 */
	void exitWhile_stat(CustomDjiControllerParser.While_statContext ctx);
	/**
	 * Enter a parse tree produced by the {@code notExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterNotExpr(CustomDjiControllerParser.NotExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code notExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitNotExpr(CustomDjiControllerParser.NotExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code unaryMinusExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterUnaryMinusExpr(CustomDjiControllerParser.UnaryMinusExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code unaryMinusExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitUnaryMinusExpr(CustomDjiControllerParser.UnaryMinusExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code multiplicationExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterMultiplicationExpr(CustomDjiControllerParser.MultiplicationExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code multiplicationExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitMultiplicationExpr(CustomDjiControllerParser.MultiplicationExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code atomExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterAtomExpr(CustomDjiControllerParser.AtomExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code atomExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitAtomExpr(CustomDjiControllerParser.AtomExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code orExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterOrExpr(CustomDjiControllerParser.OrExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code orExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitOrExpr(CustomDjiControllerParser.OrExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code additiveExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterAdditiveExpr(CustomDjiControllerParser.AdditiveExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code additiveExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitAdditiveExpr(CustomDjiControllerParser.AdditiveExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code powExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterPowExpr(CustomDjiControllerParser.PowExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code powExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitPowExpr(CustomDjiControllerParser.PowExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code relationalExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterRelationalExpr(CustomDjiControllerParser.RelationalExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code relationalExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitRelationalExpr(CustomDjiControllerParser.RelationalExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code equalityExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterEqualityExpr(CustomDjiControllerParser.EqualityExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code equalityExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitEqualityExpr(CustomDjiControllerParser.EqualityExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code andExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void enterAndExpr(CustomDjiControllerParser.AndExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code andExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#expr}.
	 * @param ctx the parse tree
	 */
	void exitAndExpr(CustomDjiControllerParser.AndExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code parExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void enterParExpr(CustomDjiControllerParser.ParExprContext ctx);
	/**
	 * Exit a parse tree produced by the {@code parExpr}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void exitParExpr(CustomDjiControllerParser.ParExprContext ctx);
	/**
	 * Enter a parse tree produced by the {@code numberAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void enterNumberAtom(CustomDjiControllerParser.NumberAtomContext ctx);
	/**
	 * Exit a parse tree produced by the {@code numberAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void exitNumberAtom(CustomDjiControllerParser.NumberAtomContext ctx);
	/**
	 * Enter a parse tree produced by the {@code booleanAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void enterBooleanAtom(CustomDjiControllerParser.BooleanAtomContext ctx);
	/**
	 * Exit a parse tree produced by the {@code booleanAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void exitBooleanAtom(CustomDjiControllerParser.BooleanAtomContext ctx);
	/**
	 * Enter a parse tree produced by the {@code idAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void enterIdAtom(CustomDjiControllerParser.IdAtomContext ctx);
	/**
	 * Exit a parse tree produced by the {@code idAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void exitIdAtom(CustomDjiControllerParser.IdAtomContext ctx);
	/**
	 * Enter a parse tree produced by the {@code stringAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void enterStringAtom(CustomDjiControllerParser.StringAtomContext ctx);
	/**
	 * Exit a parse tree produced by the {@code stringAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void exitStringAtom(CustomDjiControllerParser.StringAtomContext ctx);
	/**
	 * Enter a parse tree produced by the {@code nilAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void enterNilAtom(CustomDjiControllerParser.NilAtomContext ctx);
	/**
	 * Exit a parse tree produced by the {@code nilAtom}
	 * labeled alternative in {@link CustomDjiControllerParser#atom}.
	 * @param ctx the parse tree
	 */
	void exitNilAtom(CustomDjiControllerParser.NilAtomContext ctx);
}