package org.tfg.custom.gen;// Generated from /Users/TFG/Documents/TFG/backend/dji-connection/src/main/antlr/CustomDjiController.g4 by ANTLR 4.13.2
import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast", "CheckReturnValue", "this-escape"})
public class CustomDjiControllerParser extends Parser {
	static { RuntimeMetaData.checkVersion("4.13.2", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		T__0=1, T__1=2, T__2=3, T__3=4, T__4=5, T__5=6, T__6=7, T__7=8, T__8=9, 
		T__9=10, T__10=11, T__11=12, T__12=13, T__13=14, T__14=15, DOUBLE=16, 
		INT=17, WS=18;
	public static final int
		RULE_program = 0, RULE_statement = 1, RULE_runStatement = 2, RULE_initKeyboardStatement = 3, 
		RULE_displaySearchOptionsStatement = 4, RULE_setTargetAltitudeStatement = 5, 
		RULE_startDroneStatement = 6, RULE_hoverStatement = 7, RULE_upStatement = 8, 
		RULE_downStatement = 9, RULE_rotateRightStatement = 10, RULE_rotateLeftStatement = 11, 
		RULE_moveAheadStatement = 12, RULE_moveBackStatement = 13;
	private static String[] makeRuleNames() {
		return new String[] {
			"program", "statement", "runStatement", "initKeyboardStatement", "displaySearchOptionsStatement", 
			"setTargetAltitudeStatement", "startDroneStatement", "hoverStatement", 
			"upStatement", "downStatement", "rotateRightStatement", "rotateLeftStatement", 
			"moveAheadStatement", "moveBackStatement"
		};
	}
	public static final String[] ruleNames = makeRuleNames();

	private static String[] makeLiteralNames() {
		return new String[] {
			null, "'initManualDrive'", "';'", "'initKeyboard'", "'('", "')'", "'displaySearchOptions'", 
			"'setTargetAltitude'", "'start'", "'hold'", "'ascend'", "'descend'", 
			"'turnRight'", "'turnLeft'", "'forward'", "'backward'"
		};
	}
	private static final String[] _LITERAL_NAMES = makeLiteralNames();
	private static String[] makeSymbolicNames() {
		return new String[] {
			null, null, null, null, null, null, null, null, null, null, null, null, 
			null, null, null, null, "DOUBLE", "INT", "WS"
		};
	}
	private static final String[] _SYMBOLIC_NAMES = makeSymbolicNames();
	public static final Vocabulary VOCABULARY = new VocabularyImpl(_LITERAL_NAMES, _SYMBOLIC_NAMES);

	/**
	 * @deprecated Use {@link #VOCABULARY} instead.
	 */
	@Deprecated
	public static final String[] tokenNames;
	static {
		tokenNames = new String[_SYMBOLIC_NAMES.length];
		for (int i = 0; i < tokenNames.length; i++) {
			tokenNames[i] = VOCABULARY.getLiteralName(i);
			if (tokenNames[i] == null) {
				tokenNames[i] = VOCABULARY.getSymbolicName(i);
			}

			if (tokenNames[i] == null) {
				tokenNames[i] = "<INVALID>";
			}
		}
	}

	@Override
	@Deprecated
	public String[] getTokenNames() {
		return tokenNames;
	}

	@Override

	public Vocabulary getVocabulary() {
		return VOCABULARY;
	}

	@Override
	public String getGrammarFileName() { return "CustomDjiController.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }

	public CustomDjiControllerParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}

	@SuppressWarnings("CheckReturnValue")
	public static class ProgramContext extends ParserRuleContext {
		public List<StatementContext> statement() {
			return getRuleContexts(StatementContext.class);
		}
		public StatementContext statement(int i) {
			return getRuleContext(StatementContext.class,i);
		}
		public ProgramContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_program; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterProgram(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitProgram(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitProgram(this);
			else return visitor.visitChildren(this);
		}
	}

	public final ProgramContext program() throws RecognitionException {
		ProgramContext _localctx = new ProgramContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_program);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(29); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(28);
				statement();
				}
				}
				setState(31); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( (((_la) & ~0x3f) == 0 && ((1L << _la) & 65482L) != 0) );
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class StatementContext extends ParserRuleContext {
		public RunStatementContext runStatement() {
			return getRuleContext(RunStatementContext.class,0);
		}
		public InitKeyboardStatementContext initKeyboardStatement() {
			return getRuleContext(InitKeyboardStatementContext.class,0);
		}
		public DisplaySearchOptionsStatementContext displaySearchOptionsStatement() {
			return getRuleContext(DisplaySearchOptionsStatementContext.class,0);
		}
		public SetTargetAltitudeStatementContext setTargetAltitudeStatement() {
			return getRuleContext(SetTargetAltitudeStatementContext.class,0);
		}
		public StartDroneStatementContext startDroneStatement() {
			return getRuleContext(StartDroneStatementContext.class,0);
		}
		public HoverStatementContext hoverStatement() {
			return getRuleContext(HoverStatementContext.class,0);
		}
		public UpStatementContext upStatement() {
			return getRuleContext(UpStatementContext.class,0);
		}
		public DownStatementContext downStatement() {
			return getRuleContext(DownStatementContext.class,0);
		}
		public RotateRightStatementContext rotateRightStatement() {
			return getRuleContext(RotateRightStatementContext.class,0);
		}
		public RotateLeftStatementContext rotateLeftStatement() {
			return getRuleContext(RotateLeftStatementContext.class,0);
		}
		public MoveAheadStatementContext moveAheadStatement() {
			return getRuleContext(MoveAheadStatementContext.class,0);
		}
		public MoveBackStatementContext moveBackStatement() {
			return getRuleContext(MoveBackStatementContext.class,0);
		}
		public StatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_statement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final StatementContext statement() throws RecognitionException {
		StatementContext _localctx = new StatementContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_statement);
		try {
			setState(45);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case T__0:
				enterOuterAlt(_localctx, 1);
				{
				setState(33);
				runStatement();
				}
				break;
			case T__2:
				enterOuterAlt(_localctx, 2);
				{
				setState(34);
				initKeyboardStatement();
				}
				break;
			case T__5:
				enterOuterAlt(_localctx, 3);
				{
				setState(35);
				displaySearchOptionsStatement();
				}
				break;
			case T__6:
				enterOuterAlt(_localctx, 4);
				{
				setState(36);
				setTargetAltitudeStatement();
				}
				break;
			case T__7:
				enterOuterAlt(_localctx, 5);
				{
				setState(37);
				startDroneStatement();
				}
				break;
			case T__8:
				enterOuterAlt(_localctx, 6);
				{
				setState(38);
				hoverStatement();
				}
				break;
			case T__9:
				enterOuterAlt(_localctx, 7);
				{
				setState(39);
				upStatement();
				}
				break;
			case T__10:
				enterOuterAlt(_localctx, 8);
				{
				setState(40);
				downStatement();
				}
				break;
			case T__11:
				enterOuterAlt(_localctx, 9);
				{
				setState(41);
				rotateRightStatement();
				}
				break;
			case T__12:
				enterOuterAlt(_localctx, 10);
				{
				setState(42);
				rotateLeftStatement();
				}
				break;
			case T__13:
				enterOuterAlt(_localctx, 11);
				{
				setState(43);
				moveAheadStatement();
				}
				break;
			case T__14:
				enterOuterAlt(_localctx, 12);
				{
				setState(44);
				moveBackStatement();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class RunStatementContext extends ParserRuleContext {
		public RunStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_runStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterRunStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitRunStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitRunStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final RunStatementContext runStatement() throws RecognitionException {
		RunStatementContext _localctx = new RunStatementContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_runStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(47);
			match(T__0);
			setState(48);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class InitKeyboardStatementContext extends ParserRuleContext {
		public TerminalNode INT() { return getToken(CustomDjiControllerParser.INT, 0); }
		public InitKeyboardStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_initKeyboardStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterInitKeyboardStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitInitKeyboardStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitInitKeyboardStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final InitKeyboardStatementContext initKeyboardStatement() throws RecognitionException {
		InitKeyboardStatementContext _localctx = new InitKeyboardStatementContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_initKeyboardStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(50);
			match(T__2);
			setState(51);
			match(T__3);
			setState(52);
			match(INT);
			setState(53);
			match(T__4);
			setState(54);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class DisplaySearchOptionsStatementContext extends ParserRuleContext {
		public DisplaySearchOptionsStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_displaySearchOptionsStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterDisplaySearchOptionsStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitDisplaySearchOptionsStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitDisplaySearchOptionsStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final DisplaySearchOptionsStatementContext displaySearchOptionsStatement() throws RecognitionException {
		DisplaySearchOptionsStatementContext _localctx = new DisplaySearchOptionsStatementContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_displaySearchOptionsStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(56);
			match(T__5);
			setState(57);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class SetTargetAltitudeStatementContext extends ParserRuleContext {
		public TerminalNode DOUBLE() { return getToken(CustomDjiControllerParser.DOUBLE, 0); }
		public SetTargetAltitudeStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_setTargetAltitudeStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterSetTargetAltitudeStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitSetTargetAltitudeStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitSetTargetAltitudeStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final SetTargetAltitudeStatementContext setTargetAltitudeStatement() throws RecognitionException {
		SetTargetAltitudeStatementContext _localctx = new SetTargetAltitudeStatementContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_setTargetAltitudeStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(59);
			match(T__6);
			setState(60);
			match(T__3);
			setState(61);
			match(DOUBLE);
			setState(62);
			match(T__4);
			setState(63);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class StartDroneStatementContext extends ParserRuleContext {
		public TerminalNode DOUBLE() { return getToken(CustomDjiControllerParser.DOUBLE, 0); }
		public StartDroneStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_startDroneStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterStartDroneStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitStartDroneStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitStartDroneStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final StartDroneStatementContext startDroneStatement() throws RecognitionException {
		StartDroneStatementContext _localctx = new StartDroneStatementContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_startDroneStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(65);
			match(T__7);
			setState(66);
			match(T__3);
			setState(67);
			match(DOUBLE);
			setState(68);
			match(T__4);
			setState(69);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class HoverStatementContext extends ParserRuleContext {
		public TerminalNode DOUBLE() { return getToken(CustomDjiControllerParser.DOUBLE, 0); }
		public HoverStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_hoverStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterHoverStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitHoverStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitHoverStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final HoverStatementContext hoverStatement() throws RecognitionException {
		HoverStatementContext _localctx = new HoverStatementContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_hoverStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(71);
			match(T__8);
			setState(72);
			match(T__3);
			setState(73);
			match(DOUBLE);
			setState(74);
			match(T__4);
			setState(75);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class UpStatementContext extends ParserRuleContext {
		public TerminalNode DOUBLE() { return getToken(CustomDjiControllerParser.DOUBLE, 0); }
		public UpStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_upStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterUpStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitUpStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitUpStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final UpStatementContext upStatement() throws RecognitionException {
		UpStatementContext _localctx = new UpStatementContext(_ctx, getState());
		enterRule(_localctx, 16, RULE_upStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(77);
			match(T__9);
			setState(78);
			match(T__3);
			setState(79);
			match(DOUBLE);
			setState(80);
			match(T__4);
			setState(81);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class DownStatementContext extends ParserRuleContext {
		public TerminalNode DOUBLE() { return getToken(CustomDjiControllerParser.DOUBLE, 0); }
		public DownStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_downStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterDownStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitDownStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitDownStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final DownStatementContext downStatement() throws RecognitionException {
		DownStatementContext _localctx = new DownStatementContext(_ctx, getState());
		enterRule(_localctx, 18, RULE_downStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(83);
			match(T__10);
			setState(84);
			match(T__3);
			setState(85);
			match(DOUBLE);
			setState(86);
			match(T__4);
			setState(87);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class RotateRightStatementContext extends ParserRuleContext {
		public RotateRightStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_rotateRightStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterRotateRightStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitRotateRightStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitRotateRightStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final RotateRightStatementContext rotateRightStatement() throws RecognitionException {
		RotateRightStatementContext _localctx = new RotateRightStatementContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_rotateRightStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(89);
			match(T__11);
			setState(90);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class RotateLeftStatementContext extends ParserRuleContext {
		public RotateLeftStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_rotateLeftStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterRotateLeftStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitRotateLeftStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitRotateLeftStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final RotateLeftStatementContext rotateLeftStatement() throws RecognitionException {
		RotateLeftStatementContext _localctx = new RotateLeftStatementContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_rotateLeftStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(92);
			match(T__12);
			setState(93);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class MoveAheadStatementContext extends ParserRuleContext {
		public TerminalNode DOUBLE() { return getToken(CustomDjiControllerParser.DOUBLE, 0); }
		public MoveAheadStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_moveAheadStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterMoveAheadStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitMoveAheadStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitMoveAheadStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final MoveAheadStatementContext moveAheadStatement() throws RecognitionException {
		MoveAheadStatementContext _localctx = new MoveAheadStatementContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_moveAheadStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(95);
			match(T__13);
			setState(96);
			match(T__3);
			setState(97);
			match(DOUBLE);
			setState(98);
			match(T__4);
			setState(99);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	@SuppressWarnings("CheckReturnValue")
	public static class MoveBackStatementContext extends ParserRuleContext {
		public TerminalNode DOUBLE() { return getToken(CustomDjiControllerParser.DOUBLE, 0); }
		public MoveBackStatementContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_moveBackStatement; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).enterMoveBackStatement(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof CustomDjiControllerListener ) ((CustomDjiControllerListener)listener).exitMoveBackStatement(this);
		}
		@Override
		public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
			if ( visitor instanceof CustomDjiControllerVisitor ) return ((CustomDjiControllerVisitor<? extends T>)visitor).visitMoveBackStatement(this);
			else return visitor.visitChildren(this);
		}
	}

	public final MoveBackStatementContext moveBackStatement() throws RecognitionException {
		MoveBackStatementContext _localctx = new MoveBackStatementContext(_ctx, getState());
		enterRule(_localctx, 26, RULE_moveBackStatement);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(101);
			match(T__14);
			setState(102);
			match(T__3);
			setState(103);
			match(DOUBLE);
			setState(104);
			match(T__4);
			setState(105);
			match(T__1);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static final String _serializedATN =
		"\u0004\u0001\u0012l\u0002\u0000\u0007\u0000\u0002\u0001\u0007\u0001\u0002"+
		"\u0002\u0007\u0002\u0002\u0003\u0007\u0003\u0002\u0004\u0007\u0004\u0002"+
		"\u0005\u0007\u0005\u0002\u0006\u0007\u0006\u0002\u0007\u0007\u0007\u0002"+
		"\b\u0007\b\u0002\t\u0007\t\u0002\n\u0007\n\u0002\u000b\u0007\u000b\u0002"+
		"\f\u0007\f\u0002\r\u0007\r\u0001\u0000\u0004\u0000\u001e\b\u0000\u000b"+
		"\u0000\f\u0000\u001f\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001"+
		"\u0001\u0001\u0001\u0003\u0001.\b\u0001\u0001\u0002\u0001\u0002\u0001"+
		"\u0002\u0001\u0003\u0001\u0003\u0001\u0003\u0001\u0003\u0001\u0003\u0001"+
		"\u0003\u0001\u0004\u0001\u0004\u0001\u0004\u0001\u0005\u0001\u0005\u0001"+
		"\u0005\u0001\u0005\u0001\u0005\u0001\u0005\u0001\u0006\u0001\u0006\u0001"+
		"\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0007\u0001\u0007\u0001"+
		"\u0007\u0001\u0007\u0001\u0007\u0001\u0007\u0001\b\u0001\b\u0001\b\u0001"+
		"\b\u0001\b\u0001\b\u0001\t\u0001\t\u0001\t\u0001\t\u0001\t\u0001\t\u0001"+
		"\n\u0001\n\u0001\n\u0001\u000b\u0001\u000b\u0001\u000b\u0001\f\u0001\f"+
		"\u0001\f\u0001\f\u0001\f\u0001\f\u0001\r\u0001\r\u0001\r\u0001\r\u0001"+
		"\r\u0001\r\u0001\r\u0000\u0000\u000e\u0000\u0002\u0004\u0006\b\n\f\u000e"+
		"\u0010\u0012\u0014\u0016\u0018\u001a\u0000\u0000i\u0000\u001d\u0001\u0000"+
		"\u0000\u0000\u0002-\u0001\u0000\u0000\u0000\u0004/\u0001\u0000\u0000\u0000"+
		"\u00062\u0001\u0000\u0000\u0000\b8\u0001\u0000\u0000\u0000\n;\u0001\u0000"+
		"\u0000\u0000\fA\u0001\u0000\u0000\u0000\u000eG\u0001\u0000\u0000\u0000"+
		"\u0010M\u0001\u0000\u0000\u0000\u0012S\u0001\u0000\u0000\u0000\u0014Y"+
		"\u0001\u0000\u0000\u0000\u0016\\\u0001\u0000\u0000\u0000\u0018_\u0001"+
		"\u0000\u0000\u0000\u001ae\u0001\u0000\u0000\u0000\u001c\u001e\u0003\u0002"+
		"\u0001\u0000\u001d\u001c\u0001\u0000\u0000\u0000\u001e\u001f\u0001\u0000"+
		"\u0000\u0000\u001f\u001d\u0001\u0000\u0000\u0000\u001f \u0001\u0000\u0000"+
		"\u0000 \u0001\u0001\u0000\u0000\u0000!.\u0003\u0004\u0002\u0000\".\u0003"+
		"\u0006\u0003\u0000#.\u0003\b\u0004\u0000$.\u0003\n\u0005\u0000%.\u0003"+
		"\f\u0006\u0000&.\u0003\u000e\u0007\u0000\'.\u0003\u0010\b\u0000(.\u0003"+
		"\u0012\t\u0000).\u0003\u0014\n\u0000*.\u0003\u0016\u000b\u0000+.\u0003"+
		"\u0018\f\u0000,.\u0003\u001a\r\u0000-!\u0001\u0000\u0000\u0000-\"\u0001"+
		"\u0000\u0000\u0000-#\u0001\u0000\u0000\u0000-$\u0001\u0000\u0000\u0000"+
		"-%\u0001\u0000\u0000\u0000-&\u0001\u0000\u0000\u0000-\'\u0001\u0000\u0000"+
		"\u0000-(\u0001\u0000\u0000\u0000-)\u0001\u0000\u0000\u0000-*\u0001\u0000"+
		"\u0000\u0000-+\u0001\u0000\u0000\u0000-,\u0001\u0000\u0000\u0000.\u0003"+
		"\u0001\u0000\u0000\u0000/0\u0005\u0001\u0000\u000001\u0005\u0002\u0000"+
		"\u00001\u0005\u0001\u0000\u0000\u000023\u0005\u0003\u0000\u000034\u0005"+
		"\u0004\u0000\u000045\u0005\u0011\u0000\u000056\u0005\u0005\u0000\u0000"+
		"67\u0005\u0002\u0000\u00007\u0007\u0001\u0000\u0000\u000089\u0005\u0006"+
		"\u0000\u00009:\u0005\u0002\u0000\u0000:\t\u0001\u0000\u0000\u0000;<\u0005"+
		"\u0007\u0000\u0000<=\u0005\u0004\u0000\u0000=>\u0005\u0010\u0000\u0000"+
		">?\u0005\u0005\u0000\u0000?@\u0005\u0002\u0000\u0000@\u000b\u0001\u0000"+
		"\u0000\u0000AB\u0005\b\u0000\u0000BC\u0005\u0004\u0000\u0000CD\u0005\u0010"+
		"\u0000\u0000DE\u0005\u0005\u0000\u0000EF\u0005\u0002\u0000\u0000F\r\u0001"+
		"\u0000\u0000\u0000GH\u0005\t\u0000\u0000HI\u0005\u0004\u0000\u0000IJ\u0005"+
		"\u0010\u0000\u0000JK\u0005\u0005\u0000\u0000KL\u0005\u0002\u0000\u0000"+
		"L\u000f\u0001\u0000\u0000\u0000MN\u0005\n\u0000\u0000NO\u0005\u0004\u0000"+
		"\u0000OP\u0005\u0010\u0000\u0000PQ\u0005\u0005\u0000\u0000QR\u0005\u0002"+
		"\u0000\u0000R\u0011\u0001\u0000\u0000\u0000ST\u0005\u000b\u0000\u0000"+
		"TU\u0005\u0004\u0000\u0000UV\u0005\u0010\u0000\u0000VW\u0005\u0005\u0000"+
		"\u0000WX\u0005\u0002\u0000\u0000X\u0013\u0001\u0000\u0000\u0000YZ\u0005"+
		"\f\u0000\u0000Z[\u0005\u0002\u0000\u0000[\u0015\u0001\u0000\u0000\u0000"+
		"\\]\u0005\r\u0000\u0000]^\u0005\u0002\u0000\u0000^\u0017\u0001\u0000\u0000"+
		"\u0000_`\u0005\u000e\u0000\u0000`a\u0005\u0004\u0000\u0000ab\u0005\u0010"+
		"\u0000\u0000bc\u0005\u0005\u0000\u0000cd\u0005\u0002\u0000\u0000d\u0019"+
		"\u0001\u0000\u0000\u0000ef\u0005\u000f\u0000\u0000fg\u0005\u0004\u0000"+
		"\u0000gh\u0005\u0010\u0000\u0000hi\u0005\u0005\u0000\u0000ij\u0005\u0002"+
		"\u0000\u0000j\u001b\u0001\u0000\u0000\u0000\u0002\u001f-";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}