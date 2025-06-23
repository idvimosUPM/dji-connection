// Generated from /Users/TFG/Documents/TFG/backend/dji-connection/src/main/antlr/CustomDjiController.g4 by ANTLR 4.13.2
package org.tfg.custom.gen;

import org.antlr.v4.runtime.NoViableAltException;
import org.antlr.v4.runtime.Parser;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.RuntimeMetaData;
import org.antlr.v4.runtime.Token;
import org.antlr.v4.runtime.TokenStream;
import org.antlr.v4.runtime.Vocabulary;
import org.antlr.v4.runtime.VocabularyImpl;
import org.antlr.v4.runtime.atn.ATN;
import org.antlr.v4.runtime.atn.ATNDeserializer;
import org.antlr.v4.runtime.atn.ParserATNSimulator;
import org.antlr.v4.runtime.atn.PredictionContextCache;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.tree.ParseTreeListener;
import org.antlr.v4.runtime.tree.ParseTreeVisitor;
import org.antlr.v4.runtime.tree.TerminalNode;

import java.util.List;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast", "CheckReturnValue", "this-escape"})
public class CustomDjiControllerParser extends Parser {
    static {
        RuntimeMetaData.checkVersion("4.13.2", RuntimeMetaData.VERSION);
    }

    protected static final DFA[] _decisionToDFA;
    protected static final PredictionContextCache _sharedContextCache =
            new PredictionContextCache();
    public static final int
            T__0 = 1, T__1 = 2, T__2 = 3, T__3 = 4, T__4 = 5, T__5 = 6, T__6 = 7, T__7 = 8, T__8 = 9,
            T__9 = 10, T__10 = 11, T__11 = 12, T__12 = 13, T__13 = 14, T__14 = 15, T__15 = 16, T__16 = 17,
            T__17 = 18, T__18 = 19, T__19 = 20, T__20 = 21, T__21 = 22, T__22 = 23, T__23 = 24,
            T__24 = 25, T__25 = 26, T__26 = 27, T__27 = 28, T__28 = 29, STRING = 30, DOUBLE = 31,
            FLOAT = 32, INT = 33, WS = 34, ASSIGN = 35, ID = 36;
    public static final int
            RULE_program = 0, RULE_statement = 1, RULE_runStatement = 2, RULE_initKeyboardStatement = 3,
            RULE_displaySearchOptionsStatement = 4, RULE_setTargetAltitudeStatement = 5,
            RULE_startDroneStatement = 6, RULE_hoverStatement = 7, RULE_upStatement = 8,
            RULE_downStatement = 9, RULE_rotateRightStatement = 10, RULE_rotateLeftStatement = 11,
            RULE_moveAheadStatement = 12, RULE_moveBackStatement = 13, RULE_logStatement = 14,
            RULE_assignmentStatement = 15, RULE_expr = 16;

    private static String[] makeRuleNames() {
        return new String[]{
                "program", "statement", "runStatement", "initKeyboardStatement", "displaySearchOptionsStatement",
                "setTargetAltitudeStatement", "startDroneStatement", "hoverStatement",
                "upStatement", "downStatement", "rotateRightStatement", "rotateLeftStatement",
                "moveAheadStatement", "moveBackStatement", "logStatement", "assignmentStatement",
                "expr"
        };
    }

    public static final String[] ruleNames = makeRuleNames();

    private static String[] makeLiteralNames() {
        return new String[]{
                null, "'initManualDrive'", "'habilitarControlManual'", "';'", "'initKeyboard'",
                "'iniciarTeclado'", "'('", "')'", "'displaySearchOptions'", "'mostrarOpcionesDeBusqueda'",
                "'setTargetAltitude'", "'establecerAltitudObjetivo'", "'start'", "'iniciar'",
                "'hold'", "'mantener'", "'ascend'", "'ascender'", "'descend'", "'descender'",
                "'turnRight'", "'girarDerecha'", "'turnLeft'", "'girarIzquierda'", "'forward'",
                "'avanzar'", "'backward'", "'retroceder'", "'log'", "'imprimir'", null,
                null, null, null, null, "'='"
        };
    }

    private static final String[] _LITERAL_NAMES = makeLiteralNames();

    private static String[] makeSymbolicNames() {
        return new String[]{
                null, null, null, null, null, null, null, null, null, null, null, null,
                null, null, null, null, null, null, null, null, null, null, null, null,
                null, null, null, null, null, null, "STRING", "DOUBLE", "FLOAT", "INT",
                "WS", "ASSIGN", "ID"
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
    public String getGrammarFileName() {
        return "CustomDjiController.g4";
    }

    @Override
    public String[] getRuleNames() {
        return ruleNames;
    }

    @Override
    public String getSerializedATN() {
        return _serializedATN;
    }

    @Override
    public ATN getATN() {
        return _ATN;
    }

    public CustomDjiControllerParser(TokenStream input) {
        super(input);
        _interp = new ParserATNSimulator(this, _ATN, _decisionToDFA, _sharedContextCache);
    }

    @SuppressWarnings("CheckReturnValue")
    public static class ProgramContext extends ParserRuleContext {
        public List<StatementContext> statement() {
            return getRuleContexts(StatementContext.class);
        }

        public StatementContext statement(int i) {
            return getRuleContext(StatementContext.class, i);
        }

        public ProgramContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_program;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterProgram(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitProgram(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitProgram(this);
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
                setState(35);
                _errHandler.sync(this);
                _la = _input.LA(1);
                do {
                    {
                        {
                            setState(34);
                            statement();
                        }
                    }
                    setState(37);
                    _errHandler.sync(this);
                    _la = _input.LA(1);
                } while ((((_la) & ~0x3f) == 0 && ((1L << _la) & 69793218358L) != 0));
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class StatementContext extends ParserRuleContext {
        public RunStatementContext runStatement() {
            return getRuleContext(RunStatementContext.class, 0);
        }

        public InitKeyboardStatementContext initKeyboardStatement() {
            return getRuleContext(InitKeyboardStatementContext.class, 0);
        }

        public DisplaySearchOptionsStatementContext displaySearchOptionsStatement() {
            return getRuleContext(DisplaySearchOptionsStatementContext.class, 0);
        }

        public SetTargetAltitudeStatementContext setTargetAltitudeStatement() {
            return getRuleContext(SetTargetAltitudeStatementContext.class, 0);
        }

        public StartDroneStatementContext startDroneStatement() {
            return getRuleContext(StartDroneStatementContext.class, 0);
        }

        public HoverStatementContext hoverStatement() {
            return getRuleContext(HoverStatementContext.class, 0);
        }

        public UpStatementContext upStatement() {
            return getRuleContext(UpStatementContext.class, 0);
        }

        public DownStatementContext downStatement() {
            return getRuleContext(DownStatementContext.class, 0);
        }

        public RotateRightStatementContext rotateRightStatement() {
            return getRuleContext(RotateRightStatementContext.class, 0);
        }

        public RotateLeftStatementContext rotateLeftStatement() {
            return getRuleContext(RotateLeftStatementContext.class, 0);
        }

        public MoveAheadStatementContext moveAheadStatement() {
            return getRuleContext(MoveAheadStatementContext.class, 0);
        }

        public MoveBackStatementContext moveBackStatement() {
            return getRuleContext(MoveBackStatementContext.class, 0);
        }

        public LogStatementContext logStatement() {
            return getRuleContext(LogStatementContext.class, 0);
        }

        public AssignmentStatementContext assignmentStatement() {
            return getRuleContext(AssignmentStatementContext.class, 0);
        }

        public StatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_statement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final StatementContext statement() throws RecognitionException {
        StatementContext _localctx = new StatementContext(_ctx, getState());
        enterRule(_localctx, 2, RULE_statement);
        try {
            setState(53);
            _errHandler.sync(this);
            switch (_input.LA(1)) {
                case T__0:
                case T__1:
                    enterOuterAlt(_localctx, 1);
                {
                    setState(39);
                    runStatement();
                }
                break;
                case T__3:
                case T__4:
                    enterOuterAlt(_localctx, 2);
                {
                    setState(40);
                    initKeyboardStatement();
                }
                break;
                case T__7:
                case T__8:
                    enterOuterAlt(_localctx, 3);
                {
                    setState(41);
                    displaySearchOptionsStatement();
                }
                break;
                case T__9:
                case T__10:
                    enterOuterAlt(_localctx, 4);
                {
                    setState(42);
                    setTargetAltitudeStatement();
                }
                break;
                case T__11:
                case T__12:
                    enterOuterAlt(_localctx, 5);
                {
                    setState(43);
                    startDroneStatement();
                }
                break;
                case T__13:
                case T__14:
                    enterOuterAlt(_localctx, 6);
                {
                    setState(44);
                    hoverStatement();
                }
                break;
                case T__15:
                case T__16:
                    enterOuterAlt(_localctx, 7);
                {
                    setState(45);
                    upStatement();
                }
                break;
                case T__17:
                case T__18:
                    enterOuterAlt(_localctx, 8);
                {
                    setState(46);
                    downStatement();
                }
                break;
                case T__19:
                case T__20:
                    enterOuterAlt(_localctx, 9);
                {
                    setState(47);
                    rotateRightStatement();
                }
                break;
                case T__21:
                case T__22:
                    enterOuterAlt(_localctx, 10);
                {
                    setState(48);
                    rotateLeftStatement();
                }
                break;
                case T__23:
                case T__24:
                    enterOuterAlt(_localctx, 11);
                {
                    setState(49);
                    moveAheadStatement();
                }
                break;
                case T__25:
                case T__26:
                    enterOuterAlt(_localctx, 12);
                {
                    setState(50);
                    moveBackStatement();
                }
                break;
                case T__27:
                case T__28:
                    enterOuterAlt(_localctx, 13);
                {
                    setState(51);
                    logStatement();
                }
                break;
                case ID:
                    enterOuterAlt(_localctx, 14);
                {
                    setState(52);
                    assignmentStatement();
                }
                break;
                default:
                    throw new NoViableAltException(this);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class RunStatementContext extends ParserRuleContext {
        public RunStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_runStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterRunStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitRunStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitRunStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final RunStatementContext runStatement() throws RecognitionException {
        RunStatementContext _localctx = new RunStatementContext(_ctx, getState());
        enterRule(_localctx, 4, RULE_runStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(55);
                _la = _input.LA(1);
                if (!(_la == T__0 || _la == T__1)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(56);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class InitKeyboardStatementContext extends ParserRuleContext {
        public TerminalNode INT() {
            return getToken(CustomDjiControllerParser.INT, 0);
        }

        public InitKeyboardStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_initKeyboardStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterInitKeyboardStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitInitKeyboardStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitInitKeyboardStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final InitKeyboardStatementContext initKeyboardStatement() throws RecognitionException {
        InitKeyboardStatementContext _localctx = new InitKeyboardStatementContext(_ctx, getState());
        enterRule(_localctx, 6, RULE_initKeyboardStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(58);
                _la = _input.LA(1);
                if (!(_la == T__3 || _la == T__4)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(59);
                match(T__5);
                setState(60);
                match(INT);
                setState(61);
                match(T__6);
                setState(62);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class DisplaySearchOptionsStatementContext extends ParserRuleContext {
        public DisplaySearchOptionsStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_displaySearchOptionsStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterDisplaySearchOptionsStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitDisplaySearchOptionsStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitDisplaySearchOptionsStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final DisplaySearchOptionsStatementContext displaySearchOptionsStatement() throws RecognitionException {
        DisplaySearchOptionsStatementContext _localctx = new DisplaySearchOptionsStatementContext(_ctx, getState());
        enterRule(_localctx, 8, RULE_displaySearchOptionsStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(64);
                _la = _input.LA(1);
                if (!(_la == T__7 || _la == T__8)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(65);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class SetTargetAltitudeStatementContext extends ParserRuleContext {
        public TerminalNode DOUBLE() {
            return getToken(CustomDjiControllerParser.DOUBLE, 0);
        }

        public SetTargetAltitudeStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_setTargetAltitudeStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterSetTargetAltitudeStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitSetTargetAltitudeStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitSetTargetAltitudeStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final SetTargetAltitudeStatementContext setTargetAltitudeStatement() throws RecognitionException {
        SetTargetAltitudeStatementContext _localctx = new SetTargetAltitudeStatementContext(_ctx, getState());
        enterRule(_localctx, 10, RULE_setTargetAltitudeStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(67);
                _la = _input.LA(1);
                if (!(_la == T__9 || _la == T__10)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(68);
                match(T__5);
                setState(69);
                match(DOUBLE);
                setState(70);
                match(T__6);
                setState(71);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class StartDroneStatementContext extends ParserRuleContext {
        public TerminalNode DOUBLE() {
            return getToken(CustomDjiControllerParser.DOUBLE, 0);
        }

        public StartDroneStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_startDroneStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterStartDroneStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitStartDroneStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitStartDroneStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final StartDroneStatementContext startDroneStatement() throws RecognitionException {
        StartDroneStatementContext _localctx = new StartDroneStatementContext(_ctx, getState());
        enterRule(_localctx, 12, RULE_startDroneStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(73);
                _la = _input.LA(1);
                if (!(_la == T__11 || _la == T__12)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(74);
                match(T__5);
                setState(75);
                match(DOUBLE);
                setState(76);
                match(T__6);
                setState(77);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class HoverStatementContext extends ParserRuleContext {
        public TerminalNode DOUBLE() {
            return getToken(CustomDjiControllerParser.DOUBLE, 0);
        }

        public HoverStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_hoverStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterHoverStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitHoverStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitHoverStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final HoverStatementContext hoverStatement() throws RecognitionException {
        HoverStatementContext _localctx = new HoverStatementContext(_ctx, getState());
        enterRule(_localctx, 14, RULE_hoverStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(79);
                _la = _input.LA(1);
                if (!(_la == T__13 || _la == T__14)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(80);
                match(T__5);
                setState(81);
                match(DOUBLE);
                setState(82);
                match(T__6);
                setState(83);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class UpStatementContext extends ParserRuleContext {
        public TerminalNode DOUBLE() {
            return getToken(CustomDjiControllerParser.DOUBLE, 0);
        }

        public UpStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_upStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterUpStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitUpStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitUpStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final UpStatementContext upStatement() throws RecognitionException {
        UpStatementContext _localctx = new UpStatementContext(_ctx, getState());
        enterRule(_localctx, 16, RULE_upStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(85);
                _la = _input.LA(1);
                if (!(_la == T__15 || _la == T__16)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(86);
                match(T__5);
                setState(87);
                match(DOUBLE);
                setState(88);
                match(T__6);
                setState(89);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class DownStatementContext extends ParserRuleContext {
        public TerminalNode DOUBLE() {
            return getToken(CustomDjiControllerParser.DOUBLE, 0);
        }

        public DownStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_downStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterDownStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitDownStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitDownStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final DownStatementContext downStatement() throws RecognitionException {
        DownStatementContext _localctx = new DownStatementContext(_ctx, getState());
        enterRule(_localctx, 18, RULE_downStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(91);
                _la = _input.LA(1);
                if (!(_la == T__17 || _la == T__18)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(92);
                match(T__5);
                setState(93);
                match(DOUBLE);
                setState(94);
                match(T__6);
                setState(95);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class RotateRightStatementContext extends ParserRuleContext {
        public RotateRightStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_rotateRightStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterRotateRightStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitRotateRightStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitRotateRightStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final RotateRightStatementContext rotateRightStatement() throws RecognitionException {
        RotateRightStatementContext _localctx = new RotateRightStatementContext(_ctx, getState());
        enterRule(_localctx, 20, RULE_rotateRightStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(97);
                _la = _input.LA(1);
                if (!(_la == T__19 || _la == T__20)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(98);
                match(T__5);
                setState(99);
                match(T__6);
                setState(100);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class RotateLeftStatementContext extends ParserRuleContext {
        public RotateLeftStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_rotateLeftStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterRotateLeftStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitRotateLeftStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitRotateLeftStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final RotateLeftStatementContext rotateLeftStatement() throws RecognitionException {
        RotateLeftStatementContext _localctx = new RotateLeftStatementContext(_ctx, getState());
        enterRule(_localctx, 22, RULE_rotateLeftStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(102);
                _la = _input.LA(1);
                if (!(_la == T__21 || _la == T__22)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(103);
                match(T__5);
                setState(104);
                match(T__6);
                setState(105);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class MoveAheadStatementContext extends ParserRuleContext {
        public TerminalNode DOUBLE() {
            return getToken(CustomDjiControllerParser.DOUBLE, 0);
        }

        public MoveAheadStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_moveAheadStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterMoveAheadStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitMoveAheadStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitMoveAheadStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final MoveAheadStatementContext moveAheadStatement() throws RecognitionException {
        MoveAheadStatementContext _localctx = new MoveAheadStatementContext(_ctx, getState());
        enterRule(_localctx, 24, RULE_moveAheadStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(107);
                _la = _input.LA(1);
                if (!(_la == T__23 || _la == T__24)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(108);
                match(T__5);
                setState(109);
                match(DOUBLE);
                setState(110);
                match(T__6);
                setState(111);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class MoveBackStatementContext extends ParserRuleContext {
        public TerminalNode DOUBLE() {
            return getToken(CustomDjiControllerParser.DOUBLE, 0);
        }

        public MoveBackStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_moveBackStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterMoveBackStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitMoveBackStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitMoveBackStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final MoveBackStatementContext moveBackStatement() throws RecognitionException {
        MoveBackStatementContext _localctx = new MoveBackStatementContext(_ctx, getState());
        enterRule(_localctx, 26, RULE_moveBackStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(113);
                _la = _input.LA(1);
                if (!(_la == T__25 || _la == T__26)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(114);
                match(T__5);
                setState(115);
                match(DOUBLE);
                setState(116);
                match(T__6);
                setState(117);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class LogStatementContext extends ParserRuleContext {
        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
        }

        public LogStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_logStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterLogStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitLogStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitLogStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final LogStatementContext logStatement() throws RecognitionException {
        LogStatementContext _localctx = new LogStatementContext(_ctx, getState());
        enterRule(_localctx, 28, RULE_logStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(119);
                _la = _input.LA(1);
                if (!(_la == T__27 || _la == T__28)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(120);
                match(T__5);
                setState(121);
                expr();
                setState(122);
                match(T__6);
                setState(123);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class AssignmentStatementContext extends ParserRuleContext {
        public TerminalNode ID() {
            return getToken(CustomDjiControllerParser.ID, 0);
        }

        public TerminalNode ASSIGN() {
            return getToken(CustomDjiControllerParser.ASSIGN, 0);
        }

        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
        }

        public AssignmentStatementContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_assignmentStatement;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterAssignmentStatement(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitAssignmentStatement(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitAssignmentStatement(this);
            else return visitor.visitChildren(this);
        }
    }

    public final AssignmentStatementContext assignmentStatement() throws RecognitionException {
        AssignmentStatementContext _localctx = new AssignmentStatementContext(_ctx, getState());
        enterRule(_localctx, 30, RULE_assignmentStatement);
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(125);
                match(ID);
                setState(126);
                match(ASSIGN);
                setState(127);
                expr();
                setState(128);
                match(T__2);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class ExprContext extends ParserRuleContext {
        public ExprContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_expr;
        }

        public ExprContext() {
        }

        public void copyFrom(ExprContext ctx) {
            super.copyFrom(ctx);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class StringExprContext extends ExprContext {
        public TerminalNode STRING() {
            return getToken(CustomDjiControllerParser.STRING, 0);
        }

        public StringExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterStringExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitStringExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitStringExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class FloatExprContext extends ExprContext {
        public TerminalNode FLOAT() {
            return getToken(CustomDjiControllerParser.FLOAT, 0);
        }

        public FloatExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterFloatExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitFloatExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitFloatExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class DoubleExprContext extends ExprContext {
        public TerminalNode DOUBLE() {
            return getToken(CustomDjiControllerParser.DOUBLE, 0);
        }

        public DoubleExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterDoubleExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitDoubleExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitDoubleExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class IntExprContext extends ExprContext {
        public TerminalNode INT() {
            return getToken(CustomDjiControllerParser.INT, 0);
        }

        public IntExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterIntExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitIntExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitIntExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class IdExprContext extends ExprContext {
        public TerminalNode ID() {
            return getToken(CustomDjiControllerParser.ID, 0);
        }

        public IdExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterIdExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitIdExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitIdExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    public final ExprContext expr() throws RecognitionException {
        ExprContext _localctx = new ExprContext(_ctx, getState());
        enterRule(_localctx, 32, RULE_expr);
        try {
            setState(135);
            _errHandler.sync(this);
            switch (_input.LA(1)) {
                case INT:
                    _localctx = new IntExprContext(_localctx);
                    enterOuterAlt(_localctx, 1);
                {
                    setState(130);
                    match(INT);
                }
                break;
                case DOUBLE:
                    _localctx = new DoubleExprContext(_localctx);
                    enterOuterAlt(_localctx, 2);
                {
                    setState(131);
                    match(DOUBLE);
                }
                break;
                case STRING:
                    _localctx = new StringExprContext(_localctx);
                    enterOuterAlt(_localctx, 3);
                {
                    setState(132);
                    match(STRING);
                }
                break;
                case FLOAT:
                    _localctx = new FloatExprContext(_localctx);
                    enterOuterAlt(_localctx, 4);
                {
                    setState(133);
                    match(FLOAT);
                }
                break;
                case ID:
                    _localctx = new IdExprContext(_localctx);
                    enterOuterAlt(_localctx, 5);
                {
                    setState(134);
                    match(ID);
                }
                break;
                default:
                    throw new NoViableAltException(this);
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            exitRule();
        }
        return _localctx;
    }

    public static final String _serializedATN =
            "\u0004\u0001$\u008a\u0002\u0000\u0007\u0000\u0002\u0001\u0007\u0001\u0002" +
                    "\u0002\u0007\u0002\u0002\u0003\u0007\u0003\u0002\u0004\u0007\u0004\u0002" +
                    "\u0005\u0007\u0005\u0002\u0006\u0007\u0006\u0002\u0007\u0007\u0007\u0002" +
                    "\b\u0007\b\u0002\t\u0007\t\u0002\n\u0007\n\u0002\u000b\u0007\u000b\u0002" +
                    "\f\u0007\f\u0002\r\u0007\r\u0002\u000e\u0007\u000e\u0002\u000f\u0007\u000f" +
                    "\u0002\u0010\u0007\u0010\u0001\u0000\u0004\u0000$\b\u0000\u000b\u0000" +
                    "\f\u0000%\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001" +
                    "\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001\u0001" +
                    "\u0001\u0001\u0001\u0001\u0001\u0001\u0003\u00016\b\u0001\u0001\u0002" +
                    "\u0001\u0002\u0001\u0002\u0001\u0003\u0001\u0003\u0001\u0003\u0001\u0003" +
                    "\u0001\u0003\u0001\u0003\u0001\u0004\u0001\u0004\u0001\u0004\u0001\u0005" +
                    "\u0001\u0005\u0001\u0005\u0001\u0005\u0001\u0005\u0001\u0005\u0001\u0006" +
                    "\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0007" +
                    "\u0001\u0007\u0001\u0007\u0001\u0007\u0001\u0007\u0001\u0007\u0001\b\u0001" +
                    "\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\t\u0001\t\u0001\t\u0001\t\u0001" +
                    "\t\u0001\t\u0001\n\u0001\n\u0001\n\u0001\n\u0001\n\u0001\u000b\u0001\u000b" +
                    "\u0001\u000b\u0001\u000b\u0001\u000b\u0001\f\u0001\f\u0001\f\u0001\f\u0001" +
                    "\f\u0001\f\u0001\r\u0001\r\u0001\r\u0001\r\u0001\r\u0001\r\u0001\u000e" +
                    "\u0001\u000e\u0001\u000e\u0001\u000e\u0001\u000e\u0001\u000e\u0001\u000f" +
                    "\u0001\u000f\u0001\u000f\u0001\u000f\u0001\u000f\u0001\u0010\u0001\u0010" +
                    "\u0001\u0010\u0001\u0010\u0001\u0010\u0003\u0010\u0088\b\u0010\u0001\u0010" +
                    "\u0000\u0000\u0011\u0000\u0002\u0004\u0006\b\n\f\u000e\u0010\u0012\u0014" +
                    "\u0016\u0018\u001a\u001c\u001e \u0000\r\u0001\u0000\u0001\u0002\u0001" +
                    "\u0000\u0004\u0005\u0001\u0000\b\t\u0001\u0000\n\u000b\u0001\u0000\f\r" +
                    "\u0001\u0000\u000e\u000f\u0001\u0000\u0010\u0011\u0001\u0000\u0012\u0013" +
                    "\u0001\u0000\u0014\u0015\u0001\u0000\u0016\u0017\u0001\u0000\u0018\u0019" +
                    "\u0001\u0000\u001a\u001b\u0001\u0000\u001c\u001d\u008a\u0000#\u0001\u0000" +
                    "\u0000\u0000\u00025\u0001\u0000\u0000\u0000\u00047\u0001\u0000\u0000\u0000" +
                    "\u0006:\u0001\u0000\u0000\u0000\b@\u0001\u0000\u0000\u0000\nC\u0001\u0000" +
                    "\u0000\u0000\fI\u0001\u0000\u0000\u0000\u000eO\u0001\u0000\u0000\u0000" +
                    "\u0010U\u0001\u0000\u0000\u0000\u0012[\u0001\u0000\u0000\u0000\u0014a" +
                    "\u0001\u0000\u0000\u0000\u0016f\u0001\u0000\u0000\u0000\u0018k\u0001\u0000" +
                    "\u0000\u0000\u001aq\u0001\u0000\u0000\u0000\u001cw\u0001\u0000\u0000\u0000" +
                    "\u001e}\u0001\u0000\u0000\u0000 \u0087\u0001\u0000\u0000\u0000\"$\u0003" +
                    "\u0002\u0001\u0000#\"\u0001\u0000\u0000\u0000$%\u0001\u0000\u0000\u0000" +
                    "%#\u0001\u0000\u0000\u0000%&\u0001\u0000\u0000\u0000&\u0001\u0001\u0000" +
                    "\u0000\u0000\'6\u0003\u0004\u0002\u0000(6\u0003\u0006\u0003\u0000)6\u0003" +
                    "\b\u0004\u0000*6\u0003\n\u0005\u0000+6\u0003\f\u0006\u0000,6\u0003\u000e" +
                    "\u0007\u0000-6\u0003\u0010\b\u0000.6\u0003\u0012\t\u0000/6\u0003\u0014" +
                    "\n\u000006\u0003\u0016\u000b\u000016\u0003\u0018\f\u000026\u0003\u001a" +
                    "\r\u000036\u0003\u001c\u000e\u000046\u0003\u001e\u000f\u00005\'\u0001" +
                    "\u0000\u0000\u00005(\u0001\u0000\u0000\u00005)\u0001\u0000\u0000\u0000" +
                    "5*\u0001\u0000\u0000\u00005+\u0001\u0000\u0000\u00005,\u0001\u0000\u0000" +
                    "\u00005-\u0001\u0000\u0000\u00005.\u0001\u0000\u0000\u00005/\u0001\u0000" +
                    "\u0000\u000050\u0001\u0000\u0000\u000051\u0001\u0000\u0000\u000052\u0001" +
                    "\u0000\u0000\u000053\u0001\u0000\u0000\u000054\u0001\u0000\u0000\u0000" +
                    "6\u0003\u0001\u0000\u0000\u000078\u0007\u0000\u0000\u000089\u0005\u0003" +
                    "\u0000\u00009\u0005\u0001\u0000\u0000\u0000:;\u0007\u0001\u0000\u0000" +
                    ";<\u0005\u0006\u0000\u0000<=\u0005!\u0000\u0000=>\u0005\u0007\u0000\u0000" +
                    ">?\u0005\u0003\u0000\u0000?\u0007\u0001\u0000\u0000\u0000@A\u0007\u0002" +
                    "\u0000\u0000AB\u0005\u0003\u0000\u0000B\t\u0001\u0000\u0000\u0000CD\u0007" +
                    "\u0003\u0000\u0000DE\u0005\u0006\u0000\u0000EF\u0005\u001f\u0000\u0000" +
                    "FG\u0005\u0007\u0000\u0000GH\u0005\u0003\u0000\u0000H\u000b\u0001\u0000" +
                    "\u0000\u0000IJ\u0007\u0004\u0000\u0000JK\u0005\u0006\u0000\u0000KL\u0005" +
                    "\u001f\u0000\u0000LM\u0005\u0007\u0000\u0000MN\u0005\u0003\u0000\u0000" +
                    "N\r\u0001\u0000\u0000\u0000OP\u0007\u0005\u0000\u0000PQ\u0005\u0006\u0000" +
                    "\u0000QR\u0005\u001f\u0000\u0000RS\u0005\u0007\u0000\u0000ST\u0005\u0003" +
                    "\u0000\u0000T\u000f\u0001\u0000\u0000\u0000UV\u0007\u0006\u0000\u0000" +
                    "VW\u0005\u0006\u0000\u0000WX\u0005\u001f\u0000\u0000XY\u0005\u0007\u0000" +
                    "\u0000YZ\u0005\u0003\u0000\u0000Z\u0011\u0001\u0000\u0000\u0000[\\\u0007" +
                    "\u0007\u0000\u0000\\]\u0005\u0006\u0000\u0000]^\u0005\u001f\u0000\u0000" +
                    "^_\u0005\u0007\u0000\u0000_`\u0005\u0003\u0000\u0000`\u0013\u0001\u0000" +
                    "\u0000\u0000ab\u0007\b\u0000\u0000bc\u0005\u0006\u0000\u0000cd\u0005\u0007" +
                    "\u0000\u0000de\u0005\u0003\u0000\u0000e\u0015\u0001\u0000\u0000\u0000" +
                    "fg\u0007\t\u0000\u0000gh\u0005\u0006\u0000\u0000hi\u0005\u0007\u0000\u0000" +
                    "ij\u0005\u0003\u0000\u0000j\u0017\u0001\u0000\u0000\u0000kl\u0007\n\u0000" +
                    "\u0000lm\u0005\u0006\u0000\u0000mn\u0005\u001f\u0000\u0000no\u0005\u0007" +
                    "\u0000\u0000op\u0005\u0003\u0000\u0000p\u0019\u0001\u0000\u0000\u0000" +
                    "qr\u0007\u000b\u0000\u0000rs\u0005\u0006\u0000\u0000st\u0005\u001f\u0000" +
                    "\u0000tu\u0005\u0007\u0000\u0000uv\u0005\u0003\u0000\u0000v\u001b\u0001" +
                    "\u0000\u0000\u0000wx\u0007\f\u0000\u0000xy\u0005\u0006\u0000\u0000yz\u0003" +
                    " \u0010\u0000z{\u0005\u0007\u0000\u0000{|\u0005\u0003\u0000\u0000|\u001d" +
                    "\u0001\u0000\u0000\u0000}~\u0005$\u0000\u0000~\u007f\u0005#\u0000\u0000" +
                    "\u007f\u0080\u0003 \u0010\u0000\u0080\u0081\u0005\u0003\u0000\u0000\u0081" +
                    "\u001f\u0001\u0000\u0000\u0000\u0082\u0088\u0005!\u0000\u0000\u0083\u0088" +
                    "\u0005\u001f\u0000\u0000\u0084\u0088\u0005\u001e\u0000\u0000\u0085\u0088" +
                    "\u0005 \u0000\u0000\u0086\u0088\u0005$\u0000\u0000\u0087\u0082\u0001\u0000" +
                    "\u0000\u0000\u0087\u0083\u0001\u0000\u0000\u0000\u0087\u0084\u0001\u0000" +
                    "\u0000\u0000\u0087\u0085\u0001\u0000\u0000\u0000\u0087\u0086\u0001\u0000" +
                    "\u0000\u0000\u0088!\u0001\u0000\u0000\u0000\u0003%5\u0087";
    public static final ATN _ATN =
            new ATNDeserializer().deserialize(_serializedATN.toCharArray());

    static {
        _decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
        for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
            _decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
        }
    }
}