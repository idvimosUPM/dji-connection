// Generated from /Users/TFG/Documents/TFG/backend/dji-connection/src/main/antlr/CustomDjiController.g4 by ANTLR 4.13.2
package org.tfg.custom.gen;

import org.antlr.v4.runtime.FailedPredicateException;
import org.antlr.v4.runtime.NoViableAltException;
import org.antlr.v4.runtime.Parser;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.RuleContext;
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
            T__24 = 25, T__25 = 26, T__26 = 27, T__27 = 28, T__28 = 29, OR = 30, AND = 31, EQ = 32,
            NEQ = 33, GT = 34, LT = 35, GTEQ = 36, LTEQ = 37, PLUS = 38, MINUS = 39, MULT = 40, DIV = 41,
            MOD = 42, POW = 43, NOT = 44, ASSIGN = 45, TRUE = 46, FALSE = 47, NIL = 48, IF = 49, ELSE = 50,
            WHILE = 51, LOG = 52, ID = 53, INT = 54, FLOAT = 55, STRING = 56, COMMENT = 57, SPACE = 58,
            OTHER = 59;
    public static final int
            RULE_program = 0, RULE_block = 1, RULE_stat = 2, RULE_runStatement = 3,
            RULE_initKeyboardStatement = 4, RULE_displaySearchOptionsStatement = 5,
            RULE_setTargetAltitudeStatement = 6, RULE_startDroneStatement = 7, RULE_hoverStatement = 8,
            RULE_upStatement = 9, RULE_downStatement = 10, RULE_rotateRightStatement = 11,
            RULE_rotateLeftStatement = 12, RULE_moveAheadStatement = 13, RULE_moveBackStatement = 14,
            RULE_logStatement = 15, RULE_assignmentStatement = 16, RULE_if_stat = 17,
            RULE_condition_block = 18, RULE_stat_block = 19, RULE_while_stat = 20,
            RULE_expr = 21, RULE_atom = 22;

    private static String[] makeRuleNames() {
        return new String[]{
                "program", "block", "stat", "runStatement", "initKeyboardStatement",
                "displaySearchOptionsStatement", "setTargetAltitudeStatement", "startDroneStatement",
                "hoverStatement", "upStatement", "downStatement", "rotateRightStatement",
                "rotateLeftStatement", "moveAheadStatement", "moveBackStatement", "logStatement",
                "assignmentStatement", "if_stat", "condition_block", "stat_block", "while_stat",
                "expr", "atom"
        };
    }

    public static final String[] ruleNames = makeRuleNames();

    private static String[] makeLiteralNames() {
        return new String[]{
                null, "'initManualDrive'", "'habilitarControlManual'", "'('", "')'",
                "';'", "'initKeyboard'", "'iniciarTeclado'", "'displaySearchOptions'",
                "'mostrarOpcionesDeBusqueda'", "'setTargetAltitude'", "'establecerAltitudObjetivo'",
                "'start'", "'iniciar'", "'hold'", "'mantener'", "'ascend'", "'ascender'",
                "'descend'", "'descender'", "'turnRight'", "'girarDerecha'", "'turnLeft'",
                "'girarIzquierda'", "'forward'", "'avanzar'", "'backward'", "'retroceder'",
                "'{'", "'}'", "'||'", "'&&'", "'=='", "'!='", "'>'", "'<'", "'>='", "'<='",
                "'+'", "'-'", "'*'", "'/'", "'%'", "'^'", "'!'", "'='", "'true'", "'false'",
                "'nil'", "'if'", "'else'", "'while'", "'log'"
        };
    }

    private static final String[] _LITERAL_NAMES = makeLiteralNames();

    private static String[] makeSymbolicNames() {
        return new String[]{
                null, null, null, null, null, null, null, null, null, null, null, null,
                null, null, null, null, null, null, null, null, null, null, null, null,
                null, null, null, null, null, null, "OR", "AND", "EQ", "NEQ", "GT", "LT",
                "GTEQ", "LTEQ", "PLUS", "MINUS", "MULT", "DIV", "MOD", "POW", "NOT",
                "ASSIGN", "TRUE", "FALSE", "NIL", "IF", "ELSE", "WHILE", "LOG", "ID",
                "INT", "FLOAT", "STRING", "COMMENT", "SPACE", "OTHER"
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
        public BlockContext block() {
            return getRuleContext(BlockContext.class, 0);
        }

        public TerminalNode EOF() {
            return getToken(CustomDjiControllerParser.EOF, 0);
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
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(46);
                block();
                setState(47);
                match(EOF);
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
    public static class BlockContext extends ParserRuleContext {
        public List<StatContext> stat() {
            return getRuleContexts(StatContext.class);
        }

        public StatContext stat(int i) {
            return getRuleContext(StatContext.class, i);
        }

        public BlockContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_block;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterBlock(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitBlock(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitBlock(this);
            else return visitor.visitChildren(this);
        }
    }

    public final BlockContext block() throws RecognitionException {
        BlockContext _localctx = new BlockContext(_ctx, getState());
        enterRule(_localctx, 2, RULE_block);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(52);
                _errHandler.sync(this);
                _la = _input.LA(1);
                while ((((_la) & ~0x3f) == 0 && ((1L << _la) & 16325548917653446L) != 0)) {
                    {
                        {
                            setState(49);
                            stat();
                        }
                    }
                    setState(54);
                    _errHandler.sync(this);
                    _la = _input.LA(1);
                }
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
    public static class StatContext extends ParserRuleContext {
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

        public If_statContext if_stat() {
            return getRuleContext(If_statContext.class, 0);
        }

        public While_statContext while_stat() {
            return getRuleContext(While_statContext.class, 0);
        }

        public StatContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_stat;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterStat(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitStat(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitStat(this);
            else return visitor.visitChildren(this);
        }
    }

    public final StatContext stat() throws RecognitionException {
        StatContext _localctx = new StatContext(_ctx, getState());
        enterRule(_localctx, 4, RULE_stat);
        try {
            setState(71);
            _errHandler.sync(this);
            switch (_input.LA(1)) {
                case T__0:
                case T__1:
                    enterOuterAlt(_localctx, 1);
                {
                    setState(55);
                    runStatement();
                }
                break;
                case T__5:
                case T__6:
                    enterOuterAlt(_localctx, 2);
                {
                    setState(56);
                    initKeyboardStatement();
                }
                break;
                case T__7:
                case T__8:
                    enterOuterAlt(_localctx, 3);
                {
                    setState(57);
                    displaySearchOptionsStatement();
                }
                break;
                case T__9:
                case T__10:
                    enterOuterAlt(_localctx, 4);
                {
                    setState(58);
                    setTargetAltitudeStatement();
                }
                break;
                case T__11:
                case T__12:
                    enterOuterAlt(_localctx, 5);
                {
                    setState(59);
                    startDroneStatement();
                }
                break;
                case T__13:
                case T__14:
                    enterOuterAlt(_localctx, 6);
                {
                    setState(60);
                    hoverStatement();
                }
                break;
                case T__15:
                case T__16:
                    enterOuterAlt(_localctx, 7);
                {
                    setState(61);
                    upStatement();
                }
                break;
                case T__17:
                case T__18:
                    enterOuterAlt(_localctx, 8);
                {
                    setState(62);
                    downStatement();
                }
                break;
                case T__19:
                case T__20:
                    enterOuterAlt(_localctx, 9);
                {
                    setState(63);
                    rotateRightStatement();
                }
                break;
                case T__21:
                case T__22:
                    enterOuterAlt(_localctx, 10);
                {
                    setState(64);
                    rotateLeftStatement();
                }
                break;
                case T__23:
                case T__24:
                    enterOuterAlt(_localctx, 11);
                {
                    setState(65);
                    moveAheadStatement();
                }
                break;
                case T__25:
                case T__26:
                    enterOuterAlt(_localctx, 12);
                {
                    setState(66);
                    moveBackStatement();
                }
                break;
                case LOG:
                    enterOuterAlt(_localctx, 13);
                {
                    setState(67);
                    logStatement();
                }
                break;
                case ID:
                    enterOuterAlt(_localctx, 14);
                {
                    setState(68);
                    assignmentStatement();
                }
                break;
                case IF:
                    enterOuterAlt(_localctx, 15);
                {
                    setState(69);
                    if_stat();
                }
                break;
                case WHILE:
                    enterOuterAlt(_localctx, 16);
                {
                    setState(70);
                    while_stat();
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
        enterRule(_localctx, 6, RULE_runStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(73);
                _la = _input.LA(1);
                if (!(_la == T__0 || _la == T__1)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(74);
                match(T__2);
                setState(75);
                match(T__3);
                setState(76);
                match(T__4);
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
        enterRule(_localctx, 8, RULE_initKeyboardStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(78);
                _la = _input.LA(1);
                if (!(_la == T__5 || _la == T__6)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(79);
                match(T__2);
                setState(80);
                match(INT);
                setState(81);
                match(T__3);
                setState(82);
                match(T__4);
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
        enterRule(_localctx, 10, RULE_displaySearchOptionsStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(84);
                _la = _input.LA(1);
                if (!(_la == T__7 || _la == T__8)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(85);
                match(T__4);
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
        public TerminalNode FLOAT() {
            return getToken(CustomDjiControllerParser.FLOAT, 0);
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
        enterRule(_localctx, 12, RULE_setTargetAltitudeStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(87);
                _la = _input.LA(1);
                if (!(_la == T__9 || _la == T__10)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(88);
                match(T__2);
                setState(89);
                match(FLOAT);
                setState(90);
                match(T__3);
                setState(91);
                match(T__4);
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
        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
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
        enterRule(_localctx, 14, RULE_startDroneStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(93);
                _la = _input.LA(1);
                if (!(_la == T__11 || _la == T__12)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(94);
                match(T__2);
                setState(95);
                expr(0);
                setState(96);
                match(T__3);
                setState(97);
                match(T__4);
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
        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
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
        enterRule(_localctx, 16, RULE_hoverStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(99);
                _la = _input.LA(1);
                if (!(_la == T__13 || _la == T__14)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(100);
                match(T__2);
                setState(101);
                expr(0);
                setState(102);
                match(T__3);
                setState(103);
                match(T__4);
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
        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
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
        enterRule(_localctx, 18, RULE_upStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(105);
                _la = _input.LA(1);
                if (!(_la == T__15 || _la == T__16)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(106);
                match(T__2);
                setState(107);
                expr(0);
                setState(108);
                match(T__3);
                setState(109);
                match(T__4);
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
        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
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
        enterRule(_localctx, 20, RULE_downStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(111);
                _la = _input.LA(1);
                if (!(_la == T__17 || _la == T__18)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(112);
                match(T__2);
                setState(113);
                expr(0);
                setState(114);
                match(T__3);
                setState(115);
                match(T__4);
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
        enterRule(_localctx, 22, RULE_rotateRightStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(117);
                _la = _input.LA(1);
                if (!(_la == T__19 || _la == T__20)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(118);
                match(T__2);
                setState(119);
                match(T__3);
                setState(120);
                match(T__4);
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
        enterRule(_localctx, 24, RULE_rotateLeftStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(122);
                _la = _input.LA(1);
                if (!(_la == T__21 || _la == T__22)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(123);
                match(T__2);
                setState(124);
                match(T__3);
                setState(125);
                match(T__4);
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
        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
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
        enterRule(_localctx, 26, RULE_moveAheadStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(127);
                _la = _input.LA(1);
                if (!(_la == T__23 || _la == T__24)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(128);
                match(T__2);
                setState(129);
                expr(0);
                setState(130);
                match(T__3);
                setState(131);
                match(T__4);
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
        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
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
        enterRule(_localctx, 28, RULE_moveBackStatement);
        int _la;
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(133);
                _la = _input.LA(1);
                if (!(_la == T__25 || _la == T__26)) {
                    _errHandler.recoverInline(this);
                } else {
                    if (_input.LA(1) == Token.EOF) matchedEOF = true;
                    _errHandler.reportMatch(this);
                    consume();
                }
                setState(134);
                match(T__2);
                setState(135);
                expr(0);
                setState(136);
                match(T__3);
                setState(137);
                match(T__4);
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
        public TerminalNode LOG() {
            return getToken(CustomDjiControllerParser.LOG, 0);
        }

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
        enterRule(_localctx, 30, RULE_logStatement);
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(139);
                match(LOG);
                setState(140);
                expr(0);
                setState(141);
                match(T__4);
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
        enterRule(_localctx, 32, RULE_assignmentStatement);
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(143);
                match(ID);
                setState(144);
                match(ASSIGN);
                setState(145);
                expr(0);
                setState(146);
                match(T__4);
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
    public static class If_statContext extends ParserRuleContext {
        public List<TerminalNode> IF() {
            return getTokens(CustomDjiControllerParser.IF);
        }

        public TerminalNode IF(int i) {
            return getToken(CustomDjiControllerParser.IF, i);
        }

        public List<Condition_blockContext> condition_block() {
            return getRuleContexts(Condition_blockContext.class);
        }

        public Condition_blockContext condition_block(int i) {
            return getRuleContext(Condition_blockContext.class, i);
        }

        public List<TerminalNode> ELSE() {
            return getTokens(CustomDjiControllerParser.ELSE);
        }

        public TerminalNode ELSE(int i) {
            return getToken(CustomDjiControllerParser.ELSE, i);
        }

        public Stat_blockContext stat_block() {
            return getRuleContext(Stat_blockContext.class, 0);
        }

        public If_statContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_if_stat;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterIf_stat(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitIf_stat(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitIf_stat(this);
            else return visitor.visitChildren(this);
        }
    }

    public final If_statContext if_stat() throws RecognitionException {
        If_statContext _localctx = new If_statContext(_ctx, getState());
        enterRule(_localctx, 34, RULE_if_stat);
        try {
            int _alt;
            enterOuterAlt(_localctx, 1);
            {
                setState(148);
                match(IF);
                setState(149);
                condition_block();
                setState(155);
                _errHandler.sync(this);
                _alt = getInterpreter().adaptivePredict(_input, 2, _ctx);
                while (_alt != 2 && _alt != org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER) {
                    if (_alt == 1) {
                        {
                            {
                                setState(150);
                                match(ELSE);
                                setState(151);
                                match(IF);
                                setState(152);
                                condition_block();
                            }
                        }
                    }
                    setState(157);
                    _errHandler.sync(this);
                    _alt = getInterpreter().adaptivePredict(_input, 2, _ctx);
                }
                setState(160);
                _errHandler.sync(this);
                switch (getInterpreter().adaptivePredict(_input, 3, _ctx)) {
                    case 1: {
                        setState(158);
                        match(ELSE);
                        setState(159);
                        stat_block();
                    }
                    break;
                }
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
    public static class Condition_blockContext extends ParserRuleContext {
        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
        }

        public Stat_blockContext stat_block() {
            return getRuleContext(Stat_blockContext.class, 0);
        }

        public Condition_blockContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_condition_block;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterCondition_block(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitCondition_block(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitCondition_block(this);
            else return visitor.visitChildren(this);
        }
    }

    public final Condition_blockContext condition_block() throws RecognitionException {
        Condition_blockContext _localctx = new Condition_blockContext(_ctx, getState());
        enterRule(_localctx, 36, RULE_condition_block);
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(162);
                expr(0);
                setState(163);
                stat_block();
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
    public static class Stat_blockContext extends ParserRuleContext {
        public BlockContext block() {
            return getRuleContext(BlockContext.class, 0);
        }

        public StatContext stat() {
            return getRuleContext(StatContext.class, 0);
        }

        public Stat_blockContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_stat_block;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterStat_block(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitStat_block(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitStat_block(this);
            else return visitor.visitChildren(this);
        }
    }

    public final Stat_blockContext stat_block() throws RecognitionException {
        Stat_blockContext _localctx = new Stat_blockContext(_ctx, getState());
        enterRule(_localctx, 38, RULE_stat_block);
        try {
            setState(170);
            _errHandler.sync(this);
            switch (_input.LA(1)) {
                case T__27:
                    enterOuterAlt(_localctx, 1);
                {
                    setState(165);
                    match(T__27);
                    setState(166);
                    block();
                    setState(167);
                    match(T__28);
                }
                break;
                case T__0:
                case T__1:
                case T__5:
                case T__6:
                case T__7:
                case T__8:
                case T__9:
                case T__10:
                case T__11:
                case T__12:
                case T__13:
                case T__14:
                case T__15:
                case T__16:
                case T__17:
                case T__18:
                case T__19:
                case T__20:
                case T__21:
                case T__22:
                case T__23:
                case T__24:
                case T__25:
                case T__26:
                case IF:
                case WHILE:
                case LOG:
                case ID:
                    enterOuterAlt(_localctx, 2);
                {
                    setState(169);
                    stat();
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
    public static class While_statContext extends ParserRuleContext {
        public TerminalNode WHILE() {
            return getToken(CustomDjiControllerParser.WHILE, 0);
        }

        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
        }

        public Stat_blockContext stat_block() {
            return getRuleContext(Stat_blockContext.class, 0);
        }

        public While_statContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_while_stat;
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterWhile_stat(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitWhile_stat(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitWhile_stat(this);
            else return visitor.visitChildren(this);
        }
    }

    public final While_statContext while_stat() throws RecognitionException {
        While_statContext _localctx = new While_statContext(_ctx, getState());
        enterRule(_localctx, 40, RULE_while_stat);
        try {
            enterOuterAlt(_localctx, 1);
            {
                setState(172);
                match(WHILE);
                setState(173);
                expr(0);
                setState(174);
                stat_block();
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
    public static class NotExprContext extends ExprContext {
        public TerminalNode NOT() {
            return getToken(CustomDjiControllerParser.NOT, 0);
        }

        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
        }

        public NotExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterNotExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitNotExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitNotExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class UnaryMinusExprContext extends ExprContext {
        public TerminalNode MINUS() {
            return getToken(CustomDjiControllerParser.MINUS, 0);
        }

        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
        }

        public UnaryMinusExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterUnaryMinusExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitUnaryMinusExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitUnaryMinusExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class MultiplicationExprContext extends ExprContext {
        public Token op;

        public List<ExprContext> expr() {
            return getRuleContexts(ExprContext.class);
        }

        public ExprContext expr(int i) {
            return getRuleContext(ExprContext.class, i);
        }

        public TerminalNode MULT() {
            return getToken(CustomDjiControllerParser.MULT, 0);
        }

        public TerminalNode DIV() {
            return getToken(CustomDjiControllerParser.DIV, 0);
        }

        public TerminalNode MOD() {
            return getToken(CustomDjiControllerParser.MOD, 0);
        }

        public MultiplicationExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterMultiplicationExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitMultiplicationExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitMultiplicationExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class AtomExprContext extends ExprContext {
        public AtomContext atom() {
            return getRuleContext(AtomContext.class, 0);
        }

        public AtomExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterAtomExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitAtomExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitAtomExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class OrExprContext extends ExprContext {
        public List<ExprContext> expr() {
            return getRuleContexts(ExprContext.class);
        }

        public ExprContext expr(int i) {
            return getRuleContext(ExprContext.class, i);
        }

        public TerminalNode OR() {
            return getToken(CustomDjiControllerParser.OR, 0);
        }

        public OrExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterOrExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitOrExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitOrExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class AdditiveExprContext extends ExprContext {
        public Token op;

        public List<ExprContext> expr() {
            return getRuleContexts(ExprContext.class);
        }

        public ExprContext expr(int i) {
            return getRuleContext(ExprContext.class, i);
        }

        public TerminalNode PLUS() {
            return getToken(CustomDjiControllerParser.PLUS, 0);
        }

        public TerminalNode MINUS() {
            return getToken(CustomDjiControllerParser.MINUS, 0);
        }

        public AdditiveExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterAdditiveExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitAdditiveExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitAdditiveExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class PowExprContext extends ExprContext {
        public List<ExprContext> expr() {
            return getRuleContexts(ExprContext.class);
        }

        public ExprContext expr(int i) {
            return getRuleContext(ExprContext.class, i);
        }

        public TerminalNode POW() {
            return getToken(CustomDjiControllerParser.POW, 0);
        }

        public PowExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterPowExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitPowExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitPowExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class RelationalExprContext extends ExprContext {
        public Token op;

        public List<ExprContext> expr() {
            return getRuleContexts(ExprContext.class);
        }

        public ExprContext expr(int i) {
            return getRuleContext(ExprContext.class, i);
        }

        public TerminalNode LTEQ() {
            return getToken(CustomDjiControllerParser.LTEQ, 0);
        }

        public TerminalNode GTEQ() {
            return getToken(CustomDjiControllerParser.GTEQ, 0);
        }

        public TerminalNode LT() {
            return getToken(CustomDjiControllerParser.LT, 0);
        }

        public TerminalNode GT() {
            return getToken(CustomDjiControllerParser.GT, 0);
        }

        public RelationalExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterRelationalExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitRelationalExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitRelationalExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class EqualityExprContext extends ExprContext {
        public Token op;

        public List<ExprContext> expr() {
            return getRuleContexts(ExprContext.class);
        }

        public ExprContext expr(int i) {
            return getRuleContext(ExprContext.class, i);
        }

        public TerminalNode EQ() {
            return getToken(CustomDjiControllerParser.EQ, 0);
        }

        public TerminalNode NEQ() {
            return getToken(CustomDjiControllerParser.NEQ, 0);
        }

        public EqualityExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterEqualityExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitEqualityExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitEqualityExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class AndExprContext extends ExprContext {
        public List<ExprContext> expr() {
            return getRuleContexts(ExprContext.class);
        }

        public ExprContext expr(int i) {
            return getRuleContext(ExprContext.class, i);
        }

        public TerminalNode AND() {
            return getToken(CustomDjiControllerParser.AND, 0);
        }

        public AndExprContext(ExprContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterAndExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitAndExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitAndExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    public final ExprContext expr() throws RecognitionException {
        return expr(0);
    }

    private ExprContext expr(int _p) throws RecognitionException {
        ParserRuleContext _parentctx = _ctx;
        int _parentState = getState();
        ExprContext _localctx = new ExprContext(_ctx, _parentState);
        ExprContext _prevctx = _localctx;
        int _startState = 42;
        enterRecursionRule(_localctx, 42, RULE_expr, _p);
        int _la;
        try {
            int _alt;
            enterOuterAlt(_localctx, 1);
            {
                setState(182);
                _errHandler.sync(this);
                switch (_input.LA(1)) {
                    case MINUS: {
                        _localctx = new UnaryMinusExprContext(_localctx);
                        _ctx = _localctx;
                        _prevctx = _localctx;

                        setState(177);
                        match(MINUS);
                        setState(178);
                        expr(9);
                    }
                    break;
                    case NOT: {
                        _localctx = new NotExprContext(_localctx);
                        _ctx = _localctx;
                        _prevctx = _localctx;
                        setState(179);
                        match(NOT);
                        setState(180);
                        expr(8);
                    }
                    break;
                    case T__2:
                    case TRUE:
                    case FALSE:
                    case NIL:
                    case ID:
                    case INT:
                    case FLOAT:
                    case STRING: {
                        _localctx = new AtomExprContext(_localctx);
                        _ctx = _localctx;
                        _prevctx = _localctx;
                        setState(181);
                        atom();
                    }
                    break;
                    default:
                        throw new NoViableAltException(this);
                }
                _ctx.stop = _input.LT(-1);
                setState(207);
                _errHandler.sync(this);
                _alt = getInterpreter().adaptivePredict(_input, 7, _ctx);
                while (_alt != 2 && _alt != org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER) {
                    if (_alt == 1) {
                        if (_parseListeners != null) triggerExitRuleEvent();
                        _prevctx = _localctx;
                        {
                            setState(205);
                            _errHandler.sync(this);
                            switch (getInterpreter().adaptivePredict(_input, 6, _ctx)) {
                                case 1: {
                                    _localctx = new PowExprContext(new ExprContext(_parentctx, _parentState));
                                    pushNewRecursionContext(_localctx, _startState, RULE_expr);
                                    setState(184);
                                    if (!(precpred(_ctx, 10)))
                                        throw new FailedPredicateException(this, "precpred(_ctx, 10)");
                                    setState(185);
                                    match(POW);
                                    setState(186);
                                    expr(10);
                                }
                                break;
                                case 2: {
                                    _localctx = new MultiplicationExprContext(new ExprContext(_parentctx, _parentState));
                                    pushNewRecursionContext(_localctx, _startState, RULE_expr);
                                    setState(187);
                                    if (!(precpred(_ctx, 7)))
                                        throw new FailedPredicateException(this, "precpred(_ctx, 7)");
                                    setState(188);
                                    ((MultiplicationExprContext) _localctx).op = _input.LT(1);
                                    _la = _input.LA(1);
                                    if (!((((_la) & ~0x3f) == 0 && ((1L << _la) & 7696581394432L) != 0))) {
                                        ((MultiplicationExprContext) _localctx).op = (Token) _errHandler.recoverInline(this);
                                    } else {
                                        if (_input.LA(1) == Token.EOF) matchedEOF = true;
                                        _errHandler.reportMatch(this);
                                        consume();
                                    }
                                    setState(189);
                                    expr(8);
                                }
                                break;
                                case 3: {
                                    _localctx = new AdditiveExprContext(new ExprContext(_parentctx, _parentState));
                                    pushNewRecursionContext(_localctx, _startState, RULE_expr);
                                    setState(190);
                                    if (!(precpred(_ctx, 6)))
                                        throw new FailedPredicateException(this, "precpred(_ctx, 6)");
                                    setState(191);
                                    ((AdditiveExprContext) _localctx).op = _input.LT(1);
                                    _la = _input.LA(1);
                                    if (!(_la == PLUS || _la == MINUS)) {
                                        ((AdditiveExprContext) _localctx).op = (Token) _errHandler.recoverInline(this);
                                    } else {
                                        if (_input.LA(1) == Token.EOF) matchedEOF = true;
                                        _errHandler.reportMatch(this);
                                        consume();
                                    }
                                    setState(192);
                                    expr(7);
                                }
                                break;
                                case 4: {
                                    _localctx = new RelationalExprContext(new ExprContext(_parentctx, _parentState));
                                    pushNewRecursionContext(_localctx, _startState, RULE_expr);
                                    setState(193);
                                    if (!(precpred(_ctx, 5)))
                                        throw new FailedPredicateException(this, "precpred(_ctx, 5)");
                                    setState(194);
                                    ((RelationalExprContext) _localctx).op = _input.LT(1);
                                    _la = _input.LA(1);
                                    if (!((((_la) & ~0x3f) == 0 && ((1L << _la) & 257698037760L) != 0))) {
                                        ((RelationalExprContext) _localctx).op = (Token) _errHandler.recoverInline(this);
                                    } else {
                                        if (_input.LA(1) == Token.EOF) matchedEOF = true;
                                        _errHandler.reportMatch(this);
                                        consume();
                                    }
                                    setState(195);
                                    expr(6);
                                }
                                break;
                                case 5: {
                                    _localctx = new EqualityExprContext(new ExprContext(_parentctx, _parentState));
                                    pushNewRecursionContext(_localctx, _startState, RULE_expr);
                                    setState(196);
                                    if (!(precpred(_ctx, 4)))
                                        throw new FailedPredicateException(this, "precpred(_ctx, 4)");
                                    setState(197);
                                    ((EqualityExprContext) _localctx).op = _input.LT(1);
                                    _la = _input.LA(1);
                                    if (!(_la == EQ || _la == NEQ)) {
                                        ((EqualityExprContext) _localctx).op = (Token) _errHandler.recoverInline(this);
                                    } else {
                                        if (_input.LA(1) == Token.EOF) matchedEOF = true;
                                        _errHandler.reportMatch(this);
                                        consume();
                                    }
                                    setState(198);
                                    expr(5);
                                }
                                break;
                                case 6: {
                                    _localctx = new AndExprContext(new ExprContext(_parentctx, _parentState));
                                    pushNewRecursionContext(_localctx, _startState, RULE_expr);
                                    setState(199);
                                    if (!(precpred(_ctx, 3)))
                                        throw new FailedPredicateException(this, "precpred(_ctx, 3)");
                                    setState(200);
                                    match(AND);
                                    setState(201);
                                    expr(4);
                                }
                                break;
                                case 7: {
                                    _localctx = new OrExprContext(new ExprContext(_parentctx, _parentState));
                                    pushNewRecursionContext(_localctx, _startState, RULE_expr);
                                    setState(202);
                                    if (!(precpred(_ctx, 2)))
                                        throw new FailedPredicateException(this, "precpred(_ctx, 2)");
                                    setState(203);
                                    match(OR);
                                    setState(204);
                                    expr(3);
                                }
                                break;
                            }
                        }
                    }
                    setState(209);
                    _errHandler.sync(this);
                    _alt = getInterpreter().adaptivePredict(_input, 7, _ctx);
                }
            }
        } catch (RecognitionException re) {
            _localctx.exception = re;
            _errHandler.reportError(this, re);
            _errHandler.recover(this, re);
        } finally {
            unrollRecursionContexts(_parentctx);
        }
        return _localctx;
    }

    @SuppressWarnings("CheckReturnValue")
    public static class AtomContext extends ParserRuleContext {
        public AtomContext(ParserRuleContext parent, int invokingState) {
            super(parent, invokingState);
        }

        @Override
        public int getRuleIndex() {
            return RULE_atom;
        }

        public AtomContext() {
        }

        public void copyFrom(AtomContext ctx) {
            super.copyFrom(ctx);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class ParExprContext extends AtomContext {
        public ExprContext expr() {
            return getRuleContext(ExprContext.class, 0);
        }

        public ParExprContext(AtomContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterParExpr(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitParExpr(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitParExpr(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class BooleanAtomContext extends AtomContext {
        public TerminalNode TRUE() {
            return getToken(CustomDjiControllerParser.TRUE, 0);
        }

        public TerminalNode FALSE() {
            return getToken(CustomDjiControllerParser.FALSE, 0);
        }

        public BooleanAtomContext(AtomContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterBooleanAtom(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitBooleanAtom(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitBooleanAtom(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class IdAtomContext extends AtomContext {
        public TerminalNode ID() {
            return getToken(CustomDjiControllerParser.ID, 0);
        }

        public IdAtomContext(AtomContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterIdAtom(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitIdAtom(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitIdAtom(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class StringAtomContext extends AtomContext {
        public TerminalNode STRING() {
            return getToken(CustomDjiControllerParser.STRING, 0);
        }

        public StringAtomContext(AtomContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterStringAtom(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitStringAtom(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitStringAtom(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class NilAtomContext extends AtomContext {
        public TerminalNode NIL() {
            return getToken(CustomDjiControllerParser.NIL, 0);
        }

        public NilAtomContext(AtomContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterNilAtom(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitNilAtom(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitNilAtom(this);
            else return visitor.visitChildren(this);
        }
    }

    @SuppressWarnings("CheckReturnValue")
    public static class NumberAtomContext extends AtomContext {
        public TerminalNode INT() {
            return getToken(CustomDjiControllerParser.INT, 0);
        }

        public TerminalNode FLOAT() {
            return getToken(CustomDjiControllerParser.FLOAT, 0);
        }

        public NumberAtomContext(AtomContext ctx) {
            copyFrom(ctx);
        }

        @Override
        public void enterRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).enterNumberAtom(this);
        }

        @Override
        public void exitRule(ParseTreeListener listener) {
            if (listener instanceof CustomDjiControllerListener)
                ((CustomDjiControllerListener) listener).exitNumberAtom(this);
        }

        @Override
        public <T> T accept(ParseTreeVisitor<? extends T> visitor) {
            if (visitor instanceof CustomDjiControllerVisitor)
                return ((CustomDjiControllerVisitor<? extends T>) visitor).visitNumberAtom(this);
            else return visitor.visitChildren(this);
        }
    }

    public final AtomContext atom() throws RecognitionException {
        AtomContext _localctx = new AtomContext(_ctx, getState());
        enterRule(_localctx, 44, RULE_atom);
        int _la;
        try {
            setState(219);
            _errHandler.sync(this);
            switch (_input.LA(1)) {
                case T__2:
                    _localctx = new ParExprContext(_localctx);
                    enterOuterAlt(_localctx, 1);
                {
                    setState(210);
                    match(T__2);
                    setState(211);
                    expr(0);
                    setState(212);
                    match(T__3);
                }
                break;
                case INT:
                case FLOAT:
                    _localctx = new NumberAtomContext(_localctx);
                    enterOuterAlt(_localctx, 2);
                {
                    setState(214);
                    _la = _input.LA(1);
                    if (!(_la == INT || _la == FLOAT)) {
                        _errHandler.recoverInline(this);
                    } else {
                        if (_input.LA(1) == Token.EOF) matchedEOF = true;
                        _errHandler.reportMatch(this);
                        consume();
                    }
                }
                break;
                case TRUE:
                case FALSE:
                    _localctx = new BooleanAtomContext(_localctx);
                    enterOuterAlt(_localctx, 3);
                {
                    setState(215);
                    _la = _input.LA(1);
                    if (!(_la == TRUE || _la == FALSE)) {
                        _errHandler.recoverInline(this);
                    } else {
                        if (_input.LA(1) == Token.EOF) matchedEOF = true;
                        _errHandler.reportMatch(this);
                        consume();
                    }
                }
                break;
                case ID:
                    _localctx = new IdAtomContext(_localctx);
                    enterOuterAlt(_localctx, 4);
                {
                    setState(216);
                    match(ID);
                }
                break;
                case STRING:
                    _localctx = new StringAtomContext(_localctx);
                    enterOuterAlt(_localctx, 5);
                {
                    setState(217);
                    match(STRING);
                }
                break;
                case NIL:
                    _localctx = new NilAtomContext(_localctx);
                    enterOuterAlt(_localctx, 6);
                {
                    setState(218);
                    match(NIL);
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

    public boolean sempred(RuleContext _localctx, int ruleIndex, int predIndex) {
        switch (ruleIndex) {
            case 21:
                return expr_sempred((ExprContext) _localctx, predIndex);
        }
        return true;
    }

    private boolean expr_sempred(ExprContext _localctx, int predIndex) {
        switch (predIndex) {
            case 0:
                return precpred(_ctx, 10);
            case 1:
                return precpred(_ctx, 7);
            case 2:
                return precpred(_ctx, 6);
            case 3:
                return precpred(_ctx, 5);
            case 4:
                return precpred(_ctx, 4);
            case 5:
                return precpred(_ctx, 3);
            case 6:
                return precpred(_ctx, 2);
        }
        return true;
    }

    public static final String _serializedATN =
            "\u0004\u0001;\u00de\u0002\u0000\u0007\u0000\u0002\u0001\u0007\u0001\u0002" +
                    "\u0002\u0007\u0002\u0002\u0003\u0007\u0003\u0002\u0004\u0007\u0004\u0002" +
                    "\u0005\u0007\u0005\u0002\u0006\u0007\u0006\u0002\u0007\u0007\u0007\u0002" +
                    "\b\u0007\b\u0002\t\u0007\t\u0002\n\u0007\n\u0002\u000b\u0007\u000b\u0002" +
                    "\f\u0007\f\u0002\r\u0007\r\u0002\u000e\u0007\u000e\u0002\u000f\u0007\u000f" +
                    "\u0002\u0010\u0007\u0010\u0002\u0011\u0007\u0011\u0002\u0012\u0007\u0012" +
                    "\u0002\u0013\u0007\u0013\u0002\u0014\u0007\u0014\u0002\u0015\u0007\u0015" +
                    "\u0002\u0016\u0007\u0016\u0001\u0000\u0001\u0000\u0001\u0000\u0001\u0001" +
                    "\u0005\u00013\b\u0001\n\u0001\f\u00016\t\u0001\u0001\u0002\u0001\u0002" +
                    "\u0001\u0002\u0001\u0002\u0001\u0002\u0001\u0002\u0001\u0002\u0001\u0002" +
                    "\u0001\u0002\u0001\u0002\u0001\u0002\u0001\u0002\u0001\u0002\u0001\u0002" +
                    "\u0001\u0002\u0001\u0002\u0003\u0002H\b\u0002\u0001\u0003\u0001\u0003" +
                    "\u0001\u0003\u0001\u0003\u0001\u0003\u0001\u0004\u0001\u0004\u0001\u0004" +
                    "\u0001\u0004\u0001\u0004\u0001\u0004\u0001\u0005\u0001\u0005\u0001\u0005" +
                    "\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0006\u0001\u0006" +
                    "\u0001\u0007\u0001\u0007\u0001\u0007\u0001\u0007\u0001\u0007\u0001\u0007" +
                    "\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\b\u0001\t\u0001\t\u0001" +
                    "\t\u0001\t\u0001\t\u0001\t\u0001\n\u0001\n\u0001\n\u0001\n\u0001\n\u0001" +
                    "\n\u0001\u000b\u0001\u000b\u0001\u000b\u0001\u000b\u0001\u000b\u0001\f" +
                    "\u0001\f\u0001\f\u0001\f\u0001\f\u0001\r\u0001\r\u0001\r\u0001\r\u0001" +
                    "\r\u0001\r\u0001\u000e\u0001\u000e\u0001\u000e\u0001\u000e\u0001\u000e" +
                    "\u0001\u000e\u0001\u000f\u0001\u000f\u0001\u000f\u0001\u000f\u0001\u0010" +
                    "\u0001\u0010\u0001\u0010\u0001\u0010\u0001\u0010\u0001\u0011\u0001\u0011" +
                    "\u0001\u0011\u0001\u0011\u0001\u0011\u0005\u0011\u009a\b\u0011\n\u0011" +
                    "\f\u0011\u009d\t\u0011\u0001\u0011\u0001\u0011\u0003\u0011\u00a1\b\u0011" +
                    "\u0001\u0012\u0001\u0012\u0001\u0012\u0001\u0013\u0001\u0013\u0001\u0013" +
                    "\u0001\u0013\u0001\u0013\u0003\u0013\u00ab\b\u0013\u0001\u0014\u0001\u0014" +
                    "\u0001\u0014\u0001\u0014\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015" +
                    "\u0001\u0015\u0001\u0015\u0003\u0015\u00b7\b\u0015\u0001\u0015\u0001\u0015" +
                    "\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015" +
                    "\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015" +
                    "\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015\u0001\u0015" +
                    "\u0001\u0015\u0005\u0015\u00ce\b\u0015\n\u0015\f\u0015\u00d1\t\u0015\u0001" +
                    "\u0016\u0001\u0016\u0001\u0016\u0001\u0016\u0001\u0016\u0001\u0016\u0001" +
                    "\u0016\u0001\u0016\u0001\u0016\u0003\u0016\u00dc\b\u0016\u0001\u0016\u0000" +
                    "\u0001*\u0017\u0000\u0002\u0004\u0006\b\n\f\u000e\u0010\u0012\u0014\u0016" +
                    "\u0018\u001a\u001c\u001e \"$&(*,\u0000\u0012\u0001\u0000\u0001\u0002\u0001" +
                    "\u0000\u0006\u0007\u0001\u0000\b\t\u0001\u0000\n\u000b\u0001\u0000\f\r" +
                    "\u0001\u0000\u000e\u000f\u0001\u0000\u0010\u0011\u0001\u0000\u0012\u0013" +
                    "\u0001\u0000\u0014\u0015\u0001\u0000\u0016\u0017\u0001\u0000\u0018\u0019" +
                    "\u0001\u0000\u001a\u001b\u0001\u0000(*\u0001\u0000&\'\u0001\u0000\"%\u0001" +
                    "\u0000 !\u0001\u000067\u0001\u0000./\u00e7\u0000.\u0001\u0000\u0000\u0000" +
                    "\u00024\u0001\u0000\u0000\u0000\u0004G\u0001\u0000\u0000\u0000\u0006I" +
                    "\u0001\u0000\u0000\u0000\bN\u0001\u0000\u0000\u0000\nT\u0001\u0000\u0000" +
                    "\u0000\fW\u0001\u0000\u0000\u0000\u000e]\u0001\u0000\u0000\u0000\u0010" +
                    "c\u0001\u0000\u0000\u0000\u0012i\u0001\u0000\u0000\u0000\u0014o\u0001" +
                    "\u0000\u0000\u0000\u0016u\u0001\u0000\u0000\u0000\u0018z\u0001\u0000\u0000" +
                    "\u0000\u001a\u007f\u0001\u0000\u0000\u0000\u001c\u0085\u0001\u0000\u0000" +
                    "\u0000\u001e\u008b\u0001\u0000\u0000\u0000 \u008f\u0001\u0000\u0000\u0000" +
                    "\"\u0094\u0001\u0000\u0000\u0000$\u00a2\u0001\u0000\u0000\u0000&\u00aa" +
                    "\u0001\u0000\u0000\u0000(\u00ac\u0001\u0000\u0000\u0000*\u00b6\u0001\u0000" +
                    "\u0000\u0000,\u00db\u0001\u0000\u0000\u0000./\u0003\u0002\u0001\u0000" +
                    "/0\u0005\u0000\u0000\u00010\u0001\u0001\u0000\u0000\u000013\u0003\u0004" +
                    "\u0002\u000021\u0001\u0000\u0000\u000036\u0001\u0000\u0000\u000042\u0001" +
                    "\u0000\u0000\u000045\u0001\u0000\u0000\u00005\u0003\u0001\u0000\u0000" +
                    "\u000064\u0001\u0000\u0000\u00007H\u0003\u0006\u0003\u00008H\u0003\b\u0004" +
                    "\u00009H\u0003\n\u0005\u0000:H\u0003\f\u0006\u0000;H\u0003\u000e\u0007" +
                    "\u0000<H\u0003\u0010\b\u0000=H\u0003\u0012\t\u0000>H\u0003\u0014\n\u0000" +
                    "?H\u0003\u0016\u000b\u0000@H\u0003\u0018\f\u0000AH\u0003\u001a\r\u0000" +
                    "BH\u0003\u001c\u000e\u0000CH\u0003\u001e\u000f\u0000DH\u0003 \u0010\u0000" +
                    "EH\u0003\"\u0011\u0000FH\u0003(\u0014\u0000G7\u0001\u0000\u0000\u0000" +
                    "G8\u0001\u0000\u0000\u0000G9\u0001\u0000\u0000\u0000G:\u0001\u0000\u0000" +
                    "\u0000G;\u0001\u0000\u0000\u0000G<\u0001\u0000\u0000\u0000G=\u0001\u0000" +
                    "\u0000\u0000G>\u0001\u0000\u0000\u0000G?\u0001\u0000\u0000\u0000G@\u0001" +
                    "\u0000\u0000\u0000GA\u0001\u0000\u0000\u0000GB\u0001\u0000\u0000\u0000" +
                    "GC\u0001\u0000\u0000\u0000GD\u0001\u0000\u0000\u0000GE\u0001\u0000\u0000" +
                    "\u0000GF\u0001\u0000\u0000\u0000H\u0005\u0001\u0000\u0000\u0000IJ\u0007" +
                    "\u0000\u0000\u0000JK\u0005\u0003\u0000\u0000KL\u0005\u0004\u0000\u0000" +
                    "LM\u0005\u0005\u0000\u0000M\u0007\u0001\u0000\u0000\u0000NO\u0007\u0001" +
                    "\u0000\u0000OP\u0005\u0003\u0000\u0000PQ\u00056\u0000\u0000QR\u0005\u0004" +
                    "\u0000\u0000RS\u0005\u0005\u0000\u0000S\t\u0001\u0000\u0000\u0000TU\u0007" +
                    "\u0002\u0000\u0000UV\u0005\u0005\u0000\u0000V\u000b\u0001\u0000\u0000" +
                    "\u0000WX\u0007\u0003\u0000\u0000XY\u0005\u0003\u0000\u0000YZ\u00057\u0000" +
                    "\u0000Z[\u0005\u0004\u0000\u0000[\\\u0005\u0005\u0000\u0000\\\r\u0001" +
                    "\u0000\u0000\u0000]^\u0007\u0004\u0000\u0000^_\u0005\u0003\u0000\u0000" +
                    "_`\u0003*\u0015\u0000`a\u0005\u0004\u0000\u0000ab\u0005\u0005\u0000\u0000" +
                    "b\u000f\u0001\u0000\u0000\u0000cd\u0007\u0005\u0000\u0000de\u0005\u0003" +
                    "\u0000\u0000ef\u0003*\u0015\u0000fg\u0005\u0004\u0000\u0000gh\u0005\u0005" +
                    "\u0000\u0000h\u0011\u0001\u0000\u0000\u0000ij\u0007\u0006\u0000\u0000" +
                    "jk\u0005\u0003\u0000\u0000kl\u0003*\u0015\u0000lm\u0005\u0004\u0000\u0000" +
                    "mn\u0005\u0005\u0000\u0000n\u0013\u0001\u0000\u0000\u0000op\u0007\u0007" +
                    "\u0000\u0000pq\u0005\u0003\u0000\u0000qr\u0003*\u0015\u0000rs\u0005\u0004" +
                    "\u0000\u0000st\u0005\u0005\u0000\u0000t\u0015\u0001\u0000\u0000\u0000" +
                    "uv\u0007\b\u0000\u0000vw\u0005\u0003\u0000\u0000wx\u0005\u0004\u0000\u0000" +
                    "xy\u0005\u0005\u0000\u0000y\u0017\u0001\u0000\u0000\u0000z{\u0007\t\u0000" +
                    "\u0000{|\u0005\u0003\u0000\u0000|}\u0005\u0004\u0000\u0000}~\u0005\u0005" +
                    "\u0000\u0000~\u0019\u0001\u0000\u0000\u0000\u007f\u0080\u0007\n\u0000" +
                    "\u0000\u0080\u0081\u0005\u0003\u0000\u0000\u0081\u0082\u0003*\u0015\u0000" +
                    "\u0082\u0083\u0005\u0004\u0000\u0000\u0083\u0084\u0005\u0005\u0000\u0000" +
                    "\u0084\u001b\u0001\u0000\u0000\u0000\u0085\u0086\u0007\u000b\u0000\u0000" +
                    "\u0086\u0087\u0005\u0003\u0000\u0000\u0087\u0088\u0003*\u0015\u0000\u0088" +
                    "\u0089\u0005\u0004\u0000\u0000\u0089\u008a\u0005\u0005\u0000\u0000\u008a" +
                    "\u001d\u0001\u0000\u0000\u0000\u008b\u008c\u00054\u0000\u0000\u008c\u008d" +
                    "\u0003*\u0015\u0000\u008d\u008e\u0005\u0005\u0000\u0000\u008e\u001f\u0001" +
                    "\u0000\u0000\u0000\u008f\u0090\u00055\u0000\u0000\u0090\u0091\u0005-\u0000" +
                    "\u0000\u0091\u0092\u0003*\u0015\u0000\u0092\u0093\u0005\u0005\u0000\u0000" +
                    "\u0093!\u0001\u0000\u0000\u0000\u0094\u0095\u00051\u0000\u0000\u0095\u009b" +
                    "\u0003$\u0012\u0000\u0096\u0097\u00052\u0000\u0000\u0097\u0098\u00051" +
                    "\u0000\u0000\u0098\u009a\u0003$\u0012\u0000\u0099\u0096\u0001\u0000\u0000" +
                    "\u0000\u009a\u009d\u0001\u0000\u0000\u0000\u009b\u0099\u0001\u0000\u0000" +
                    "\u0000\u009b\u009c\u0001\u0000\u0000\u0000\u009c\u00a0\u0001\u0000\u0000" +
                    "\u0000\u009d\u009b\u0001\u0000\u0000\u0000\u009e\u009f\u00052\u0000\u0000" +
                    "\u009f\u00a1\u0003&\u0013\u0000\u00a0\u009e\u0001\u0000\u0000\u0000\u00a0" +
                    "\u00a1\u0001\u0000\u0000\u0000\u00a1#\u0001\u0000\u0000\u0000\u00a2\u00a3" +
                    "\u0003*\u0015\u0000\u00a3\u00a4\u0003&\u0013\u0000\u00a4%\u0001\u0000" +
                    "\u0000\u0000\u00a5\u00a6\u0005\u001c\u0000\u0000\u00a6\u00a7\u0003\u0002" +
                    "\u0001\u0000\u00a7\u00a8\u0005\u001d\u0000\u0000\u00a8\u00ab\u0001\u0000" +
                    "\u0000\u0000\u00a9\u00ab\u0003\u0004\u0002\u0000\u00aa\u00a5\u0001\u0000" +
                    "\u0000\u0000\u00aa\u00a9\u0001\u0000\u0000\u0000\u00ab\'\u0001\u0000\u0000" +
                    "\u0000\u00ac\u00ad\u00053\u0000\u0000\u00ad\u00ae\u0003*\u0015\u0000\u00ae" +
                    "\u00af\u0003&\u0013\u0000\u00af)\u0001\u0000\u0000\u0000\u00b0\u00b1\u0006" +
                    "\u0015\uffff\uffff\u0000\u00b1\u00b2\u0005\'\u0000\u0000\u00b2\u00b7\u0003" +
                    "*\u0015\t\u00b3\u00b4\u0005,\u0000\u0000\u00b4\u00b7\u0003*\u0015\b\u00b5" +
                    "\u00b7\u0003,\u0016\u0000\u00b6\u00b0\u0001\u0000\u0000\u0000\u00b6\u00b3" +
                    "\u0001\u0000\u0000\u0000\u00b6\u00b5\u0001\u0000\u0000\u0000\u00b7\u00cf" +
                    "\u0001\u0000\u0000\u0000\u00b8\u00b9\n\n\u0000\u0000\u00b9\u00ba\u0005" +
                    "+\u0000\u0000\u00ba\u00ce\u0003*\u0015\n\u00bb\u00bc\n\u0007\u0000\u0000" +
                    "\u00bc\u00bd\u0007\f\u0000\u0000\u00bd\u00ce\u0003*\u0015\b\u00be\u00bf" +
                    "\n\u0006\u0000\u0000\u00bf\u00c0\u0007\r\u0000\u0000\u00c0\u00ce\u0003" +
                    "*\u0015\u0007\u00c1\u00c2\n\u0005\u0000\u0000\u00c2\u00c3\u0007\u000e" +
                    "\u0000\u0000\u00c3\u00ce\u0003*\u0015\u0006\u00c4\u00c5\n\u0004\u0000" +
                    "\u0000\u00c5\u00c6\u0007\u000f\u0000\u0000\u00c6\u00ce\u0003*\u0015\u0005" +
                    "\u00c7\u00c8\n\u0003\u0000\u0000\u00c8\u00c9\u0005\u001f\u0000\u0000\u00c9" +
                    "\u00ce\u0003*\u0015\u0004\u00ca\u00cb\n\u0002\u0000\u0000\u00cb\u00cc" +
                    "\u0005\u001e\u0000\u0000\u00cc\u00ce\u0003*\u0015\u0003\u00cd\u00b8\u0001" +
                    "\u0000\u0000\u0000\u00cd\u00bb\u0001\u0000\u0000\u0000\u00cd\u00be\u0001" +
                    "\u0000\u0000\u0000\u00cd\u00c1\u0001\u0000\u0000\u0000\u00cd\u00c4\u0001" +
                    "\u0000\u0000\u0000\u00cd\u00c7\u0001\u0000\u0000\u0000\u00cd\u00ca\u0001" +
                    "\u0000\u0000\u0000\u00ce\u00d1\u0001\u0000\u0000\u0000\u00cf\u00cd\u0001" +
                    "\u0000\u0000\u0000\u00cf\u00d0\u0001\u0000\u0000\u0000\u00d0+\u0001\u0000" +
                    "\u0000\u0000\u00d1\u00cf\u0001\u0000\u0000\u0000\u00d2\u00d3\u0005\u0003" +
                    "\u0000\u0000\u00d3\u00d4\u0003*\u0015\u0000\u00d4\u00d5\u0005\u0004\u0000" +
                    "\u0000\u00d5\u00dc\u0001\u0000\u0000\u0000\u00d6\u00dc\u0007\u0010\u0000" +
                    "\u0000\u00d7\u00dc\u0007\u0011\u0000\u0000\u00d8\u00dc\u00055\u0000\u0000" +
                    "\u00d9\u00dc\u00058\u0000\u0000\u00da\u00dc\u00050\u0000\u0000\u00db\u00d2" +
                    "\u0001\u0000\u0000\u0000\u00db\u00d6\u0001\u0000\u0000\u0000\u00db\u00d7" +
                    "\u0001\u0000\u0000\u0000\u00db\u00d8\u0001\u0000\u0000\u0000\u00db\u00d9" +
                    "\u0001\u0000\u0000\u0000\u00db\u00da\u0001\u0000\u0000\u0000\u00dc-\u0001" +
                    "\u0000\u0000\u0000\t4G\u009b\u00a0\u00aa\u00b6\u00cd\u00cf\u00db";
    public static final ATN _ATN =
            new ATNDeserializer().deserialize(_serializedATN.toCharArray());

    static {
        _decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
        for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
            _decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
        }
    }
}