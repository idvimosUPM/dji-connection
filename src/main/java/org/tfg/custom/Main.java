package org.tfg.custom;

import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import org.tfg.custom.gen.CustomDjiControllerLexer;
import org.tfg.custom.gen.CustomDjiControllerParser;
import org.tfg.custom.listener.CustomDjiControllerListener;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class Main {
    public static void main(String[] args) throws IOException {

        // Start Webots
        startWebots("/Users/TFG/Documents/TFG/backend/tfg-webots/worlds/mavic2-tfg.wbt");

        CustomDjiController controller = new CustomDjiController();

        // Instructions file uploaded
        String code = new String(Files.readAllBytes(Paths.get("/Users/TFG/Documents/TFG/backend/dji-connection/src/main/resources/instructions/instructions.txt")));

        // Lexer and Parser
        CustomDjiControllerLexer lexer = new CustomDjiControllerLexer(CharStreams.fromString(code));
        CommonTokenStream tokens = new CommonTokenStream(lexer);
        CustomDjiControllerParser parser = new CustomDjiControllerParser(tokens);

        // Parsing
        CustomDjiControllerParser.ProgramContext tree = parser.program();

        // Listener
        ParseTreeWalker walker = new ParseTreeWalker();
        CustomDjiControllerListener listener = new CustomDjiControllerListener(controller);
        walker.walk(listener, tree);
    }

    private static void startWebots(String worldFilePath) {
        try {
            ProcessBuilder processBuilder = new ProcessBuilder("/Applications/Webots.app/Contents/MacOS/webots", worldFilePath);
            processBuilder.start();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}