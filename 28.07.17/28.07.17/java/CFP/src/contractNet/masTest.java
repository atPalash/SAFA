package contractNet;

import java.io.*;
import java.nio.file.*;
import java.util.Objects;

/**
 * This program demonstrates how to write characters to a text file
 * using a BufferedReader for efficiency.
 * @author www.codejava.net
 *
 */
public class masTest {

    public static void main(String[] args) {
        try {
            String fs = System.getProperty("file.separator");
            String path_w = "C:" + fs + "Users" + fs + "halder" + fs + "Desktop" + fs + "testingFiles" + fs + "27.07.17 - Combined" + fs + "mobile" + fs + "java_reply.txt";
            FileWriter writer = new FileWriter(path_w, false);
            BufferedWriter bufferedWriter = new BufferedWriter(writer);

            bufferedWriter.write("start-10 end-03");
            bufferedWriter.close();
            WatchService watcher = FileSystems.getDefault().newWatchService();
            Path dir = Paths.get("C:" + fs + "Users" + fs + "halder" + fs + "Desktop" + fs + "testingFiles" + fs + "27.07.17 - Combined" + fs + "mobile");
            dir.register(watcher, StandardWatchEventKinds.ENTRY_MODIFY);
            int count = 0;
            while (true){
                count++;
                WatchKey key;
                try {
                    // wait for a key to be available
                    key = watcher.take();
                } catch (InterruptedException ex) {
                    return;
                }
                for (WatchEvent<?> event : key.pollEvents()) {

                    // get event type
                    WatchEvent.Kind<?> kind = event.kind();

                    // get file name
                    @SuppressWarnings("unchecked")
                    WatchEvent<Path> ev = (WatchEvent<Path>) event;
                    Path fileName = ev.context();
                    if(count%2==0){
                        if (kind == StandardWatchEventKinds.OVERFLOW) {
                            System.out.println("overflow error");
                        }
                        else if (kind == StandardWatchEventKinds.ENTRY_MODIFY && Objects.equals(fileName.toString(), "python_reply.txt")) {
                            System.out.println("File modified" + fileName);
                            String path_r = "C:" + fs + "Users" + fs + "halder" + fs + "Desktop" + fs + "testingFiles" + fs + "27.07.17 - Combined" + fs + "mobile" + fs + "python_reply.txt";
                            BufferedReader fr = new BufferedReader(new FileReader(path_r));
                            String line = fr.readLine();
                            System.out.println(line);

                            if(Objects.equals(line, "reached start")){
                                System.out.println("hello" + line);
                            }
                            else if(Objects.equals(line, "loaded start")){
                                System.out.println(line);
                            }
                        }
                    }
                }
                // IMPORTANT: The key must be reset after processed
                boolean valid = key.reset();
                if (!valid) {
                    break;
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
}