package contractNet;

import java.util.Arrays;
//import contractNet.contractNet.pathPlanner;

public class masTest {
    public static void main(String[] args) {
        String content = "transport,start-10 end-33";
        String title = content.split(",")[0];
        System.out.println(title);
        String java_text = content.split(",")[1];
        System.out.println(java_text);
    }
}