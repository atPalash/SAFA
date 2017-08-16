package contractNet;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URL;
import java.net.URLConnection;
import java.util.Arrays;
import java.util.Objects;

public class pathPlanner {
    public int[][] gridValue(){
        int[][] gridArray = new int[4][4];
        try {
            URL path_url = new URL("http://130.230.147.89:8000/path");
            URLConnection yc = path_url.openConnection();
            BufferedReader in = new BufferedReader(new InputStreamReader(yc.getInputStream()));
            String inputLine;
            StringBuilder path = new StringBuilder();
            while ((inputLine = in.readLine()) != null)
                path.append(inputLine);
            String gridString = path.toString();
            int col = 0;
            int row = 0;
            for (int i = 0; i < gridString.length(); i++){
                if ((gridString.charAt(i) != '[') && (gridString.charAt(i) != ']') && (gridString.charAt(i) != ',') && (gridString.charAt(i) != ' ')&& (row <= 3 && col <=3)){
                    gridArray[row][col] = Character.getNumericValue(gridString.charAt(i));
                    col++;
                    if (col > 3){
                        col = 0;
                        row ++;
                    }
                }
            }
            in.close();
        }  catch (IOException e) {
            e.printStackTrace();
        }
        return gridArray;
    }
    public String[][] optimumPolicy (int[] goalCoordinates){
        int[][]gridArray = gridValue();
        int cost = 1;
        int[][] delta = {{-1,0}, {0,-1}, {1,0}, {0,1}};
        String[] deltaName = {"U", "L", "D", "R"};
        int[][] value = new int[4][4];
        for (int[] row: value)
            Arrays.fill(row, 99);
        String[][] policy = new String[4][4];
        for (String[] row: policy)
            Arrays.fill(row, " ");
        boolean change = true;

        while (change){
            change = false;
            for (int x = 0; x <gridArray.length; x++){
                for (int y =0; y < gridArray[0].length; y++){
                    if (goalCoordinates[0] == x && goalCoordinates[1] == y){
                        if (value[x][y] > 0){
                            value[x][y] = 0;
                            policy[x][y] = "*";
                            change = true;
                        }
                    }
                    else if (gridArray[x][y] == 0){
                        for (int a = 0; a < delta.length; a++){
                            int x2 = x + delta[a][0];
                            int y2 = y + delta[a][1];

                            if (x2 >= 0 && x2 < gridArray.length && y2 >= 0 && y2 < gridArray[0].length && gridArray[x2][y2] == 0){
                                int v2 = value[x2][y2] + cost;
                                if (v2 < value[x][y]){
                                    change = true;
                                    value[x][y] = v2;
                                    policy[x][y] = deltaName[a];
                                }
                            }
                        }
                    }
                }
            }
        }
        return policy;
    }
    public int stepsToGoal(int[] goalCoordinates, int[] startCoordinates){
        String[][] policy = optimumPolicy(goalCoordinates);
        int[] tempCoor = startCoordinates.clone();
        int step = 0;
        String direction;
        while(true){
            if (Objects.equals(policy[tempCoor[0]][tempCoor[1]], "*")){
//                System.out.println("steps" + step);
                break;
            }
            else{
                direction = policy[tempCoor[0]][tempCoor[1]];
//                System.out.println("temp" + tempCoor[0]+tempCoor[1]);
//                System.out.println(direction);
                if (Objects.equals(direction, "U")){
                    if (tempCoor[0] - 1 >= 0){
                        tempCoor[0]--;
//                        System.out.println(tempCoor[0]);
                    }
                }
                else if(Objects.equals(direction, "D")){
                    if(tempCoor[0] + 1 <= 3){
                        tempCoor[0]++;
                    }
                }
                else if(Objects.equals(direction, "L")){
                    if (tempCoor[1] - 1 >=0){
                        tempCoor[1]--;
                    }
                }
                else if(Objects.equals(direction, "R")){
                    if(tempCoor[1] + 1 <= 3){
                        tempCoor[1]++;
                    }
                }
                else{
                    continue;
                }
            }
            step++;
        }
        return step;
    }
}
