package contractNet;

import java.util.Objects;

public class masTest2 {
    public static void main(String[] args) {
        int[][] gridArray = {{0, 0, 0, 0},
                            {0, 0, 0, 0},
                            {0, 0, 0, 0},
                            {0, 0, 0, 0}};
        String[][] optimumPolicy = {{"R", "R", "R", "D"}, {"R", "R", "R", "D"}, {"R", "R", "R", "D"}, {"R", "R", "R", "*"}};
//        int[] iniCoor = {3, 3};
//        int[] resCoor = {1, 0};
//        int[] tempCoor = resCoor.clone();
//        int x_step_pos = 0;
//        int x_step_neg = 0;
//        int y_step_pos = 0;
//        int y_step_neg = 0;
//        int step = 0;
//        String direction;
////        System.out.println(tempCoor[0]);
////        System.out.println(tempCoor[1]);
////        System.out.println(Objects.equals(optimumPolicy[tempCoor[0]][tempCoor[1]], "L"));
//        while(true){
//                if (Objects.equals(optimumPolicy[tempCoor[0]][tempCoor[1]], "*")){
//                    System.out.println("steps" + step);
//                    break;
//                }
//                else{
//                    direction = optimumPolicy[tempCoor[0]][tempCoor[1]];
//                    System.out.println("temp" + tempCoor[0]+tempCoor[1]);
//                    System.out.println(direction);
//                    if (Objects.equals(direction, "U")){
//                        if (tempCoor[0] - 1 >= 0){
//                            tempCoor[0]--;
//                            System.out.println(tempCoor[0]);
//                        }
//                    }
//                    else if(Objects.equals(direction, "D")){
//                        if(tempCoor[0] + 1 <= 3){
//                            tempCoor[0]++;
//                        }
//                    }
//                    else if(Objects.equals(direction, "L")){
//                        if (tempCoor[1] - 1 >=0){
//                            tempCoor[1]--;
//                        }
//                    }
//                    else if(Objects.equals(direction, "R")){
//                        if(tempCoor[1] + 1 <= 3){
//                            tempCoor[1]++;
//                        }
//                    }
//                }
//                step++;
//        }
    }
}
