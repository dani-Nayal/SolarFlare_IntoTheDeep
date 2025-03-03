package org.firstinspires.ftc.teamcode.base.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class SubmersibleCycleAuto extends OpMode {
    public double[][] values = new double[3][3];
    public int selectedRow = 0;
    public int selectedColumn = 0;
    public boolean dpadPressed = false;

    public String[][] labelArray = new String[][]{new String[]{"Sample 1 X","Sample 1 Y","Sample 1 Deg"}, new String[]{"Sample 2 X","Sample 2 Y","Sample 2 Deg"}, new String[]{"Sample 3 X","Sample 3 Y","Sample 3 Deg"}};

    @Override
    public void init() {}

    @Override
    public void init_loop() {

        if (gamepad1.dpad_up && selectedRow>0&&!dpadPressed){
            selectedRow-=1;
        }
        else if (gamepad1.dpad_down && selectedRow<2&&!dpadPressed){
            selectedRow+=1;
        }
        else if (gamepad1.dpad_left && selectedColumn>0&&!dpadPressed){
            selectedRow-=1;
        }
        else if (gamepad1.dpad_right && selectedColumn<2&&!dpadPressed){
            selectedRow+=1;
        }
        dpadPressed = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;

        if (!labelArray[selectedRow][selectedColumn].startsWith("[")){
            labelArray[selectedRow][selectedColumn]="["+labelArray[selectedRow][selectedColumn]+"]";
        }

        if (gamepad1.right_trigger>0){
            if (selectedColumn!=3){
                values[selectedRow][selectedColumn]+=0.2;
            }
            else{
                if (values[selectedRow][selectedColumn]<88) {
                    values[selectedRow][selectedColumn] += 2;
                }
                else{
                    values[selectedRow][selectedColumn] = 90;
                }
            }
        }
        else if (gamepad1.left_trigger>0){
            if (selectedColumn!=3){
                values[selectedRow][selectedColumn]-=0.2;
            }
            else{
                if (values[selectedRow][selectedColumn]>-88) {
                    values[selectedRow][selectedColumn] -= 2;
                }
                else{
                    values[selectedRow][selectedColumn] = -90;
                }
            }
        }
        else if (gamepad1.right_bumper){
            if (selectedColumn!=3){
                values[selectedRow][selectedColumn]+=0.01;
            }
            else{
                if (values[selectedRow][selectedColumn]<89.75) {
                    values[selectedRow][selectedColumn] += 0.25;
                }
                else{
                    values[selectedRow][selectedColumn] = 90;
                }
            }
        }
        else if (gamepad1.left_bumper){
            if (selectedColumn!=3){
                values[selectedRow][selectedColumn]-=0.01;
            }
            else{
                if (values[selectedRow][selectedColumn]>-89.75) {
                    values[selectedRow][selectedColumn] -= 0.25;
                }
                else{
                    values[selectedRow][selectedColumn] = -90;
                }
            }
        }

        telemetry.addData(labelArray[0][0]+": " + values[0][0]+"    "+labelArray[0][1]+": " + values[0][1]+"    "+labelArray[0][2]+": " + values[0][2]+"    ","");
        telemetry.addData(labelArray[1][0]+": " + values[1][0]+"    "+labelArray[1][1]+": " + values[1][1]+"    "+labelArray[1][2]+": " + values[1][2]+"    ","");
        telemetry.addData(labelArray[2][0]+": " + values[2][0]+"    "+labelArray[2][1]+": " + values[2][1]+"    "+labelArray[2][2]+": " + values[2][2]+"    ","");
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}
