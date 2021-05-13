package org.firstinspires.ftc.teamcode.ultimategoal.util.math;

public class Matrix {
    private double[][] val;

    public Matrix(double[][] val) {
        this.val = val;
    }

    public double[][] getVal() {
        return val;
    }

    public int getNumRow() {
        return val.length;
    }

    public int getNumCol() {
        return val[0].length;
    }

    // note: row 0 and col 0 rather than row 1 and col 1
    public double getCell(int row, int col){
        return val[row][col];
    }

    public void setCell(int row, int col, double value){
        val[row][col] = value;
    }
}
