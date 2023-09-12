package org.firstinspires.ftc.teamcode.DataTypes;

public class booleanInt {
    public boolean bool;
    public int num;

    public booleanInt(boolean bool, int num) {
        this.bool = bool;
        this.num = num;
    }

    public boolean getBool() {
        return this.bool;
    }

    public int getNum() {
        return this.num;
    }

    public void setNum(int num) {
        this.num = num;
    }

    public void setBool(boolean bool) {
        this.bool = bool;
    }
}