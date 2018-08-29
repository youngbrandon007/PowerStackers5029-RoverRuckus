package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Utils.control;

import java.util.ArrayList;


public class SuperArrayList<E> extends ArrayList<E> {

    @Override
    public E get(int index) {
        if (index < 0) {
            return super.get(this.size() + index);
        }
        return super.get(index);
    }

}
