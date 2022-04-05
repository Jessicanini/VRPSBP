package base;

import ilog.concert.IloNumVar;

public class IloNumVarArray {

    // Creation of a new class similar to an ArrayList for CPLEX unknowns
    // taken from "cutsotck.java"
    int _num = 0;
    IloNumVar[] _array = new IloNumVar[32];

    public void add(IloNumVar ivar) {
        if (_num >= _array.length) {
            IloNumVar[] array = new IloNumVar[2 * _array.length];
            System.arraycopy(_array, 0, array, 0, _num);
            _array = array;
        }
        _array[_num] = ivar;
        _array[_num].setName("x" + String.valueOf(_num));
        _num += 1;
    }

    public IloNumVar getElement(int i) {
        return _array[i];
    }

    public int getSize() {
        return _num;
    }

}
