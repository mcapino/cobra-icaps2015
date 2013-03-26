package cz.agents.deconfliction.util;

import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

public class Vector extends Vector3d {

    private static final long serialVersionUID = -2185025482385772759L;

    public Vector(double x, double y, double z) {
        super(x, y, z);
    }

    public Vector(double[] p) {
        super(p);
    }

    public Vector(Vector p1) {
        super(p1);
    }

    public Vector(Vector3f p1) {
        super(p1);
    }

    public Vector(Tuple3f t1) {
        super(t1);
    }

    public Vector(Tuple3d t1) {
        super(t1);
    }

    public Vector() {
        super();
    }
    
    public static Vector subtract(Tuple3d v1, Tuple3d v2) {
    	Vector result = new Vector(v1);
    	result.sub(v2);
    	return result;
    }
    
}
